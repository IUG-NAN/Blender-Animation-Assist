"""Borrow one frame of control channels from another Action on this rig."""
import bpy
from bpy_extras.anim_utils import action_get_channelbag_for_slot
from mathutils import Matrix, Quaternion

from . import arp_pose_assist, body_pose_assist, chain_motion, pivot_rotate
from .key_edit import channelbag, key_transaction


class BorrowSettings(bpy.types.PropertyGroup):
    action: bpy.props.StringProperty(name="コピー元Action")
    slot: bpy.props.StringProperty(name="元スロット")
    pelvis: bpy.props.StringProperty(name="コピーする骨盤", description="この骨盤自身はコピーし、親の配置階層を除外します。空欄は自動判定")
    frame: bpy.props.IntProperty(name="コピー元F", default=0)


def source_slot(action, settings, dest_slot):
    slots = [s for s in action.slots if s.target_id_type == "OBJECT"]
    if len(slots) == 1:
        return slots[0]
    if settings.slot:
        found = next((s for s in slots if settings.slot in (s.identifier, s.name_display)), None)
        if found:
            return found
    elif dest_slot:
        found = next((s for s in slots if s.identifier == dest_slot.identifier), None)
        if found:
            return found
    raise ValueError("コピー元のアーマチュア用スロットを指定してください")


def paths(bone):
    return ("location", chain_motion.rotation_path(bone), "scale")


def locked(bone, prop, index):
    if prop == "location":
        return bone.lock_location[index] or bone.bone.use_connect
    if prop == "scale":
        return bone.lock_scale[index]
    if prop == "rotation_euler":
        return bone.lock_rotation[index]
    return any(bone.lock_rotation) or (bone.lock_rotations_4d and bone.lock_rotation_w)


def pose_boundary(rig, name):
    bones = rig.pose.bones
    pelvis = bones.get("c_root_master.x")
    if pelvis is None:
        if name:
            pelvis = bones.get(name)
            if pelvis is None:
                raise ValueError("コピーする骨盤が見つかりません")
        else:
            pelvis = next((bones[n] for n in ("pelvis", "Pelvis", "hips", "Hips") if n in bones), None)
    if pelvis:
        return pelvis, pelvis.parent
    return None, next((bones[n] for n in ("c_spin", "root", "Root", "ROOT") if n in bones), None)


def body_controls(rig):
    bones = rig.pose.bones
    if "c_root_master.x" in bones and "c_spin" in bones:
        # Custom character controls may also belong to Deform and Reference.
        # Shape-less internal shape-transform and mechanism bones are not pose inputs.
        return {b.name for b in bones if b.custom_shape or b.bone.get("cc")
                or any(col.name in {"Main", "Secondary", "RigTweak"} for col in b.bone.collections)}
    return {b.name for b in bones if not b.name.startswith(("MCH-", "ORG-"))}


def rest_value(prop, index):
    if prop == "scale" or (prop == "rotation_quaternion" and index == 0):
        return 1.0
    if prop == "rotation_axis_angle" and index == 2:
        return 1.0
    return 0.0


def pose_channels(rig, source, dest, frame, included):
    """Read only pose paths shared with this rig, never object transforms or rig UI settings."""
    controls = {}
    drivers = {f.data_path for f in rig.animation_data.drivers}
    result = {}
    for name in included:
        b = rig.pose.bones[name]
        for prop in paths(b):
            path = b.path_from_id(prop)
            controls[path] = (b, prop, False)
            if path not in drivers:
                for i, current in enumerate(getattr(b, prop)):
                    neutral = rest_value(prop, i)
                    if current != neutral and not locked(b, prop, i):
                        result[name, prop, i] = neutral
        for prop in b.keys():
            if isinstance(b[prop], (float, int)) and prop != "pole_viz":
                controls[data_path(b, prop, None)] = (b, prop, True)
    matched = set()
    for bag in (dest, source):
        if not bag:
            continue
        for fc in bag.fcurves:
            control = controls.get(fc.data_path)
            if not control or fc.data_path in drivers:
                continue
            b, prop, custom = control
            i = None if custom else fc.array_index
            if not custom and (i >= len(getattr(b, prop)) or locked(b, prop, i)):
                continue
            if fc.mute or (fc.group and fc.group.mute):
                continue
            if not (fc.keyframe_points or fc.sampled_points or fc.modifiers):
                continue
            if bag == source:
                value = fc.evaluate(frame)
                if not custom:
                    matched.add(b.name)
            elif custom:
                default = b.id_properties_ui(prop).as_dict().get("default")
                if default is None:
                    continue
                value = default
            else:
                value = rest_value(prop, i)
            result[b.name, prop, i] = type(b[prop])(value) if custom else value
    return result, matched


def data_path(b, prop, index):
    return b.path_from_id() + '["' + bpy.utils.escape_identifier(prop) + '"]' if index is None else b.path_from_id(prop)


def put(rig, values):
    for (name, prop, i), value in values.items():
        b = rig.pose.bones[name]
        if i is None:
            b[prop] = value
        else:
            getattr(b, prop)[i] = value


def read(rig, keys):
    return {(n, p, i): rig.pose.bones[n][p] if i is None else getattr(rig.pose.bones[n], p)[i]
            for n, p, i in keys}


def independent_controls(rig, values, protected):
    names = {n for n, _, _ in values}
    # IK constraints can target a control parented outside the placement hierarchy.
    targets = {n for b in rig.pose.bones for con in b.constraints if con.type == "IK"
               for obj, n in ((con.target, con.subtarget), (con.pole_target, con.pole_subtarget))
               if obj == rig and n}
    return sorted((b for b in rig.pose.bones if b.name not in protected
                   and (b.name in names or b.name in targets)
                   and (not b.parent or b.name in targets
                        or b.name.startswith(("c_hand_ik.", "c_foot_ik.", "c_arms_pole.", "c_leg_pole.")))),
                  key=lambda b: ("pole" in b.name.lower(), len(b.parent_recursive)))


def transform_keys(b):
    return [(b.name, p, i) for p in paths(b) for i in range(len(getattr(b, p)))]


def prepare_pose(c, root, values, protected):
    rig = c.object
    # A native snap can touch inactive controls as well as the selected hand/foot.
    switches = []
    keys = set(values)
    for (n, p, i), mode in values.items():
        if p == "ik_fk_switch" and mode != rig.pose.bones[n][p] and n.startswith(("c_hand_ik.", "c_foot_ik.")):
            native = arp_pose_assist.rig_functions(c)
            limb = "arm" if n.startswith("c_hand") else "leg"
            target_mode = rig.pose.bones[n][p]
            if mode not in (0, 1) or target_mode not in (0, 1):
                raise ValueError("IK/FKのブレンド中はポーズを流用できません")
            side = native.get_bone_side(n)
            names = getattr(native, "fk_" + limb) + getattr(native, "ik_" + limb)
            for name in names:
                b = rig.pose.bones.get(name + side)
                if b and name.startswith("c_"):
                    keys.update(transform_keys(b))
                    keys.update((b.name, k, None) for k in ("auto_stretch", "stretch_length") if k in b)
            snap = getattr(native, ("fk_to_ik_" if target_mode else "ik_to_fk_") + limb)
            switches.append((snap, side))
    controls = independent_controls(rig, values, protected) if root else []
    for b in controls:
        keys.update(transform_keys(b))
    roots = {k: values[k] for k in values if k[0] in protected}
    for name in protected:
        for k in transform_keys(rig.pose.bones[name]):
            roots.setdefault(k, rest_value(k[1], k[2]))
    keys.update(roots)
    before = read(rig, keys)
    auto = c.scene.tool_settings.use_keyframe_insert_auto
    mirror = rig.pose.use_mirror_x
    active = rig.data.bones.active
    selected = {b.name for b in rig.pose.bones if b.select}
    try:
        c.scene.tool_settings.use_keyframe_insert_auto = False
        rig.pose.use_mirror_x = False
        dest_anchor = root.matrix.copy() if root else Matrix.Identity(4)
        put(rig, values)
        put(rig, roots)
        c.view_layer.update()
        if root:
            delta = dest_anchor @ root.matrix.inverted()
            targets = [(b, delta @ b.matrix) for b in controls]
            put(rig, {k: before[k] for k in roots})
            c.view_layer.update()
            for b, target in targets:
                current = b.matrix.copy()
                if max(abs(v - w) for row, other in zip(current, target)
                       for v, w in zip(row, other)) < 1e-5:
                    continue
                # The evaluated matrix includes Child Of and its inverse. Apply only
                # the residual after the destination root and earlier goals have moved.
                basis = b.matrix_basis @ current.inverted() @ target
                old = read(rig, transform_keys(b))
                b.matrix_basis = basis
                put(rig, {k: v for k, v in old.items() if locked(b, k[1], k[2])})
                c.view_layer.update()
        # Native mode matching also sets the locked transverse axes of the FK hinge.
        # Keep those solver outputs so an old hidden twist cannot reappear on replay.
        solver_keys = {k for k in keys if k[2] is not None and k[0].startswith(("c_leg_fk.", "c_forearm_fk."))
                       and k[1] == "rotation_euler" and locked(rig.pose.bones[k[0]], k[1], k[2])}
        pre_snap = read(rig, solver_keys) if switches else {}
        for snap, side in switches:
            snap(rig, side, add_keyframe=False)
            c.view_layer.update()
        result = {k: v for k, v in read(rig, keys).items()
                  if k[0] not in protected and (k in values or abs(v - before[k]) > 1e-6)
                  and (k[2] is None or not locked(rig.pose.bones[k[0]], k[1], k[2])
                       or (k in pre_snap and abs(v - pre_snap[k]) > 1e-6))}
        for k in list(result):
            if k[1] == "ik_fk_switch" and result[k] == before[k]:
                del result[k]
        return result
    finally:
        put(rig, before)
        for b in rig.pose.bones:
            if b.select != (b.name in selected):
                b.select = b.name in selected
        rig.data.bones.active = active
        rig.pose.use_mirror_x = mirror
        c.scene.tool_settings.use_keyframe_insert_auto = auto
        c.view_layer.update()


class BorrowPose:
    def __init__(self, c):
        self.rig = rig = c.object
        self.scene = c.scene
        self.frame = c.scene.frame_current
        ad = rig.animation_data
        if not ad or not ad.action or not ad.action_slot or ad.use_tweak_mode or c.scene.frame_subframe:
            raise ValueError("貼り付け先Actionを開き、整数フレームで使用してください")
        if (c.screen and c.screen.is_animation_playing or body_pose_assist._session
                or pivot_rotate.is_active(c.scene, rig)):
            raise ValueError("再生・PoseAssist・Pivotを終了してからプレビューしてください")
        self.edit = chain_motion.selected_edit(c)
        self.action, self.slot = ad.action, ad.action_slot
        settings = c.window_manager.baa_borrow
        source = bpy.data.actions.get(settings.action)
        if not source or source == self.action:
            raise ValueError("作業中とは別のコピー元Actionを選択してください")
        slot = source_slot(source, settings, self.slot)
        bag = action_get_channelbag_for_slot(source, slot)
        if not bag:
            raise ValueError("コピー元スロットにカーブがありません")
        pelvis, root = pose_boundary(rig, settings.pelvis)
        protected = {b.name for b in [root, *root.parent_recursive]} if root else set()
        included = body_controls(rig)
        if pelvis:
            included.add(pelvis.name)
        values, matched = pose_channels(rig, bag, channelbag(rig), settings.frame, included | protected)
        if not matched - protected:
            raise ValueError("コピー元に、このリグと対応する全身ポーズのカーブがありません")
        if self.edit:
            self.edit.sampling = True
        try:
            prepared = prepare_pose(c, root, values, protected)
            self.values = {k: v for k, v in prepared.items() if k[0] in included}
        finally:
            if self.edit:
                self.edit.sampling = False
        self.pelvis_label = pelvis.name if pelvis else ""
        self.before = read(rig, self.values)
        for name in {k[0] for k in self.values}:
            b = rig.pose.bones[name]
            keys = [(name, "rotation_quaternion", i) for i in range(4)]
            if all(k in self.values for k in keys):
                q = Quaternion([self.values[k] for k in keys])
                if q.magnitude < 1e-7:
                    raise ValueError(f"{name} のコピー元回転を評価できません")
                q.normalize()
                q.make_compatible(b.rotation_quaternion)
                self.values.update(zip(keys, q))
        self.label = f"{source.name} ({settings.frame}F) → {self.action.name} ({self.frame}F)"
        self.validate(c)

    def validate(self, c):
        ad = self.rig.animation_data
        if (c.object != self.rig or c.scene != self.scene or self.scene.frame_current != self.frame or self.scene.frame_subframe
                or not ad or ad.action != self.action or ad.action_slot != self.slot):
            raise ValueError("貼り付け先が変わりました。プレビューをやり直してください")
        if ad.use_tweak_mode:
            raise ValueError("Action Editorで対象Actionを開いてください")
        writes = {(data_path(self.rig.pose.bones[n], p, i), i or 0)
                  for n, p, i in self.values}
        if any((f.data_path, f.array_index) in writes for f in ad.drivers):
            raise ValueError("ドライバーで制御されているチャンネルにキーを記録できません")
        bag = channelbag(self.rig)
        if bag:
            for fc in bag.fcurves:
                if (fc.data_path, fc.array_index) in writes and (
                        fc.lock or fc.mute or fc.modifiers or fc.sampled_points or
                        (fc.group and (fc.group.lock or fc.group.mute))):
                    raise ValueError(f"{fc.data_path} にキーを記録できません")
            names = {n for n, _, _ in self.values}
            if any(g.name in names and (g.lock or g.mute) for g in bag.groups):
                raise ValueError("対象ボーンのチャンネルグループがロックまたはミュートされています")

    def preview(self, c):
        if self.edit:
            self.edit.sampling = True
        put(self.rig, self.values)
        c.view_layer.update()

    def restore(self, c):
        ad = self.rig.animation_data
        if (ad and ad.action == self.action and ad.action_slot == self.slot and
                self.scene.frame_current == self.frame and not self.scene.frame_subframe):
            put(self.rig, self.before)
            c.view_layer.update()
        if self.edit:
            self.edit.sampling = False

    def commit(self, c):
        self.validate(c)
        writes = {data_path(self.rig.pose.bones[n], p, i) for n, p, i in self.values}
        with key_transaction(self.rig, writes):
            put(self.rig, self.values)
            for n, prop, i in self.values:
                b = self.rig.pose.bones[n]
                path = '["' + bpy.utils.escape_identifier(prop) + '"]' if i is None else prop
                if not b.keyframe_insert(data_path=path, index=-1 if i is None else i, frame=self.frame, group=n):
                    raise ValueError(f"{n} のキーを記録できません")
            c.view_layer.update()
        if self.edit:
            self.edit.sampling = False
            if chain_motion._edit is self.edit:
                chain_motion.rebase(c, self.edit)


class POSE_OT_baa_borrow(bpy.types.Operator):
    bl_idname = "pose.baa_borrow"
    bl_label = "全身ポーズを流用"
    bl_options = {"UNDO"}

    @classmethod
    def poll(cls, c):
        return c.mode == "POSE" and c.object and c.object.type == "ARMATURE"

    def invoke(self, c, event):
        try:
            self.state = BorrowPose(c)
            self.state.preview(c)
            return c.window_manager.invoke_props_dialog(self, width=370, confirm_text="現在Fにキーを記録")
        except (ValueError, RuntimeError, KeyError) as e:
            self.cancel(c)
            self.report({"WARNING"}, str(e))
            return {"CANCELLED"}

    def draw(self, c):
        if self.state.pelvis_label:
            self.layout.label(text=f"コピーする骨盤   {self.state.pelvis_label}", icon="BONE_DATA")
        self.layout.label(text=self.state.label)

    def cancel(self, c):
        if hasattr(self, "state"):
            self.state.restore(c)

    def execute(self, c):
        try:
            if not hasattr(self, "state"):
                self.state = BorrowPose(c)
            self.state.commit(c)
        except (ValueError, RuntimeError, KeyError) as e:
            self.cancel(c)
            self.report({"WARNING"}, str(e))
            return {"CANCELLED"}
        return {"FINISHED"}


def draw(layout, c):
    header, box = layout.panel("baa_borrow_pose", default_closed=True)
    header.label(text="別Actionから全身ポーズを流用")
    if box:
        s = c.window_manager.baa_borrow
        box.prop_search(s, "action", bpy.data, "actions")
        action = bpy.data.actions.get(s.action)
        if action and len(action.slots) > 1:
            box.prop_search(s, "slot", action, "slots")
        box.prop(s, "frame")
        if "c_root_master.x" in c.object.pose.bones:
            box.label(text="コピーする骨盤   c_root_master.x", icon="BONE_DATA")
        else:
            pelvis, _ = pose_boundary(c.object, "")
            label = f"コピーする骨盤（自動: {pelvis.name}）" if pelvis and not s.pelvis else "コピーする骨盤"
            box.prop_search(s, "pelvis", c.object.pose, "bones", text=label)
        box.operator_context = "INVOKE_DEFAULT"
        box.operator("pose.baa_borrow", text="現在Fでプレビュー", icon="POSE_HLT")


def register():
    bpy.utils.register_class(BorrowSettings)
    bpy.utils.register_class(POSE_OT_baa_borrow)
    bpy.types.WindowManager.baa_borrow = bpy.props.PointerProperty(type=BorrowSettings)


def unregister():
    del bpy.types.WindowManager.baa_borrow
    bpy.utils.unregister_class(POSE_OT_baa_borrow)
    bpy.utils.unregister_class(BorrowSettings)
