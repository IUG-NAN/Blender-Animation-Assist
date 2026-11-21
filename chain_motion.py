bl_info = {
    "name": "Chain Motion",
    "author": "Codex",
    "version": (0, 5, 0),
    "blender": (5, 2, 0),
    "location": "3D View > Pose Assist > チェーンモーション",
    "category": "Animation",
}

import re
from math import atan2, ceil, floor, radians
from types import SimpleNamespace

import bpy
from . import pivot_rotate
from .key_edit import key_transaction, channelbag
from bpy_extras.anim_utils import action_get_channelbag_for_slot
from mathutils import Quaternion, Vector


AXES = "XYZ"


def ik_chain(rig, active):
    """Resolve an in-rig two-link IK constraint through its goal or pole."""
    if not active:
        return None
    chains = []
    for joint in rig.pose.bones:
        for con in joint.constraints:
            if (con.type != "IK" or con.chain_count != 2 or not joint.parent or con.mute
                    or con.influence <= 0 or con.target != rig or con.pole_target != rig):
                continue
            target = rig.pose.bones.get(con.subtarget)
            pole = rig.pose.bones.get(con.pole_subtarget)
            if not target or not pole:
                continue
            if active not in (pole, target) and active not in target.parent_recursive:
                continue
            # Shared rig roots can feed many IK targets. Only expose an unambiguous match.
            chain = (joint.parent, joint, target, pole)
            if chain not in chains:
                chains.append(chain)
    return chains[0] if len(chains) == 1 else None


def selection(context, *, for_lag=False):
    active = context.active_pose_bone
    selected = list(context.selected_pose_bones or ())
    if not active or active not in selected or any(b.id_data != context.object for b in selected):
        raise ValueError("1体のリグで対象のコントローラーを選択してください")
    chain = ik_chain(context.object, active)
    if chain and len(selected) == 1:
        if any(chain[-1].lock_location):
            raise ValueError("ポールの移動チャンネルがロックされています")
        if not for_lag and "ik_fk_switch" in active and active["ik_fk_switch"] != 0:
            raise ValueError("IKモードのコントローラーを選択してください")
        return chain, active if "ik_fk_switch" in active else None, True
    if len(selected) < 2:
        raise ValueError("IKゴールまたはポールを選択してください")
    selected.sort(key=lambda b: len(b.parent_recursive))
    if any(parent not in child.parent_recursive for parent, child in zip(selected, selected[1:])):
        raise ValueError("1本のFKチェーンを選択してください")
    return selected, None, False


def rotation_path(b):
    return {"QUATERNION": "rotation_quaternion", "AXIS_ANGLE": "rotation_axis_angle"}.get(
        b.rotation_mode, "rotation_euler")


def assign_rotation(b, q, previous=None):
    path = rotation_path(b)
    if path == "rotation_euler":
        current = b.rotation_euler.copy()
        angles = q.to_euler(b.rotation_mode, previous if previous is not None else current)
        for i in range(3):
            if not b.lock_rotation[i]:
                current[i] = angles[i]
        b.rotation_euler = current
        return current
    if any(b.lock_rotation) or (b.lock_rotations_4d and b.lock_rotation_w):
        raise ValueError(f"{b.name} の回転チャンネルにロックがあります")
    q.normalize()
    q.make_compatible(previous if previous is not None else b.matrix_basis.to_quaternion())
    if path == "rotation_quaternion":
        b.rotation_quaternion = q
    else:
        axis, angle = q.to_axis_angle()
        b.rotation_axis_angle = (angle, *axis)
    return q


def key_transform(b, path, frame):
    locked = b.lock_rotation if path == "rotation_euler" else ()
    for i in range(len(getattr(b, path))):
        if locked and locked[i]:
            continue
        if not b.keyframe_insert(data_path=path, index=i, frame=frame, group=b.name):
            raise ValueError(f"{b.name} のキーを書き込めませんでした")


def check_channels(context, writes):
    ad = context.object.animation_data
    if ad and ad.use_tweak_mode:
        raise ValueError("Action Editorで対象Actionを開いてください")
    bag = action_get_channelbag_for_slot(ad.action, ad.action_slot) if ad and ad.action and ad.action_slot else None
    for b, path in writes:
        if path in {"rotation_quaternion", "rotation_axis_angle"} and (
                any(b.lock_rotation) or (b.lock_rotations_4d and b.lock_rotation_w)):
            raise ValueError(f"{b.name} の回転チャンネルにロックがあります")
        full = b.path_from_id(path)
        curves = ([f for f in bag.fcurves if f.data_path == full] if bag else [])
        group = bag.groups.get(b.name) if bag else None
        if ((group and (group.lock or group.mute)) or any(
                f.lock or f.mute or f.modifiers or f.sampled_points or
                (f.group and (f.group.lock or f.group.mute)) for f in curves)) or (
                ad and any(f.data_path == full for f in ad.drivers)):
            raise ValueError(f"{b.name} に直接編集できないカーブがあります")


def projected(v, axis):
    return v - axis * v.dot(axis)


def move_pole(pole, position):
    space = pole.matrix @ pole.matrix_basis.inverted()
    pole.location += space.to_3x3().inverted() @ (position - pole.head)



def options(c):
    return c.window_manager.chain_motion_options


# Common anatomical names, using the selected bone's own prefix and side suffix.
# This keeps control bones, deformation bones and reference skeletons separate.
TORSO = re.compile(r"^(.*?)(pelvis|hips|hip|root|spine|chest|thorax)(?:[._ -]?(\d+))?([._:-][a-z]+)?$", re.I)


def usable(b):
    return not (b.bone.hide or b.bone.hide_select or all(b.lock_rotation)) and (
        not b.bone.collections or any(group.is_visible_effectively for group in b.bone.collections))


def torso_chain(rig, active):
    match = TORSO.fullmatch(active.name)
    if not match:
        return []
    prefix, _, _, suffix = match.groups()
    bases, spines, tops = [], [], []
    for b in rig.pose.bones:
        part = TORSO.fullmatch(b.name)
        if not (part and usable(b) and part.group(1) == prefix and part.group(4) == suffix):
            continue
        role = part.group(2).lower()
        if role == "spine":
            spines.append((int(part.group(3) or 0), b))
        elif role in {"chest", "thorax"}:
            tops.append(b)
        else:
            bases.append((role == "root", b))
    spines.sort(key=lambda pair: pair[0])
    if not spines or len(tops) > 1 or len({i for i, _ in spines}) != len(spines):
        return []
    # Prefer the pelvis to the whole-rig root when both exist.
    bases.sort(key=lambda pair: pair[0])
    chain = [b for _, b in bases[:1]] + [b for _, b in spines] + tops
    return chain if active in chain and len(chain) >= 2 else []


def selected_edit(c):
    """Only the single rotation handle selected by the direction tool owns the session."""
    if _edit and c.object == _edit.rig and c.mode == "POSE":
        active = c.active_pose_bone
        if active and active.name == _edit.names[0] and list(c.selected_pose_bones or ()) == [active]:
            return _edit
    return None


def resolve_chain(c):
    active = c.active_pose_bone
    if c.mode != "POSE" or not active:
        raise ValueError("背骨を1本、または操作するチェーンの骨を複数選択してください")
    picked = list(c.selected_pose_bones or ())
    if not picked:
        raise ValueError("対象のボーンを選択してください")
    if any(b.id_data != c.object for b in picked):
        raise ValueError("同じリグのボーンを選択してください")
    torso = torso_chain(c.object, active)
    if len(picked) < 2:
        edit = selected_edit(c)
        if edit:
            return [c.object.pose.bones[n] for n in edit.base_names]
        if torso:
            return torso
        raise ValueError("このチェーンは使用するボーンを複数選択してください")
    if torso and all(b in torso for b in picked):
        return [b for b in torso if b in picked]
    selected_names = {b.name for b in picked}
    for preset in c.object.chain_motion_presets:
        if len(preset.members) == len(picked) and {m.name for m in preset.members} == selected_names:
            return preset_bones(c.object, preset.name)
    picked.sort(key=lambda b: len(b.parent_recursive))
    if any(parent not in child.parent_recursive for parent, child in zip(picked, picked[1:])):
        raise ValueError("親から子へつながる1本のチェーンを選択してください")
    return picked


class ChainMotionMember(bpy.types.PropertyGroup):
    """Bone name, in root-to-tip order."""


class ChainMotionPreset(bpy.types.PropertyGroup):
    members: bpy.props.CollectionProperty(type=ChainMotionMember)


def preset_bones(rig, name):
    preset = rig.chain_motion_presets.get(name)
    if preset is None:
        raise ValueError(f"保存チェーン「{name}」が見つかりません")
    bones = []
    for member in preset.members:
        bone = rig.pose.bones.get(member.name)
        if bone is None:
            raise ValueError(f"「{name}」のボーン {member.name} がリグにありません")
        if not usable(bone):
            raise ValueError(f"{member.name} を表示し、回転可能にしてください")
        bones.append(bone)
    if len(bones) < 2:
        raise ValueError("チェーンのボーンが2本未満です")
    return bones


class RigPresetOperator:
    @classmethod
    def poll(cls, c):
        return c.mode == "POSE" and c.object and c.object.type == "ARMATURE"


class POSE_OT_chain_save(RigPresetOperator, bpy.types.Operator):
    bl_idname = "pose.chain_motion_save"
    bl_label = "チェーンに名前を付けて保存"
    bl_description = "現在のチェーンの骨と順序を、このリグに保存します"
    bl_options = {"UNDO"}
    preset_name: bpy.props.StringProperty(name="チェーン名")

    def invoke(self, c, event):
        try:
            bones = resolve_chain(c)
        except (ValueError, KeyError) as error:
            self.report({"WARNING"}, str(error))
            return {"CANCELLED"}
        self.preset_name = f"{bones[0].name} – {bones[-1].name}"
        return c.window_manager.invoke_props_dialog(self)

    def draw(self, c):
        self.layout.prop(self, "preset_name")
        try:
            for b in resolve_chain(c):
                self.layout.label(text=b.name, icon="BONE_DATA")
        except (ValueError, KeyError):
            pass

    def execute(self, c):
        name = self.preset_name.strip()
        presets = c.object.chain_motion_presets
        try:
            if not name:
                raise ValueError("チェーン名を入力してください")
            if name in presets:
                raise ValueError(f"「{name}」はこのリグに登録済みです")
            bones = resolve_chain(c)
        except (ValueError, KeyError) as error:
            self.report({"WARNING"}, str(error))
            return {"CANCELLED"}
        preset = presets.add()
        preset.name = name
        for b in bones:
            preset.members.add().name = b.name
        return {"FINISHED"}


class POSE_OT_chain_recall(RigPresetOperator, bpy.types.Operator):
    bl_idname = "pose.chain_motion_recall"
    bl_label = "保存チェーンを選択"
    bl_description = "保存した骨を選択します。回転分散中は現在の向きを引き継ぎます"
    bl_options = {"UNDO"}
    preset_name: bpy.props.StringProperty()

    def execute(self, c):
        try:
            bones = preset_bones(c.object, self.preset_name)
        except ValueError as error:
            self.report({"WARNING"}, str(error))
            return {"CANCELLED"}
        if selected_edit(c):
            return bpy.ops.pose.chain_motion_direction(direction=_edit.direction, preset_name=self.preset_name)
        for b in c.selected_pose_bones or ():
            b.select = False
        for b in bones:
            b.select = True
        c.object.data.bones.active = bones[0].bone
        return {"FINISHED"}


class POSE_OT_chain_forget(RigPresetOperator, bpy.types.Operator):
    bl_idname = "pose.chain_motion_forget"
    bl_label = "登録を削除"
    bl_description = "このリグからチェーン名と骨の一覧を削除します"
    bl_options = {"UNDO"}
    preset_name: bpy.props.StringProperty()

    def invoke(self, c, event):
        return c.window_manager.invoke_confirm(
            self, event, title="保存チェーンを削除", message=f"「{self.preset_name}」の登録を削除しますか？",
            confirm_text="削除",
        )

    def execute(self, c):
        presets = c.object.chain_motion_presets
        index = presets.find(self.preset_name)
        if index < 0:
            return {"CANCELLED"}
        presets.remove(index)
        return {"FINISHED"}


class VIEW3D_MT_chain_presets(bpy.types.Menu):
    bl_label = "保存チェーン"

    def draw(self, c):
        for preset in c.object.chain_motion_presets:
            self.layout.operator("pose.chain_motion_recall", text=preset.name).preset_name = preset.name
        self.layout.separator()
        self.layout.menu("VIEW3D_MT_chain_delete", text="登録を削除…", icon="TRASH")


class VIEW3D_MT_chain_delete(bpy.types.Menu):
    bl_label = "削除するチェーン"

    def draw(self, c):
        self.layout.operator_context = "INVOKE_DEFAULT"
        for preset in c.object.chain_motion_presets:
            self.layout.operator("pose.chain_motion_forget", text=preset.name).preset_name = preset.name


_edit = None


def read_channels(bones):
    return [tuple(getattr(b, rotation_path(b))) for b in bones]


def write_channels(bones, values):
    for b, v in zip(bones, values):
        setattr(b, rotation_path(b), v)


def evaluated_fk_channels(c, bones):
    goal = lag_goal(c.object, bones, False)
    if not goal or not goal.name.startswith("c_hand_ik") or goal["ik_fk_switch"] == 1:
        return read_channels(bones)
    if goal["ik_fk_switch"] != 0:
        raise ValueError("腕のIK/FKがブレンド中のため開始・到達ポーズを取得できません")
    from .arp_pose_assist import rig_functions
    arp = rig_functions(c)
    rig = c.object
    side = arp.get_bone_side(goal.name)
    fk = [rig.pose.bones[base + side] for base in arp.fk_arm[:3]]
    # Sample the visible IK pose in FK coordinates without keying or changing selection.
    rotations = read_channels(fk)
    scale, stretch = tuple(fk[-1].scale), fk[-1]["stretch_length"]
    old_active = rig.data.bones.active
    selected = [(b, b.select) for b in (fk[-1], goal)]
    settings = c.scene.tool_settings
    auto, mirror = settings.use_keyframe_insert_auto, rig.pose.use_mirror_x
    try:
        settings.use_keyframe_insert_auto = False
        rig.pose.use_mirror_x = False
        arp.fk_to_ik_arm(rig, side, add_keyframe=False)
        return read_channels(bones)
    finally:
        goal["ik_fk_switch"] = 0.0
        write_channels(fk, rotations)
        fk[-1].scale, fk[-1]["stretch_length"] = scale, stretch
        rig.pose.use_mirror_x = mirror
        settings.use_keyframe_insert_auto = auto
        for bone, value in selected:
            bone.select = value
        rig.data.bones.active = old_active
        rig.update_tag()
        c.view_layer.update()


def evaluated_start(c, bones, start):
    # Frame evaluation can replace unkeyed transforms. Return the rig to its
    # actual working pose, including unselected controllers, before interaction.
    end = c.scene.frame_current
    editing = _edit
    if editing:
        editing.sampling = True
    all_bones = list(c.object.pose.bones)
    working = [(tuple(b.location), tuple(getattr(b, rotation_path(b))), tuple(b.scale))
               for b in all_bones]
    try:
        c.scene.frame_set(start)
        return evaluated_fk_channels(c, bones)
    finally:
        c.scene.frame_set(end)
        for b, (loc, rot, scale) in zip(all_bones, working):
            path = rotation_path(b)
            if tuple(b.location) != loc:
                b.location = loc
            if tuple(getattr(b, path)) != rot:
                setattr(b, path, rot)
            if tuple(b.scale) != scale:
                b.scale = scale
        c.view_layer.update()
        if editing:
            editing.sampling = False


def active_action(rig):
    return rig.animation_data.action if rig.animation_data else None


class ChainPose:
    def __init__(self, c, bones):
        self.sampling = False
        self.area = c.area
        self.base_names = [b.name for b in bones]
        self.direction = "FIRST"
        self.needs_rebase = False
        self.rig = c.object
        self.scene = c.scene
        self.frame = c.scene.frame_current
        self.action = active_action(self.rig)
        self.names = [b.name for b in bones]
        self.start = options(c).start
        self.source = None
        self.raw = read_channels(bones)
        self.timeline_raw = self.raw
        self.local = [b.matrix_basis.to_quaternion() for b in bones]
        self.pose = [b.matrix.to_quaternion() for b in bones]
        self.last = tuple(self.raw[0])
        self.keyed_leader = self.last
        self.layer = c.view_layer
        self.modes = [b.rotation_mode for b in bones]
        ad = self.rig.animation_data
        self.slot = ad.action_slot if ad else None
        bag = action_get_channelbag_for_slot(ad.action, ad.action_slot) if ad and ad.action and ad.action_slot else None
        self.original_channels = {(fc.data_path, fc.array_index) for fc in bag.fcurves} if bag else set()

    def sample_new_start(self, c, bones, frame):
        # First keys on any chain member extrapolate back to the start.
        ad = self.rig.animation_data
        bag = action_get_channelbag_for_slot(ad.action, ad.action_slot) if ad and ad.action and ad.action_slot else None
        paths = {b.path_from_id(rotation_path(b)) for b in bones}
        new = [fc for fc in bag.fcurves if fc.data_path in paths and not fc.mute and
               (fc.data_path, fc.array_index) not in self.original_channels] if bag else []
        final = read_channels(bones)
        try:
            for fc in new:
                fc.mute = True
            write_channels(bones, self.timeline_raw)
            self.source = evaluated_start(c, bones, frame)
            self.start = frame
        finally:
            for fc in new:
                fc.mute = False
            write_channels(bones, final)
            c.view_layer.update()

    def bones(self):
        return [self.rig.pose.bones[n] for n in self.names]

    def follow(self, *, force=False, update=True):
        bones = self.bones()
        lead = bones[0]
        current = tuple(getattr(lead, rotation_path(lead)))
        if current == self.last and not force:
            return
        self.last = current
        local = lead.matrix_basis.to_quaternion()
        swing = self.pose[0] @ self.local[0].rotation_difference(local) @ self.pose[0].inverted()
        for i, b in enumerate(bones[1:], 1):
            if current == self.raw[0]:
                setattr(b, rotation_path(b), self.raw[i])
                continue
            weight = (len(bones) - i) / len(bones)
            delta = Quaternion().slerp(swing, weight)
            q = self.local[i] @ self.pose[i].inverted() @ delta @ self.pose[i]
            assign_rotation(b, q)
        if update:
            self.layer.update()

    def sync_keyed_followers(self, c):
        """Extend a native rotation key on the selected handle to the displayed chain."""
        bones = self.bones()
        lead = bones[0]
        path = rotation_path(lead)
        current = tuple(getattr(lead, path))
        if current == self.keyed_leader:
            return
        ad = self.rig.animation_data
        if (not ad or not ad.action or not ad.action_slot or ad.use_tweak_mode
                or (self.action and (ad.action != self.action or ad.action_slot != self.slot))):
            return
        bag = channelbag(self.rig)
        if not bag:
            return
        # Existing keys (common at 0F) only count if they contain the NEW rotation.
        # Location-only keys and an unkeyed drag leave the followers in preview.
        for i, value in enumerate(current):
            if value == self.keyed_leader[i]:
                continue
            fc = bag.fcurves.find(lead.path_from_id(path), index=i)
            if not fc or not any(abs(k.co.x - self.frame) < 1e-4 and
                                 abs(k.co.y - value) < 1e-6 for k in fc.keyframe_points):
                return
        writes = [(b, rotation_path(b)) for b in bones[1:]]
        check_channels(c, writes)
        # Native key insertion can re-evaluate the old follower curves, or can
        # precede the preview timer. Reconstruct the pose from the session reference.
        self.follow(force=True, update=False)
        with key_transaction(self.rig, {b.path_from_id(prop) for b, prop in writes}):
            for b, prop in writes:
                key_transform(b, prop, self.frame)
        self.keyed_leader = current
        self.action, self.slot = ad.action, ad.action_slot


def stop_edit():
    global _edit
    if _edit and _edit.area:
        try:
            _edit.area.header_text_set(None)
            _edit.area.tag_redraw()
        except ReferenceError:
            pass
    _edit = None
    if bpy.app.timers.is_registered(preview_chain):
        bpy.app.timers.unregister(preview_chain)


def preview_chain():
    global _edit
    c = bpy.context
    try:
        e = _edit
        if e and e.sampling:
            return 1 / 30
        if e and pivot_rotate.is_active(e.scene, e.rig):
            e.needs_rebase = True
            return 1 / 30
        if not e or c.scene != e.scene or not selected_edit(c):
            if (e and c.scene == e.scene and c.object == e.rig and c.mode == "POSE"
                    and c.scene.frame_current == e.frame and not c.scene.frame_subframe):
                e.sync_keyed_followers(c)
            if e and e.area:
                e.area.header_text_set(None)
                e.area.tag_redraw()
            _edit = None
            return None
        if c.screen and c.screen.is_animation_playing or c.scene.frame_subframe:
            e.needs_rebase = True
            return 1 / 30
        ad = e.rig.animation_data
        if (e.needs_rebase or c.scene.frame_current != e.frame or
                (e.action and (active_action(e.rig) != e.action or ad.action_slot != e.slot))):
            rebase(c, e)
        else:
            e.follow()
            if not any(op.bl_idname.startswith("TRANSFORM_OT_") or op.bl_idname == "GIZMOGROUP_OT_gizmo_tweak"
                       for op in c.window.modal_operators):
                e.sync_keyed_followers(c)
    except (KeyError, ReferenceError, RuntimeError, ValueError):
        _edit = None
        return None
    return 1 / 30


def rebase(c, e):
    """Capture a fresh rotation reference at the frame the animator is posing."""
    area, direction, names = e.area, e.direction, e.base_names
    e.__init__(c, e.bones())
    e.area, e.direction, e.base_names = area, direction, names


class POSE_OT_chain_direction(bpy.types.Operator):
    bl_idname = "pose.chain_motion_direction"
    bl_label = "回転分散の向き"
    bl_options = {"UNDO"}
    preset_name: bpy.props.StringProperty(options={"HIDDEN", "SKIP_SAVE"})
    direction: bpy.props.EnumProperty(items=[
        ("OFF", "通常", "通常のボーン回転に戻る"),
        ("FIRST", "先頭から", "根元のコントローラーを選び、手動回転を先へ伝える"),
        ("LAST", "末尾から", "先端のコントローラーを選び、手動回転を根元へ伝える")])

    def execute(self, c):
        global _edit
        try:
            if _edit and not selected_edit(c):
                stop_edit()
            if _edit:
                _edit.follow()
                if _edit.frame == c.scene.frame_current:
                    _edit.sync_keyed_followers(c)
            if self.direction == "OFF":
                if _edit and c.scene.tool_settings.use_keyframe_insert_auto and read_channels(_edit.bones()) != _edit.raw:
                    for b in _edit.bones():
                        key_transform(b, rotation_path(b), _edit.frame)
                stop_edit()
                return {"FINISHED"}
            if c.mode != "POSE" or c.scene.frame_subframe or c.scene.frame_current < 0:
                raise ValueError("ポーズモードで0F以降の整数フレームを選択してください")
            base = preset_bones(c.object, self.preset_name) if self.preset_name else resolve_chain(c)
            bones = base if self.direction == "FIRST" else list(reversed(base))
            if any(not usable(b) for b in bones):
                raise ValueError("表示されている回転可能なコントローラーを選択してください")
            check_channels(c, [(b, rotation_path(b)) for b in bones])
            state = ChainPose(c, bones)
            if _edit and _edit.rig == c.object and _edit.frame == state.frame and set(_edit.names) == set(state.names):
                source_order = [_edit.names.index(n) for n in state.names]
                state.timeline_raw = [_edit.timeline_raw[i] for i in source_order]
                state.original_channels = _edit.original_channels
                state.start = _edit.start
                state.source = [_edit.source[i] for i in source_order] if _edit.source else None
            elif state.start < state.frame:
                state.source = evaluated_start(c, bones, state.start)
            state.base_names = [b.name for b in base]
            state.direction = self.direction
            stop_edit()
            for b in c.selected_pose_bones or ():
                b.select = False
            bones[0].select = True
            c.object.data.bones.active = bones[0].bone
            _edit = state
            if state.area:
                state.area.header_text_set(f"回転分散  |  {bones[0].name} を R / 回転ギズモで回す")
            bpy.app.timers.register(preview_chain, first_interval=0.0)
        except (ValueError, RuntimeError) as err:
            self.report({"WARNING"}, str(err))
            return {"CANCELLED"}
        return {"FINISHED"}


def lag_goal(rig, bones, ik):
    if ik:
        target = bones[2]
        return next((b for b in (target, *target.parent_recursive) if "ik_fk_switch" in b), None)
    goals = set()
    for bone in bones:
        match = re.fullmatch(r"c_(arm|forearm|hand|thigh|leg|foot|toes)_fk([._].+)", bone.name)
        if match:
            base = "c_hand_ik" if match[1] in {"arm", "forearm", "hand"} else "c_foot_ik"
            goal = rig.pose.bones.get(base + match[2])
            if goal and "ik_fk_switch" in goal:
                goals.add(goal)
    if len(goals) > 1:
        raise ValueError("複数の腕・脚が含まれています。1本のリムを指定してください")
    return next(iter(goals), None)


class LagMode:
    """Make one limb's switch constant over the inclusive bake interval."""

    def __init__(self, rig, bones, ik, start, end):
        from .arp_pose_assist import switch_curve
        self.goal = lag_goal(rig, bones, ik)
        self.value = 0 if ik else 1
        self.start, self.end = start, end
        self.curve = None
        if self.goal is None:
            return
        ad = rig.animation_data
        self.path = self.goal.path_from_id() + '["ik_fk_switch"]'
        self.curve = fc = switch_curve(rig, self.goal)
        if ((ad and (ad.use_tweak_mode or any(d.data_path == self.path for d in ad.drivers)))
                or (fc and (fc.lock or fc.mute or fc.modifiers or fc.sampled_points
                            or (fc.group and (fc.group.lock or fc.group.mute))))):
            raise ValueError(f"{self.goal.name} のIK/FKカーブを直接編集できません")
        self.original = self.goal["ik_fk_switch"]
        self.before = fc.evaluate(start - 1) if fc else self.original
        self.after = fc.evaluate(end + 1) if fc else self.original

    def pose_for_sample(self, context):
        if self.goal and self.goal["ik_fk_switch"] != self.value:
            self.goal["ik_fk_switch"] = self.value
            self.goal.id_data.update_tag()
            context.view_layer.update()

    def apply(self):
        if not self.goal:
            return
        from .arp_pose_assist import switch_curve
        fc = self.curve
        if fc is None:
            if self.original == self.value:
                return
            if not self.goal.keyframe_insert(data_path='["ik_fk_switch"]', frame=self.start):
                raise ValueError(f"{self.goal.name} のIK/FKキーを書き込めません")
            fc = switch_curve(self.goal.id_data, self.goal)
        points = fc.keyframe_points
        for i in reversed(range(len(points))):
            if self.start < points[i].co.x < self.end:
                points.remove(points[i], fast=True)
        fc.update()
        if (len(points) and self.before == self.after == self.value
                and all(k.co.y == self.value and
                        (k.interpolation == "CONSTANT" or
                         (k.handle_left.y == self.value and k.handle_right.y == self.value))
                        for k in points)):
            return

        def set_key(frame, value, hold):
            key = next((p for p in points if p.co.x == frame), None)
            if key is None:
                key = points.insert(frame, value)
                key.interpolation = "CONSTANT"
            elif key.co.y != value:
                key.co.y = value
            if hold:
                key.interpolation = "CONSTANT"
        # Retain the neighboring modes on integer frames. A fractional key
        # closer to the boundary already anchors that side of the interval.
        if self.start > 0 and self.before != self.value and not any(
                self.start - 1 < p.co.x < self.start for p in points):
            set_key(self.start - 1, self.before, True)
        if self.after != self.value and not any(self.end < p.co.x < self.end + 1 for p in points):
            set_key(self.end + 1, self.after, False)
        set_key(self.start, self.value, True)
        set_key(self.end, self.value, False)
        fc.update()


def write_transition(c, state, bones, lag):
    start, end = options(c).start, state.frame
    travel = end - start - lag * (len(bones) - 1)
    if travel < 1:
        raise ValueError("開始Fから到達Fまでに全節が動けるよう、開始Fまたは節間ラグを設定してください")
    if c.object.animation_data and c.object.animation_data.use_tweak_mode:
        raise ValueError("Action Editorで対象Actionを開いてください")
    mode = LagMode(c.object, bones, False, start, end)
    if start != state.start or state.source is None:
        state.sample_new_start(c, bones, start)
    dest = evaluated_fk_channels(c, bones)
    attrs = [(b, rotation_path(b)) for b in bones]
    check_channels(c, attrs)
    paths = {b.path_from_id(path) for b, path in attrs}
    if mode.goal:
        paths.add(mode.path)
    current = read_channels(bones)
    try:
        with key_transaction(c.object, paths):
            write_lag_curves(c, state, attrs, dest, start, end, lag, travel)
            mode.apply()
            mode.pose_for_sample(c)
            write_channels(bones, dest)
            c.view_layer.update()
    except Exception:
        if mode.goal:
            mode.goal["ik_fk_switch"] = mode.original
        write_channels(bones, current)
        c.view_layer.update()
        raise


def exterior_handles(curve, start, end):
    """Preserve the outside halves of Bezier segments when Blender recalculates handles."""
    points = list(curve.keyframe_points)
    handles = {}
    for p in points:
        if p.co.x <= start or p.co.x > end:
            handles[p.co.x, "left"] = p.handle_left.copy()
        if p.co.x < start or p.co.x >= end:
            handles[p.co.x, "right"] = p.handle_right.copy()
    outgoing = None
    for boundary in (start, end):
        if any(p.co.x == boundary for p in points):
            continue
        a = next((p for p in reversed(points) if p.co.x < boundary), None)
        b = next((p for p in points if p.co.x > boundary), None)
        if not a or not b:
            continue
        if boundary == end:
            outgoing = a.interpolation
        if a.interpolation in {"LINEAR", "CONSTANT"}:
            continue
        if a.interpolation != "BEZIER":
            raise ValueError("この補間の区間を分割するには、開始Fと到達Fにキーを打ってください")
        p0, p1, p2, p3 = a.co.copy(), a.handle_right.copy(), b.handle_left.copy(), b.co.copy()
        width = abs(p1.x - p0.x) + abs(p3.x - p2.x)
        if width > p3.x - p0.x:
            scale = (p3.x - p0.x) / width
            p1, p2 = p0.lerp(p1, scale), p3.lerp(p2, scale)
        lo, hi = 0.0, 1.0
        for _ in range(26):
            t = (lo + hi) / 2
            ab, bc, cd = p0.lerp(p1,t), p1.lerp(p2,t), p2.lerp(p3,t)
            l, r = ab.lerp(bc,t), bc.lerp(cd,t)
            if l.lerp(r,t).x < boundary:
                lo = t
            else:
                hi = t
        if boundary == start:
            handles[a.co.x, "right"], handles[start, "left"] = ab, l
        else:
            handles[end, "right"], handles[b.co.x, "left"] = r, cd
    return handles, outgoing


def restore_exterior(curve, handles, outgoing, end):
    for p in curve.keyframe_points:
        if p.co.x == end and outgoing:
            p.interpolation = outgoing
        for side in ("left", "right"):
            saved = handles.get((p.co.x, side))
            prop = "handle_" + side
            if saved is not None and (getattr(p, prop) - saved).length > 1e-7:
                setattr(p, prop + "_type", "FREE")
                setattr(p, prop, saved)
    curve.update()


def write_lag_curves(c, state, attrs, dest, start, end, lag, travel):
    for j, (b, path) in enumerate(attrs):
        a, z = list(state.source[j]), list(dest[j])
        end_values = z
        seam = None
        if path == "rotation_quaternion" and Quaternion(a).dot(Quaternion(z)) < 0:
            z = [-v for v in z]
            bag = channelbag(c.object)
            if bag and any(fc.data_path == b.path_from_id(path) and
                           any(p.co.x > end for p in fc.keyframe_points) for fc in bag.fcurves):
                # A constant hold between q and -q preserves the outgoing authored sign
                # without interpolating through zero. Keep the hold inside the range.
                seam = end - 0.025
            else:
                end_values = dest[j] = z
        begin = start + j * lag
        finish = begin + travel
        bag = channelbag(c.object)
        old = {fc.array_index: exterior_handles(fc, start, end) for fc in bag.fcurves
               if fc.data_path == b.path_from_id(path)} if bag else {}
        # First ensure this bone has channels in the active Action/slot.
        key_transform(b, path, end)
        ad = c.object.animation_data
        bag = action_get_channelbag_for_slot(ad.action, ad.action_slot)
        for k in range(len(z)):
            if path == "rotation_euler" and b.lock_rotation[k]:
                continue
            curve = bag.fcurves.find(b.path_from_id(path), index=k)
            # Reshape previously keyed points within the transition as well.
            times = {float(start), float(begin), float(finish), float(end)}
            times.update(p.co.x for p in curve.keyframe_points if start < p.co.x < end)
            if seam is not None:
                times.add(seam)
            for f in sorted(times):
                t = max(0., min(1., (f - begin) / travel))
                value = end_values[k] if f == end else a[k] + (z[k] - a[k]) * t
                if seam is not None and seam <= f < end:
                    value = z[k]
                p = curve.keyframe_points.insert(f, value, options={"FAST"})
                if f < end:
                    p.interpolation = "CONSTANT" if seam is not None and f >= seam else "LINEAR"
            curve.update()
            if k in old:
                restore_exterior(curve, *old[k], end)


class POSE_OT_chain_pose_finish(bpy.types.Operator):
    bl_idname = "pose.chain_motion_pose_finish"
    bl_label = "チェーンポーズ確定"
    bl_options = {"UNDO"}

    def execute(self, c):
        e = selected_edit(c)
        try:
            if c.mode != "POSE" or c.scene.frame_subframe or c.scene.frame_current < 0:
                raise ValueError("ポーズモードで0F以降の整数フレームを選択してください")
            if e:
                if c.scene.frame_current != e.frame:
                    rebase(c, e)
                e.follow()
            else:
                e = ChainPose(c, resolve_chain(c))
            write_transition(c, e, e.bones(), options(c).lag)
            if e is _edit:
                rebase(c, e)
        except (ValueError, RuntimeError) as err:
            self.report({"WARNING"}, str(err))
            return {"CANCELLED"}
        return {"FINISHED"}


@bpy.app.handlers.persistent
def discard_edit(*args):
    stop_edit()


@bpy.app.handlers.persistent
def leave_pose_frame(scene, *args):
    if _edit and pivot_rotate.is_active(scene, _edit.rig):
        return
    e = selected_edit(bpy.context)
    if e and not e.needs_rebase and not e.sampling and scene == e.scene and (scene.frame_current != e.frame or scene.frame_subframe):
        # Unkeyed preview channels would otherwise carry into the next frame.
        try:
            e.sync_keyed_followers(bpy.context)
            write_channels(e.bones(), e.raw)
        finally:
            e.needs_rebase = True


@bpy.app.handlers.persistent
def enter_pose_frame(scene, *args):
    e = _edit
    if e and pivot_rotate.is_active(scene, e.rig):
        return
    c = bpy.context
    if (e and e.needs_rebase and not e.sampling and e.scene == scene and c.object == e.rig
            and not scene.frame_subframe and not (c.screen and c.screen.is_animation_playing)):
        rebase(c, e)


class ChainOptions(bpy.types.PropertyGroup):
    start: bpy.props.IntProperty(name="開始フレーム", default=0, min=0)

    angle: bpy.props.FloatProperty(name="曲げ角", subtype="ANGLE", default=radians(30))
    axis: bpy.props.EnumProperty(name="FKローカル軸", items=[(s, s, "") for s in AXES], default="Z")
    lag: bpy.props.IntProperty(name="1節のラグ (F)", default=1, min=0, soft_max=12)
    direction: bpy.props.EnumProperty(name="伝播方向", items=[
        ("AUTO", "自動", "FKは根元から、IKは先端から"),
        ("ROOT", "根元から", "肩・腰側を起点にする"),
        ("TIP", "先端から", "手・足側を起点にする")])


class POSE_OT_chain_bend(bpy.types.Operator):
    bl_idname = "pose.chain_motion_bend"
    bl_label = "現在のポーズを曲げる"
    bl_description = "FKは合計角度を均等配分、IKは手足の位置を保って曲げ面を回転。自動キーに対応"
    bl_options = {"REGISTER", "UNDO"}

    @classmethod
    def poll(cls, context):
        return context.mode == "POSE" and context.active_pose_bone is not None

    def execute(self, context):
        opts = context.window_manager.chain_motion_options
        mutated = False
        try:
            chain, _, ik = selection(context)
            auto = context.scene.tool_settings.use_keyframe_insert_auto
            frame = context.scene.frame_current + context.scene.frame_subframe
            if auto and frame < 0:
                raise ValueError("0F以降で自動キーを使用してください")
            if ik:
                root, _, target, pole = chain
                axis = target.head - root.head
                if axis.length < 1e-7 or projected(pole.head-root.head, axis.normalized()).length < 1e-7:
                    raise ValueError("リムの曲げ面を決定できません")
                writes = [(pole, "location")]
                check_channels(context, writes)
                move_pole(pole, root.head + Quaternion(axis, opts.angle) @ (pole.head - root.head))
                mutated = True
            else:
                index = AXES.index(opts.axis)
                usable = [b for b in chain if not b.lock_rotation[index]]
                if not usable:
                    raise ValueError("選択した軸は全節でロックされています")
                writes = [(b, rotation_path(b)) for b in usable]
                check_channels(context, writes)
                step = opts.angle / len(usable)
                axis = Vector(tuple(i == index for i in range(3)))
                for b in usable:
                    if b.rotation_mode not in {"QUATERNION", "AXIS_ANGLE"} and any(b.lock_rotation):
                        b.rotation_euler[index] += step
                    else:
                        assign_rotation(b, b.matrix_basis.to_quaternion() @ Quaternion(axis, step))
                    mutated = True
            if auto:
                for b, path in writes:
                    key_transform(b, path, frame)
            context.view_layer.update()
        except (ValueError, RuntimeError) as error:
            self.report({"WARNING"}, str(error))
            return {"FINISHED"} if mutated else {"CANCELLED"}
        return {"FINISHED"}


def ik_sample(chain):
    root, joint, target, pole = chain
    axis = target.head - root.head
    if axis.length < 1e-7:
        raise ValueError("肩と手、または股関節と足首が重なるフレームがあります")
    axis.normalize()
    # At full extension the pole still defines the chosen side.
    elbow = joint.head.copy()
    if projected(elbow - root.head, axis).length < 1e-5:
        elbow += projected(pole.head - root.head, axis).normalized() * max(joint.length * .01, 1e-5)
    return SimpleNamespace(root=root.head.copy(), tip=target.head.copy(), joint=elbow,
                           axis=axis, pole=pole.head.copy(), location=pole.location.copy(),
                           space=(pole.matrix @ pole.matrix_basis.inverted()).to_3x3().inverted())


def delayed_pole(now, past, from_tip):
    # For a moving hand compare elbows relative to the shoulder. For a planted
    # foot compare knees relative to the ankle, so whole-character travel cancels.
    anchor = "root" if from_tip else "tip"
    wanted = past.joint - getattr(past, anchor) + getattr(now, anchor)
    a = projected(now.joint - now.root, now.axis)
    b = projected(wanted - now.root, now.axis)
    if min(a.length, b.length) < 1e-7:
        return now.location
    twist = atan2(now.axis.dot(a.cross(b)), a.dot(b))
    delta = (Quaternion(now.axis, twist) @ (now.pole - now.root)) - (now.pole - now.root)
    return now.location + now.space @ delta


class POSE_OT_chain_clean_mode(bpy.types.Operator):
    bl_idname = "pose.chain_motion_clean_mode"
    bl_label = "ラグ区間のIK/FKキーを整理"
    bl_description = "既存のラグの変形キーを保ち、選択リムの区間内をFKチェーンまたはIKゴールのモードに統一"
    bl_options = {"UNDO"}

    start: bpy.props.IntProperty(name="開始", min=0)
    end: bpy.props.IntProperty(name="終了", min=0)

    @classmethod
    def poll(cls, c):
        ad = c.object.animation_data if c.mode == "POSE" and c.object else None
        return bool(ad and ad.action and not ad.use_tweak_mode and c.active_pose_bone)

    def invoke(self, c, event):
        self.start = options(c).start
        self.end = max(self.start + 1, c.scene.frame_current)
        return c.window_manager.invoke_props_dialog(self)

    def execute(self, c):
        try:
            if self.end <= self.start:
                raise ValueError("開始より後の終了フレームを指定してください")
            edit = selected_edit(c)
            if edit:
                bones, ik = edit.bones(), False
            else:
                bones, _, ik = selection(c, for_lag=True)
            mode = LagMode(c.object, bones, ik, self.start, self.end)
            if mode.goal is None:
                raise ValueError("IK/FKのある腕・脚のチェーンまたはIKゴールを選択してください")
            mode.apply()
            if self.start <= c.scene.frame_current_final <= self.end:
                mode.pose_for_sample(c)
        except (ValueError, RuntimeError) as error:
            self.report({"WARNING"}, str(error))
            return {"CANCELLED"}
        self.report({"INFO"}, f"{mode.goal.name}  {self.start}–{self.end}F → {'IK' if ik else 'FK'}")
        return {"FINISHED"}


class POSE_OT_chain_lag(bpy.types.Operator):
    bl_idname = "pose.chain_motion_lag"
    bl_label = "範囲にラグをキー化"
    bl_description = "現在のActionを先にサンプルし、指定範囲の各フレームにFK回転またはIKポール位置を記録"
    bl_options = {"UNDO"}

    start: bpy.props.IntProperty(name="開始", min=0)
    end: bpy.props.IntProperty(name="終了", min=0)

    @classmethod
    def poll(cls, context):
        ad = context.object.animation_data if context.mode == "POSE" else None
        return bool(ad and ad.action and not ad.use_tweak_mode and context.active_pose_bone)

    def invoke(self, context, event):
        sc = context.scene
        first, last = context.object.animation_data.action.frame_range
        if sc.use_preview_range:
            first, last = sc.frame_preview_start, sc.frame_preview_end
        self.start, self.end = max(0, ceil(first)), max(0, floor(last))
        return context.window_manager.invoke_props_dialog(self)

    def execute(self, context):
        sc = context.scene
        opts = context.window_manager.chain_motion_options
        frame, subframe = sc.frame_current, sc.frame_subframe
        success = sampled = False
        try:
            if self.start >= self.end or not opts.lag:
                raise ValueError("開始より後の終了フレームと、1F以上のラグを指定してください")
            chain, _, ik = selection(context, for_lag=True)
            mode = LagMode(context.object, chain, ik, self.start, self.end)
            tip = opts.direction == "TIP" or (opts.direction == "AUTO" and ik)
            # For reverse FK, upstream compensation also reaches the leading tip:
            # its evaluated orientation must stay at its current-time value.
            moving = chain[-1:] if ik else (chain if tip else chain[1:])
            writes = [(b, "location" if ik else rotation_path(b)) for b in moving]
            check_channels(context, writes)
            samples = []
            for f in range(self.start, self.end + 1):
                sc.frame_set(f)
                sampled = True
                mode.pose_for_sample(context)
                if ik:
                    samples.append(ik_sample(chain))
                else:
                    samples.append([(b.matrix.to_quaternion(), tuple(getattr(b, rotation_path(b))))
                                    for b in chain])
            paths = {b.path_from_id(path) for b, path in writes}
            if mode.goal:
                paths.add(mode.path)
            with key_transaction(context.object, paths):
                mode.apply()
                previous = {}
                for i, now in enumerate(samples):
                    f = self.start + i
                    sc.frame_set(f)
                    if ik:
                        pole = chain[-1]
                        pole.location = delayed_pole(now, samples[max(0, i - opts.lag)], tip)
                        key_transform(pole, "location", f)
                    else:
                        for j, b in enumerate(chain):
                            if not tip and j == 0:
                                continue
                            path = rotation_path(b)
                            setattr(b, path, now[j][1])
                            context.view_layer.update()
                            delay = opts.lag * (len(chain) - 1 - j if tip else j)
                            wanted = samples[max(0, i - delay)][j][0]
                            current = b.matrix_basis.to_quaternion()
                            local = current @ b.matrix.to_quaternion().inverted() @ wanted
                            previous[j] = assign_rotation(b, local, previous.get(j))
                            key_transform(b, path, f)
                            context.view_layer.update()
            success = True
        except (ValueError, RuntimeError) as error:
            self.report({"WARNING"}, str(error))
            return {"CANCELLED"}
        finally:
            if sampled:
                if mode.goal and not success:
                    mode.goal["ik_fk_switch"] = mode.original
                sc.frame_set(frame, subframe=subframe)
        self.report({"INFO"}, f"{self.start}–{self.end}F に {opts.lag}F のラグを記録しました")
        return {"FINISHED"}


class VIEW3D_PT_chain_motion(bpy.types.Panel):
    bl_space_type = "VIEW_3D"
    bl_region_type = "UI"
    bl_category = "BAA"
    bl_parent_id = "VIEW3D_PT_baa_root"
    bl_order = 20
    bl_label = "チェーンモーション"

    def draw(self, context):
        opts = context.window_manager.chain_motion_options
        if context.mode != "POSE":
            self.layout.label(text="ポーズモードで使用します")
            return
        col = self.layout.column(align=True)
        edit = selected_edit(context)
        library = col.row(align=True)
        if context.object.chain_motion_presets:
            library.menu("VIEW3D_MT_chain_presets", text="保存チェーン", icon="PRESET")
        library.operator("pose.chain_motion_save", text="操作中のチェーンを保存" if edit else "選択を名前付きで保存", icon="ADD")
        mode = edit.direction if edit else "OFF"
        row = col.row(align=True)
        for value, label in (("OFF", "通常"), ("FIRST", "先頭から"), ("LAST", "末尾から")):
            row.operator("pose.chain_motion_direction", text=label, depress=mode == value).direction = value
        ready = None
        if edit:
            col.label(text=f"回す骨   {edit.names[0]}", icon="ORIENTATION_GIMBAL")
            col.label(text="追従   " + " → ".join(edit.names[1:]))
        else:
            picked = context.selected_pose_bones or ()
            if len(picked) >= 2:
                try:
                    ready = resolve_chain(context)
                    col.label(text=f"選択中   {ready[0].name} → {ready[-1].name}（{len(ready)}本）")
                except ValueError as error:
                    col.label(text=str(error))
            else:
                col.label(text="背骨を1本、または対象の骨を複数選択")
            b = context.active_pose_bone
            if b and ik_chain(context.object, b):
                box = self.layout.box()
                box.label(text="IKポール")
                box.prop(opts, "angle")
                box.operator("pose.chain_motion_bend", text="ポールを回転")
                box.prop(opts, "direction")
                box.prop(opts, "lag")
                box.operator("pose.chain_motion_lag", text="IKポールのラグをキー化")
        if edit or ready:
            count = len(edit.names) if edit else len(ready)
            end = context.scene.frame_current
            col.prop(opts, "start")
            col.prop(opts, "lag")
            apply = col.row()
            apply.enabled = opts.start + (count - 1) * opts.lag < end
            names = edit.names if edit else [b.name for b in ready]
            members = set(names)
            label = next((p.name for p in context.object.chain_motion_presets
                          if {m.name for m in p.members} == members), names[0])
            apply.operator("pose.chain_motion_pose_finish", text=f"{label} · {opts.start}–{end}F に遅延キー")
            if not apply.enabled:
                col.label(text="開始Fを早めるか、節間ラグを減らしてください", icon="INFO")


classes = (ChainMotionMember, ChainMotionPreset, ChainOptions, POSE_OT_chain_save,
           POSE_OT_chain_recall, POSE_OT_chain_forget, VIEW3D_MT_chain_delete, VIEW3D_MT_chain_presets, POSE_OT_chain_direction,
           POSE_OT_chain_pose_finish, POSE_OT_chain_bend, POSE_OT_chain_clean_mode, POSE_OT_chain_lag, VIEW3D_PT_chain_motion)


def register():
    for cls in classes:
        bpy.utils.register_class(cls)
    bpy.types.Object.chain_motion_presets = bpy.props.CollectionProperty(type=ChainMotionPreset)
    bpy.types.WindowManager.chain_motion_options = bpy.props.PointerProperty(type=ChainOptions)
    bpy.app.handlers.frame_change_pre.append(leave_pose_frame)
    bpy.app.handlers.frame_change_post.append(enter_pose_frame)


    for hooks in (bpy.app.handlers.undo_pre, bpy.app.handlers.redo_pre, bpy.app.handlers.load_pre):
        hooks.append(discard_edit)


def unregister():
    stop_edit()
    bpy.app.handlers.frame_change_pre.remove(leave_pose_frame)
    bpy.app.handlers.frame_change_post.remove(enter_pose_frame)
    for hooks in (bpy.app.handlers.undo_pre, bpy.app.handlers.redo_pre, bpy.app.handlers.load_pre):
        hooks.remove(discard_edit)
    del bpy.types.WindowManager.chain_motion_options
    del bpy.types.Object.chain_motion_presets
    for cls in reversed(classes):
        bpy.utils.unregister_class(cls)


if __name__ == "__main__":
    register()
