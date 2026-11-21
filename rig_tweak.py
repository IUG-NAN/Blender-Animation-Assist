# reuse-checked: Tools\rig_tweak.py の公開用の写し（社内向けの見出しだけ外す。中身は同じ）
# Rig Tweak — 部位（背骨全部など、選んだ骨の組）に、元のアニメの上から足す位置と回転を1つの輪で付ける
#
# 使い方: ポーズモードで部位の骨をまとめて選び（例: 背骨3本）、右クリック > Rig Tweak > 「Tweakを追加」。
#   根元の骨に重なって紫の輪（tw_<根元の骨>）が1つ出る。輪をギズモ・G・R で動かすと、回転は根元から先へ配分して足され、
#   位置は根元の骨に足される。キーは輪に自分で打つ（Actionごとに別の値）。
#   右クリックは追加と「Tweak ⇔ 部位の骨」（＋Pivot Rotate）だけ。効き・0に戻す・外す・一覧はサイドバーの Tool タブ
#   （リグに Tweak があるときだけ出る）。
# 中身（部位 = 骨 B1..Bn、B1 が根元）:
#   tw_base_<B1>: B1 と同じレスト・親・拘束。B1 のチャンネルをドライバーで写し、拘束の効きも写す。= 調整前の B1
#   tw_<B1>（輪）: tw_base_<B1> の子。動かした量が、調整前の B1 を基準にした差分。部位の骨の名前と効きをプロパティに持つ
#   B1:  Copy Location（ワールド→ワールド、RigTweak_Loc）で輪の位置へ
#   Bk:  Copy Rotation（輪のローカル → tw_base_<B1> を基準にした空間、元の回転の前に掛ける、RigTweak_Rot）、
#        効き = 輪の効き × 配分（k / (1+2+…+n)。根元ほど小さく先ほど大きい。合計1）
#   行列で写すので、オイラーでもクォータニオンでも同じに効く。骨1本だけを選べば、その骨が輪にそのまま重なる。
#   tw_ の骨は変形しない（Auto-Rig Pro の書き出し対象の c_ / cc_ にしない）。書き出しには結果だけが焼かれる。
#   輪にキーが無いActionへ切り替えたら、輪を0に戻す（前のActionの値を持ち越さない）。

bl_info = {
    "name": "Rig Tweak",
    "author": "NAN",
    "version": (2, 0, 0),
    "blender": (5, 2, 0),
    "location": "3Dビュー > ポーズモードの右クリック > Rig Tweak / サイドバー > Tool",
    "description": "部位（選んだ骨の組）に、元のアニメの上から足す位置と回転を1つの輪で付ける。値はActionごと",
    "category": "Animation",
}

import math
import bpy
from bpy.app.handlers import persistent

PRE, PRE_BASE = "tw_", "tw_base_"
CON_LOC, CON_ROT = "RigTweak_Loc", "RigTweak_Rot"
P_BONES, P_INF = "tw_bones", "tw_influence"
COLL, COLL_BASE, SHAPE_COLL = "RigTweak", "RigTweak_Base", "RigTweak_Shapes"
COLOR = ((0.85, 0.25, 0.95), (1.0, 0.6, 1.0), (1.0, 0.9, 1.0))   # 通常・選択・アクティブ（ARPの色と被らない紫）


def is_tweak(name):
    return name.startswith(PRE)


def rings(rig):
    return [pb for pb in rig.pose.bones if P_BONES in pb.keys()]


def members(ring):
    return [n for n in str(ring[P_BONES]).split(",") if n]


def ring_of(rig, name):
    """輪の名前・部位の骨の名前から、その部位の輪を返す"""
    pb = rig.pose.bones.get(name)
    if pb is not None and P_BONES in pb.keys():
        return pb
    for r in rings(rig):
        if name in members(r):
            return r
    return None


def _channels(mode):
    rot = ("rotation_quaternion", 4) if mode == 'QUATERNION' else \
          ("rotation_axis_angle", 4) if mode == 'AXIS_ANGLE' else ("rotation_euler", 3)
    return (("location", 3), rot, ("scale", 3))


def _remove_drivers(rig, prefix):
    if rig.animation_data:
        for fc in [f for f in rig.animation_data.drivers if f.data_path.startswith(prefix)]:
            rig.animation_data.drivers.remove(fc)


def _driver(owner, path, rig, src_path, expr="v", index=-1):
    d = (owner.driver_add(path, index) if index >= 0 else owner.driver_add(path)).driver
    d.type = 'SCRIPTED'
    for v in list(d.variables):
        d.variables.remove(v)
    v = d.variables.new()
    v.name = "v"
    v.type = 'SINGLE_PROP'
    v.targets[0].id_type = 'OBJECT'
    v.targets[0].id = rig
    v.targets[0].data_path = src_path
    d.expression = expr


def _set_sel(pb, v):
    if hasattr(pb, "select"):
        pb.select = v
    else:
        pb.bone.select = v


def _sel(pb):
    return pb.select if hasattr(pb, "select") else pb.bone.select


def _depth(bone):
    d, p = 0, bone.parent
    while p:
        d, p = d + 1, p.parent
    return d


def _edit_mode(context, rig, fn):
    vl = context.view_layer
    prev_active, prev_mode = vl.objects.active, rig.mode
    vl.objects.active = rig
    bpy.ops.object.mode_set(mode='EDIT')
    try:
        fn()
    finally:
        bpy.ops.object.mode_set(mode=prev_mode if prev_mode in ('POSE', 'OBJECT') else 'POSE')
        vl.objects.active = prev_active


def _bone_coll(arm, name, visible):
    c = arm.collections_all.get(name) or arm.collections.new(name)
    c.is_visible = visible
    return c


def _shape(name):
    """輪＋外向きのとげ4本（骨の長さを1とした寸法）"""
    ob_name = "cs_" + name
    me = bpy.data.meshes.get(ob_name) or bpy.data.meshes.new(ob_name)
    n, r, y = 32, 0.6, 0.5
    verts = [(r * math.cos(2 * math.pi * i / n), y, r * math.sin(2 * math.pi * i / n)) for i in range(n)]
    edges = [(i, (i + 1) % n) for i in range(n)]
    for k in range(4):
        a = 2 * math.pi * k / 4
        verts += [(r * math.cos(a), y, r * math.sin(a)), (r * 1.35 * math.cos(a), y, r * 1.35 * math.sin(a))]
        edges.append((len(verts) - 2, len(verts) - 1))
    me.clear_geometry()
    me.from_pydata(verts, edges, [])
    ob = bpy.data.objects.get(ob_name) or bpy.data.objects.new(ob_name, me)
    coll = bpy.data.collections.get(SHAPE_COLL)
    if coll is None:
        coll = bpy.data.collections.new(SHAPE_COLL)
        bpy.context.scene.collection.children.link(coll)
        coll.hide_viewport = True
        coll.hide_render = True
    if coll not in ob.users_collection:
        coll.objects.link(ob)
    return ob


def _mark_seen(rig):
    ad = rig.animation_data
    _last_action[rig.name] = (ad.action.name if ad and ad.action else None, getattr(ad, "action_slot_handle", None) if ad else None)


def add_group(context, rig, names):
    """選んだ骨の組（部位）に輪を1つ付ける。キーには触れない。付いた輪を返す"""
    arm = rig.data
    names = sorted(set(names), key=lambda n: _depth(arm.bones[n]))
    root = names[0]
    ring, base = PRE + root, PRE_BASE + root
    src = rig.pose.bones[root]

    def make():
        ref = arm.edit_bones[root]
        for bn in (base, ring):
            eb = arm.edit_bones.new(bn)
            eb.head, eb.tail, eb.roll = ref.head.copy(), ref.tail.copy(), ref.roll
            eb.use_deform = False
            eb.use_connect = False
            if bn == base:
                eb.parent = ref.parent
                eb.use_inherit_rotation = ref.use_inherit_rotation
                eb.inherit_scale = ref.inherit_scale
                eb.use_local_location = ref.use_local_location
                eb.use_relative_parent = ref.use_relative_parent
            else:
                eb.parent = arm.edit_bones[base]
    _edit_mode(context, rig, make)

    for bn, coll in ((base, _bone_coll(arm, COLL_BASE, False)), (ring, _bone_coll(arm, COLL, True))):
        b = arm.bones[bn]
        for c in list(b.collections):
            c.unassign(b)
        coll.assign(b)
        b.use_deform = False

    # 受け骨: 根元の骨のチャンネルと拘束を写す
    bpb = rig.pose.bones[base]
    bpb.rotation_mode = src.rotation_mode
    for prop, n in _channels(src.rotation_mode):
        for i in range(n):
            _driver(bpb, prop, rig, 'pose.bones["%s"].%s[%d]' % (root, prop, i), index=i)
    for c in src.constraints:
        if c.name in (CON_LOC, CON_ROT):
            continue
        nc = bpb.constraints.copy(c)
        _driver(nc, "influence", rig, 'pose.bones["%s"].constraints["%s"].influence' % (root, c.name))
    arm.bones[base].hide = True

    # 輪
    rpb = rig.pose.bones[ring]
    rpb.rotation_mode = src.rotation_mode
    rpb.lock_scale = (True, True, True)
    rpb[P_BONES] = ",".join(names)
    rpb[P_INF] = 1.0
    rpb.id_properties_ui(P_INF).update(min=0.0, max=1.0, soft_min=0.0, soft_max=1.0, default=1.0, subtype='FACTOR',
                                       description="この部位の調整の効き。キーを打てばActionごとに持てる")
    rpb.custom_shape = _shape(ring)
    rpb.use_custom_shape_bone_size = True
    if hasattr(rpb, "custom_shape_wire_width"):
        rpb.custom_shape_wire_width = 2.0
    b = arm.bones[ring]
    b.show_wire = True
    rpb.color.palette = 'DEFAULT'
    b.color.palette = 'CUSTOM'
    b.color.custom.normal, b.color.custom.select, b.color.custom.active = COLOR

    inf_path = 'pose.bones["%s"]["%s"]' % (ring, P_INF)
    # 位置は根元へ
    c = src.constraints.new('COPY_LOCATION')
    c.name = CON_LOC
    c.target, c.subtarget = rig, ring
    c.target_space = c.owner_space = 'WORLD'
    _driver(c, "influence", rig, inf_path)
    # 回転は根元から先へ配分
    total = sum(range(1, len(names) + 1))
    for k, n in enumerate(names, 1):
        c = rig.pose.bones[n].constraints.new('COPY_ROTATION')
        c.name = CON_ROT
        c.target, c.subtarget = rig, ring
        c.target_space = 'LOCAL'
        c.owner_space = 'CUSTOM'
        c.space_object, c.space_subtarget = rig, base
        c.mix_mode = 'BEFORE'
        _driver(c, "influence", rig, inf_path, "v*%.6f" % (k / total))
    _mark_seen(rig)
    return rpb


def remove_group(context, rig, ring_name):
    """部位の Tweak を外す。Action に残った輪のキーは消さない"""
    rpb = rig.pose.bones.get(ring_name)
    if rpb is None:
        return
    root = ring_name[len(PRE):]
    for n in members(rpb):
        pb = rig.pose.bones.get(n)
        if pb is None:
            continue
        for cn in (CON_LOC, CON_ROT):
            _remove_drivers(rig, 'pose.bones["%s"].constraints["%s"]' % (n, cn))
            if cn in pb.constraints:
                pb.constraints.remove(pb.constraints[cn])
    base = PRE_BASE + root
    for bn in (ring_name, base):
        _remove_drivers(rig, 'pose.bones["%s"]' % bn)
    names = [n for n in (ring_name, base) if n in rig.data.bones]

    def kill():
        for n in names:
            eb = rig.data.edit_bones.get(n)
            if eb:
                rig.data.edit_bones.remove(eb)
    _edit_mode(context, rig, kill)
    ob = bpy.data.objects.get("cs_" + ring_name)
    if ob is not None and not any(pb.custom_shape == ob for obj in bpy.data.objects
                                  if obj.type == 'ARMATURE' and obj.pose for pb in obj.pose.bones):
        me = ob.data
        bpy.data.objects.remove(ob, do_unlink=True)
        if me is not None and me.users == 0:
            bpy.data.meshes.remove(me)
    for cn in (COLL, COLL_BASE):
        c = rig.data.collections_all.get(cn)
        if c is not None and len(c.bones) == 0:
            rig.data.collections.remove(c)


def reset_ring(pb):
    pb.location = (0.0, 0.0, 0.0)
    if pb.rotation_mode == 'QUATERNION':
        pb.rotation_quaternion = (1.0, 0.0, 0.0, 0.0)
    elif pb.rotation_mode == 'AXIS_ANGLE':
        pb.rotation_axis_angle = (0.0, 0.0, 1.0, 0.0)
    else:
        pb.rotation_euler = (0.0, 0.0, 0.0)


# --- Actionを切り替えたときの持ち越し止め ---------------------------------
_last_action = {}


def _keyed_paths(rig):
    ad = rig.animation_data
    if ad is None or ad.action is None:
        return set()
    try:
        from bpy_extras import anim_utils
        cb = anim_utils.action_get_channelbag_for_slot(ad.action, ad.action_slot)
        return {f.data_path for f in cb.fcurves} if cb else set()
    except Exception:
        return set()


def _check_rigs():
    for rig in bpy.data.objects:
        if rig.type != 'ARMATURE' or rig.pose is None:
            continue
        rs = rings(rig)
        if not rs:
            continue
        ad = rig.animation_data
        cur = (ad.action.name if ad and ad.action else None, getattr(ad, "action_slot_handle", None) if ad else None)
        if _last_action.get(rig.name) == cur:
            continue
        _last_action[rig.name] = cur
        keyed = _keyed_paths(rig)
        for pb in rs:
            pre = 'pose.bones["%s"]' % pb.name
            if not any(p.startswith(pre + ".") for p in keyed):
                reset_ring(pb)
            if not any(p == '%s["%s"]' % (pre, P_INF) for p in keyed):
                pb[P_INF] = 1.0


@persistent
def _on_depsgraph(scene, depsgraph):
    global _initialize_seen
    if _initialize_seen:
        # Blender restricts scene access while it registers an add-on.
        # Seed on the first real evaluation so enabling keeps unkeyed Tweak edits.
        for rig in bpy.data.objects:
            if rig.type == 'ARMATURE' and rig.pose and rings(rig):
                _mark_seen(rig)
        _initialize_seen = False
    _check_rigs()


@persistent
def _on_load(dummy):
    global _initialize_seen
    _initialize_seen = False
    _last_action.clear()


# --- 操作 ---------------------------------------------------------------
def _pose_poll(context):
    o = context.object
    return o is not None and o.type == 'ARMATURE' and context.mode == 'POSE'


def _selected_rings(rig):
    out = []
    for pb in rig.pose.bones:
        if _sel(pb):
            r = ring_of(rig, pb.name)
            if r is not None and r not in out:
                out.append(r)
    return out


class RIGTWEAK_OT_add(bpy.types.Operator):
    """選んだ骨の組（部位）に、輪を1つ付ける。キーには触れない"""
    bl_idname = "rig_tweak.add"
    bl_label = "Tweakを追加"
    bl_options = {'REGISTER', 'UNDO'}
    preset_name: bpy.props.StringProperty(options={'SKIP_SAVE'})
    poll = classmethod(lambda cls, context: _pose_poll(context))

    def execute(self, context):
        rig = context.object
        if self.preset_name:
            from .chain_motion import preset_bones
            try:
                names = [bone.name for bone in preset_bones(rig, self.preset_name)]
            except ValueError as error:
                self.report({'WARNING'}, str(error))
                return {'CANCELLED'}
        else:
            names = [pb.name for pb in rig.pose.bones if _sel(pb) and not is_tweak(pb.name)]
        taken = [n for n in names if ring_of(rig, n) is not None]
        if taken:
            self.report({'WARNING'}, "もう Tweak の付いた骨が入っている: %s" % ", ".join(taken))
            return {'CANCELLED'}
        if not names:
            self.report({'WARNING'}, "部位の骨が選ばれていない")
            return {'CANCELLED'}
        rpb = add_group(context, rig, names)
        for pb in rig.pose.bones:
            _set_sel(pb, pb == rpb)
        rig.data.bones.active = rig.data.bones[rpb.name]
        self.report({'INFO'}, "Tweakを付けた: %s（%d 本）" % (rpb.name, len(names)))
        return {'FINISHED'}


class RIGTWEAK_OT_toggle(bpy.types.Operator):
    """選択を、輪と部位の骨の間で行き来する"""
    bl_idname = "rig_tweak.toggle"
    bl_label = "Tweak ⇔ 部位の骨"
    bl_options = {'REGISTER', 'UNDO'}
    poll = classmethod(lambda cls, context: _pose_poll(context))

    def execute(self, context):
        rig = context.object
        pbs = rig.pose.bones
        act = context.active_pose_bone
        on_ring = act is not None and P_BONES in act.keys()
        rs = _selected_rings(rig)
        if not rs:
            return {'CANCELLED'}
        pick = [n for r in rs for n in members(r)] if on_ring else [r.name for r in rs]
        pick = [n for n in pick if n in pbs]
        for pb in pbs:
            _set_sel(pb, pb.name in pick)
        rig.data.bones.active = rig.data.bones[pick[0] if on_ring else pick[-1]]
        return {'FINISHED'}


class RIGTWEAK_OT_reset(bpy.types.Operator):
    """選んだ部位の輪を0に戻す（キーは打たない）"""
    bl_idname = "rig_tweak.reset"
    bl_label = "Tweakを0に戻す"
    bl_options = {'REGISTER', 'UNDO'}
    poll = classmethod(lambda cls, context: _pose_poll(context))

    def execute(self, context):
        for r in _selected_rings(context.object):
            reset_ring(r)
        return {'FINISHED'}


class RIGTWEAK_OT_remove(bpy.types.Operator):
    """選んだ部位の Tweak を外す。Actionに残った輪のキーは消さない"""
    bl_idname = "rig_tweak.remove"
    bl_label = "Tweakを外す"
    bl_options = {'REGISTER', 'UNDO'}
    poll = classmethod(lambda cls, context: _pose_poll(context))

    def execute(self, context):
        rig = context.object
        names = [r.name for r in _selected_rings(rig)]
        for n in names:
            remove_group(context, rig, n)
        self.report({'INFO'}, "Tweakを外した: %s" % (", ".join(names) or "なし"))
        return {'FINISHED'}


class RIGTWEAK_OT_select(bpy.types.Operator):
    """この部位の輪だけを選ぶ"""
    bl_idname = "rig_tweak.select"
    bl_label = "Tweakを選ぶ"
    bl_options = {'REGISTER', 'UNDO'}
    bone: bpy.props.StringProperty()
    poll = classmethod(lambda cls, context: _pose_poll(context))

    def execute(self, context):
        rig = context.object
        if self.bone not in rig.pose.bones:
            return {'CANCELLED'}
        for pb in rig.pose.bones:
            _set_sel(pb, pb.name == self.bone)
        rig.data.bones.active = rig.data.bones[self.bone]
        return {'FINISHED'}


def _label(r):
    m = members(r)
    return m[0] if len(m) == 1 else "%s 〜 %s（%d本）" % (m[0], m[-1], len(m))


class RIGTWEAK_PT_panel(bpy.types.Panel):
    bl_label = "既存ポーズの調整"
    bl_space_type = 'VIEW_3D'
    bl_region_type = 'UI'
    bl_category = "BAA"
    bl_parent_id = "VIEW3D_PT_baa_root"
    bl_order = 30
    bl_options = {'DEFAULT_CLOSED'}

    @classmethod
    def poll(cls, context):
        return _pose_poll(context)

    def draw(self, context):
        rig = context.object
        L = self.layout
        row = L.row(align=True)
        row.operator("pivot_rotate.enable", text="選択中をPivotにする", icon='PIVOT_ACTIVE')
        if context.scene.baa_pivot.rig:
            row.operator("pivot_rotate.disable", text="Pivot解除")
        L.operator("rig_tweak.add", text="選択骨にTweakを追加", icon='ADD')
        if rig.chain_motion_presets:
            L.menu("RIGTWEAK_MT_saved_chains", icon='PRESET')
        if rings(rig):
            L.operator("rig_tweak.toggle")
        act = context.active_pose_bone
        cur = ring_of(rig, act.name) if act else None
        for r in rings(rig):
            box = L.box()
            row = box.row(align=True)
            row.operator("rig_tweak.select", text=_label(r), icon='RESTRICT_SELECT_OFF', depress=(r == cur)).bone = r.name
            box.prop(r, '["%s"]' % P_INF, text="効き", slider=True)
        if cur is not None:
            row = L.row(align=True)
            row.operator("rig_tweak.reset", icon='LOOP_BACK')
            row.operator("rig_tweak.remove", icon='X')


class RIGTWEAK_MT_saved_chains(bpy.types.Menu):
    bl_label = "保存チェーンからTweakを追加"

    def draw(self, context):
        for preset in context.object.chain_motion_presets:
            self.layout.operator("rig_tweak.add", text=preset.name).preset_name = preset.name


class RIGTWEAK_MT_menu(bpy.types.Menu):
    bl_label = "Rig Tweak"
    bl_idname = "RIGTWEAK_MT_menu"

    def draw(self, context):
        L = self.layout
        L.operator("rig_tweak.add", icon='ADD')
        if context.object.chain_motion_presets:
            L.menu("RIGTWEAK_MT_saved_chains", icon='PRESET')
        L.operator("rig_tweak.toggle", icon='ARROW_LEFTRIGHT')
        if hasattr(bpy.ops, "pivot_rotate") and hasattr(bpy.ops.pivot_rotate, "enable"):   # Pivot Rotate のアドオンが入っていれば
            L.separator()
            L.operator("pivot_rotate.enable", icon='PIVOT_ACTIVE')
            L.operator("pivot_rotate.disable", icon='X')


def _menu(self, context):
    self.layout.separator()
    self.layout.menu("RIGTWEAK_MT_menu")


CLASSES = (RIGTWEAK_OT_add, RIGTWEAK_OT_toggle, RIGTWEAK_OT_reset, RIGTWEAK_OT_remove, RIGTWEAK_OT_select,
           RIGTWEAK_MT_saved_chains, RIGTWEAK_MT_menu, RIGTWEAK_PT_panel)


_initialize_seen = True


def register():
    global _initialize_seen
    _initialize_seen = True
    for c in CLASSES:
        bpy.utils.register_class(c)
    bpy.types.VIEW3D_MT_pose_context_menu.append(_menu)
    if _on_depsgraph not in bpy.app.handlers.depsgraph_update_post:
        bpy.app.handlers.depsgraph_update_post.append(_on_depsgraph)
    if _on_load not in bpy.app.handlers.load_post:
        bpy.app.handlers.load_post.append(_on_load)


def unregister():
    bpy.types.VIEW3D_MT_pose_context_menu.remove(_menu)
    for h, lst in ((_on_depsgraph, bpy.app.handlers.depsgraph_update_post), (_on_load, bpy.app.handlers.load_post)):
        if h in lst:
            lst.remove(h)
    for c in reversed(CLASSES):
        bpy.utils.unregister_class(c)


if __name__ == "__main__":
    register()
