# reuse-checked: Tools\pivot_rotate.py の公開用の写し（社内向けの見出しだけ外す。中身は同じ）
# Pivot Rotate — 任意のコントローラーを中心に、IKのまま全身を回す（Cascadeurのピボット回転。リグは変えない）
#
# 使い方:
#   1. ポーズモードで、中心にしたいコントローラーを選ぶ
#   2. 右クリックメニューの「選択中をPivotにする」を押す
#   3. いつものギズモか R で回す。キーは必要なら自分で打つ（このアドオンはキーを打たない）
#   4. 終わったら右クリックメニューの「Pivot解除」（元の選択とピボットポイントの設定に戻す）
# 中身: 全身を動かす最上位のコントローラーを選び、Pivotの骨をアクティブにして、
#   ピボットポイントを「アクティブ要素」にするだけ。回転は Blender の標準の変形で行う。
#   Auto-Rig Pro のリグは TARGETS（骨盤・足IK・手IK・ポール）を選ぶ。それ以外のリグは、親の無い骨（ルート）を選ぶ。
#   他の選んだ骨に Child Of で付いている骨（ARPの c_arms_pole は c_root_master.x に付く）は、二重に回るので選ばない。
#   FKの骨は親が選ばれていれば Blender が二重に回さない。

bl_info = {
    "name": "Pivot Rotate",
    "author": "NAN",
    "version": (0, 3, 0),
    "blender": (5, 2, 0),
    "location": "3Dビュー > ポーズモードの右クリックメニュー",
    "description": "選んだコントローラーを中心に、ギズモ・R でIKのまま全身を回す",
    "category": "Animation",
}

import json

import bpy


TARGETS = ["c_root_master.x", "c_foot_ik.l", "c_foot_ik.r", "c_hand_ik.l", "c_hand_ik.r",
           "c_leg_pole.l", "c_leg_pole.r", "c_arms_pole.l", "c_arms_pole.r"]   # Auto-Rig Pro
class BAAPivotState(bpy.types.PropertyGroup):
    rig: bpy.props.PointerProperty(type=bpy.types.Object)
    selected: bpy.props.StringProperty()
    active: bpy.props.StringProperty()
    pivot_point: bpy.props.StringProperty()
    pivot: bpy.props.StringProperty()


def is_active(scene, rig):
    state = getattr(scene, "baa_pivot", None)
    return state is not None and rig is not None and state.rig == rig


def restore(scene):
    state = scene.baa_pivot
    rig = state.rig
    if rig and rig.pose:
        selected = set(json.loads(state.selected))
        for bone in rig.pose.bones:
            _set_sel(bone, bone.name in selected)
        rig.data.bones.active = rig.data.bones.get(state.active)
    scene.tool_settings.transform_pivot_point = state.pivot_point
    scene.property_unset("baa_pivot")


def _sel(pb):
    return pb.select if hasattr(pb, "select") else pb.bone.select


def _set_sel(pb, v):
    if hasattr(pb, "select"):
        pb.select = v
    else:
        pb.bone.select = v


def _follows(pb, names):
    """names のどれかに Child Of で付いている（効きが半分以上）"""
    return any(c.type == 'CHILD_OF' and not c.mute and c.influence > 0.5 and c.subtarget in names for c in pb.constraints)


def _targets(rig):
    names = [n for n in TARGETS if n in rig.pose.bones]
    if not names:   # ARP以外: 親の無い骨
        names = [b.name for b in rig.data.bones if b.parent is None]
    return [n for n in names if not _follows(rig.pose.bones[n], set(names) - {n})]


class PIVOTROT_OT_enable(bpy.types.Operator):
    """アクティブなコントローラーを中心に、全身をギズモ・R で回せるようにする"""
    bl_idname = "pivot_rotate.enable"
    bl_label = "選択中をPivotにする"
    bl_options = {'REGISTER', 'UNDO'}

    @classmethod
    def poll(cls, context):
        o = context.object
        return context.scene.baa_pivot.rig in (None, o) and o is not None and o.type == 'ARMATURE' and context.mode == 'POSE' and context.active_pose_bone is not None

    def execute(self, context):
        rig = context.object
        pivot = context.active_pose_bone.name
        ts = context.scene.tool_settings
        state = context.scene.baa_pivot
        if not state.rig:
            state.selected = json.dumps([pb.name for pb in rig.pose.bones if _sel(pb)])
            state.active = pivot
            state.pivot_point = ts.transform_pivot_point
            state.rig = rig
        chosen = _targets(rig)
        for pb in rig.pose.bones:
            _set_sel(pb, pb.name in chosen or pb.name == pivot)
        rig.data.bones.active = rig.data.bones[pivot]
        ts.transform_pivot_point = 'ACTIVE_ELEMENT'
        state.pivot = pivot
        self.report({'INFO'}, f"Pivot: {pivot}（{len(chosen)} 本を選択）")
        return {'FINISHED'}


class PIVOTROT_OT_disable(bpy.types.Operator):
    """有効化する前の選択とピボットポイントに戻す"""
    bl_idname = "pivot_rotate.disable"
    bl_label = "Pivot解除"
    bl_options = {'REGISTER', 'UNDO'}

    @classmethod
    def poll(cls, context):
        return context.scene.baa_pivot.rig is not None

    def execute(self, context):
        from . import body_pose_assist, chain_motion
        rig = context.scene.baa_pivot.rig
        restore(context.scene)
        body_pose_assist.resume_after_pivot(context, rig)
        if chain_motion._edit and chain_motion._edit.rig == rig:
            chain_motion._edit.needs_rebase = True
        return {'FINISHED'}


def _menu(self, context):
    """ポーズモードの右クリックメニュー（タブを開かずに使う）"""
    L = self.layout
    L.separator()
    if context.scene.baa_pivot.rig:
        L.operator("pivot_rotate.disable", text=f"Pivot解除（{context.scene.baa_pivot.pivot}）", icon='X')
    L.operator("pivot_rotate.enable", icon='PIVOT_ACTIVE')


CLASSES = (BAAPivotState, PIVOTROT_OT_enable, PIVOTROT_OT_disable)


def register():
    for c in CLASSES:
        bpy.utils.register_class(c)
    bpy.types.Scene.baa_pivot = bpy.props.PointerProperty(type=BAAPivotState)
    bpy.types.VIEW3D_MT_pose_context_menu.append(_menu)


def unregister():
    bpy.types.VIEW3D_MT_pose_context_menu.remove(_menu)
    for scene in {window.scene for window in bpy.context.window_manager.windows}:
        if scene.baa_pivot.rig:
            restore(scene)
    del bpy.types.Scene.baa_pivot
    for c in reversed(CLASSES):
        bpy.utils.unregister_class(c)


if __name__ == "__main__":
    register()
