bl_info = {
    "name": "Mirror Pose",
    "author": "Codex",
    "version": (0, 1, 0),
    "blender": (5, 2, 0),
    "location": "3D View > Sidebar > Pose Assist",
    "category": "Animation",
}

import bpy
from bpy_extras.anim_utils import AutoKeying, action_get_channelbag_for_slot
from mathutils import Quaternion, Vector


def mirrored_rotation(source, target):
    """Reflect a controller's local rotation across X, in the target's rotation mode."""
    mode = target.rotation_mode
    if mode == source.rotation_mode and mode not in {"QUATERNION", "AXIS_ANGLE"}:
        x, y, z = source.rotation_euler
        return "rotation_euler", (x, -y, -z), target.lock_rotation

    if source.rotation_mode == "QUATERNION":
        q = source.rotation_quaternion.normalized()
    elif source.rotation_mode == "AXIS_ANGLE":
        angle, x, y, z = source.rotation_axis_angle
        q = Quaternion(Vector((x, y, z)), angle)
    else:
        q = source.rotation_euler.to_quaternion()
    q = Quaternion((q.w, q.x, -q.y, -q.z))
    if mode == "QUATERNION":
        q.make_compatible(target.rotation_quaternion)
        return "rotation_quaternion", q, AutoKeying.get_4d_rotlock(target)
    if mode == "AXIS_ANGLE":
        axis, angle = q.to_axis_angle()
        return "rotation_axis_angle", (angle, *axis), AutoKeying.get_4d_rotlock(target)
    return "rotation_euler", q.to_euler(mode, target.rotation_euler), target.lock_rotation


class POSE_OT_arp_assist_mirror_keyed(bpy.types.Operator):
    bl_idname = "pose.arp_assist_mirror_keyed"
    bl_label = "反対側へポーズをコピー＋キー"
    bl_description = "選択したコントローラーの位置・回転・スケールを左右反転し、反対側に現在フレームのキーを挿入"
    bl_options = {"UNDO"}

    @classmethod
    def poll(cls, context):
        return context.mode == "POSE" and bool(context.selected_pose_bones)

    def execute(self, context):
        rig = context.object
        selected = {p.name: p for p in context.selected_pose_bones if p.id_data == rig}
        if len(selected) != len(context.selected_pose_bones):
            self.report({"WARNING"}, "1つのリグでコピー元を選択してください")
            return {"CANCELLED"}
        targets = []
        for name, source in selected.items():
            opposite = bpy.utils.flip_name(name)
            target = rig.pose.bones.get(opposite)
            if not name.startswith("c_") or opposite == name or target is None:
                continue
            if opposite in selected:
                self.report({"WARNING"}, f"{name} と {opposite} のうち、コピー元だけを選んでください")
                return {"CANCELLED"}
            targets.append((source, target))
        if not targets:
            self.report({"WARNING"}, "左右に対応するコントローラーを選択してください")
            return {"CANCELLED"}

        anim = rig.animation_data
        bag = None
        if anim and anim.action and anim.action_slot:
            bag = action_get_channelbag_for_slot(anim.action, anim.action_slot)
        driven = {(fc.data_path, fc.array_index) for fc in anim.drivers} if anim else set()
        writes = []
        for source, target in targets:
            x, y, z = source.location
            channels = [
                mirrored_rotation(source, target),
                ("scale", tuple(source.scale), target.lock_scale),
            ]
            if not target.bone.use_connect:
                channels.append(("location", (-x, y, z), target.lock_location))
            for prop, values, locks in channels:
                path = target.path_from_id(prop)
                for index, value in enumerate(values):
                    if locks[index] or (path, index) in driven:
                        continue
                    curve = bag.fcurves.find(path, index=index) if bag else None
                    if curve and (curve.lock or (curve.group and curve.group.lock)):
                        self.report({"WARNING"}, f"{target.name} の {prop} にロックされたカーブがあります")
                        return {"CANCELLED"}
                    writes.append((target, prop, index, value))
        if not writes:
            self.report({"WARNING"}, "貼り付け先に編集可能なチャンネルがありません")
            return {"CANCELLED"}

        try:
            for target, prop, index, value in writes:
                getattr(target, prop)[index] = value
            for target, prop, index, _ in writes:
                if not target.keyframe_insert(data_path=prop, index=index, group=target.name,
                                              keytype=context.scene.tool_settings.keyframe_type):
                    raise RuntimeError(f"{target.name} / {prop}[{index}]")
        except (RuntimeError, TypeError, ValueError) as error:
            self.report({"WARNING"}, f"記録を完了できませんでした。1回のUndoで取り消せます ({error})")
            return {"FINISHED"}
        context.view_layer.update()
        count = len({target.name for target, _, _, _ in writes})
        self.report({"INFO"}, f"反対側の {count} 本にキーを挿入しました")
        return {"FINISHED"}


classes = (POSE_OT_arp_assist_mirror_keyed,)


def register():
    for cls in classes:
        bpy.utils.register_class(cls)


def unregister():
    for cls in reversed(classes):
        bpy.utils.unregister_class(cls)


if __name__ == "__main__":
    register()
