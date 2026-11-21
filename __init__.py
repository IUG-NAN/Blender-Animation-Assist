"""Blender Animation Assist. The component operator and data names are stable."""

bl_info = {
    "name": "Blender Animation Assist (BAA) プレビュー版",
    "author": "IUG-NAN, Codex",
    "version": (1, 5, 2),
    "blender": (5, 2, 0),
    "location": "3D View > BAA / Pose Context Menu",
    "description": "ポーズ、チェーン、全身の回転、既存アニメーションの追加調整",
    "category": "Animation",
}

import bpy

from . import arp_pose_assist, arp_mirror_pose, chain_motion, body_pose_assist, pivot_rotate, rig_tweak, pose_borrow

COMPONENTS = (arp_pose_assist, arp_mirror_pose, chain_motion, body_pose_assist, pivot_rotate, rig_tweak, pose_borrow)


class VIEW3D_PT_baa_root(bpy.types.Panel):
    bl_space_type = "VIEW_3D"
    bl_region_type = "UI"
    bl_category = "BAA"
    bl_label = "Blender Animation Assist"

    def draw_header(self, context):
        self.layout.label(text="", icon="ACTION")

    def draw(self, context):
        self.layout.label(text="プレビュー版")


def register():
    conflicts = [part.__name__.rsplit(".", 1)[-1] for part in COMPONENTS
                 if part.__name__.rsplit(".", 1)[-1] in bpy.context.preferences.addons]
    if conflicts:
        raise RuntimeError("先に旧版のアドオンを無効にしてください: " + ", ".join(conflicts))
    registered = []
    bpy.utils.register_class(VIEW3D_PT_baa_root)
    try:
        for part in COMPONENTS:
            part.register()
            registered.append(part)
    except Exception:
        for part in reversed(registered):
            part.unregister()
        bpy.utils.unregister_class(VIEW3D_PT_baa_root)
        raise


def unregister():
    for part in reversed(COMPONENTS):
        part.unregister()
    bpy.utils.unregister_class(VIEW3D_PT_baa_root)
