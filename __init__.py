"""Blender Animation Assist. The component operator and data names are stable."""

import addon_utils
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
    # Older packages have no duplicate guard. Even a disabled legacy copy
    # could take over the same RNA classes if enabled after this extension.
    legacy = [mod.__name__ for mod in addon_utils.modules()
              if not mod.__name__.startswith("bl_ext.")
              and mod.bl_info.get("name", "").startswith("Blender Animation Assist (BAA)")]
    if legacy:
        raise RuntimeError("旧BAAをプリファレンスからアンインストールし、Blenderを再起動してください: "
                           + ", ".join(legacy))
    if getattr(bpy.types, VIEW3D_PT_baa_root.__name__, None) is not None:
        raise RuntimeError("別のBAAがすでに有効です。使用するBAAを1つだけ有効にしてください")
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
    if getattr(bpy.types, VIEW3D_PT_baa_root.__name__, None) is not VIEW3D_PT_baa_root:
        return
    for part in reversed(COMPONENTS):
        part.unregister()
    bpy.utils.unregister_class(VIEW3D_PT_baa_root)
