bl_info = {
    "name": "Pose Assist",
    "author": "Codex",
    "version": (0, 9, 1),
    "blender": (5, 2, 0),
    "location": "3D View > Sidebar > Pose Assist",
    "category": "Animation",
}

from importlib import import_module
from math import atan2, copysign, degrees, pi

import bpy
from . import pivot_rotate
from bpy.app.handlers import persistent
from bpy_extras.anim_utils import AutoKeying, action_get_channelbag_for_slot
from mathutils import Quaternion, Vector


SWITCH = '["ik_fk_switch"]'


def rig_functions(context):
    package = next(
        (key for key in context.preferences.addons.keys() if key.split(".")[-1] == "auto_rig_pro"),
        None,
    )
    if not package:
        raise ValueError("Auto-Rig Proを有効にしてください")
    return import_module(package + ".src.rig_functions")


def selected_limb(context):
    active = context.active_pose_bone
    if active is None:
        raise ValueError("腕または脚のコントローラーを選択してください")
    arp = rig_functions(context)
    side = arp.get_bone_side(active.name)
    for limb, controllers in (("arm", arp.ik_arm + arp.fk_arm),
                              ("leg", arp.ik_leg + arp.fk_leg)):
        if arp.is_selected(controllers, active.name):
            goal_name = ("c_hand_ik" if limb == "arm" else "c_foot_ik") + side
            goal = context.object.pose.bones.get(goal_name)
            if goal and "ik_fk_switch" in goal:
                return arp, limb, side, goal
    raise ValueError("腕または脚のコントローラーを選択してください")


def switch_curve(rig, goal):
    anim = rig.animation_data
    if not anim or not anim.action or not anim.action_slot:
        return None
    bag = action_get_channelbag_for_slot(anim.action, anim.action_slot)
    if bag:
        return bag.fcurves.find(goal.path_from_id() + SWITCH)
    return None


def match_hand_key(hand, scene):
    """Choose the hemisphere of a newly recorded quaternion using its neighboring key."""
    if hand.rotation_mode != "QUATERNION":
        return
    anim = hand.id_data.animation_data
    if not anim or not anim.action or not anim.action_slot:
        return
    bag = action_get_channelbag_for_slot(anim.action, anim.action_slot)
    if not bag:
        return
    path = hand.path_from_id("rotation_quaternion")
    curves = [bag.fcurves.find(path, index=i) for i in range(4)]
    if any(not fc or fc.lock or fc.mute or fc.modifiers for fc in curves):
        return
    frame = anim.nla_tweak_strip_time_to_scene(scene.frame_current_final, invert=True)
    times = {p.co.x for fc in curves for p in fc.keyframe_points}
    if frame not in times or len(times) < 2:
        return
    previous = max((t for t in times if t < frame), default=None)
    ref = previous if previous is not None else min(t for t in times if t > frame)
    values = [fc.evaluate(frame) for fc in curves]
    if sum(v * fc.evaluate(ref) for v, fc in zip(values, curves)) >= 0:
        return
    for fc, v in zip(curves, values):
        point = next((p for p in fc.keyframe_points if p.co.x == frame), None)
        if point is None:
            fc.keyframe_points.insert(frame, -v)
        else:
            left, right = point.handle_left.y, point.handle_right.y
            point.co.y = -v
            point.handle_left.y, point.handle_right.y = -left, -right
        fc.update()


def switch_limb(context, arp, limb, side, goal):
    rig = context.object
    anim = rig.animation_data
    if anim and anim.use_tweak_mode:
        raise ValueError("Action Editorで対象Actionを開いて実行してください")
    frame = context.scene.frame_current
    if context.scene.frame_subframe:
        raise ValueError("整数フレームで実行してください")
    old = goal["ik_fk_switch"]
    if old not in (0.0, 1.0):
        raise ValueError("IK/FKがブレンド中です。IKまたはFKのフレームを選んでください")
    direction = "fk_to_ik" if old == 0 else "ik_to_fk"
    snap = getattr(arp, f"{direction}_{limb}", None)
    if snap is None:
        raise ValueError("このリグの同期処理に対応していません")

    curve = switch_curve(rig, goal)
    if curve and (curve.lock or curve.mute or curve.modifiers):
        raise ValueError("切替カーブがロック・ミュート中か、モディファイア付きです")
    initial_mode = curve.evaluate(0) if curve and len(curve.keyframe_points) else old
    chain = (("arm_stretch", "forearm_stretch", "hand") if limb == "arm"
             else ("thigh_stretch", "leg_stretch", "foot"))
    rotations = {pb.name: pb.matrix.to_quaternion() for base in chain
                 if (pb := rig.pose.bones.get(base + side)) is not None}
    mirror = rig.pose.use_mirror_x
    try:
        rig.pose.use_mirror_x = False
        snap(rig, side, add_keyframe=True)
        if limb == "arm":
            for hand in (goal, rig.pose.bones[arp.fk_arm[2] + side]):
                match_hand_key(hand, context.scene)
        curve = switch_curve(rig, goal)
        if curve is None or goal["ik_fk_switch"] != 1 - old:
            raise RuntimeError("切替キーを確認できませんでした")
        # A switch is a step. Use its preceding authored key as the
        # beginning of that step; only the first switch needs a baseline.
        previous = next((p for p in reversed(list(curve.keyframe_points)) if p.co.x < frame), None)
        if previous is not None:
            previous.interpolation = "CONSTANT"
        elif frame > 0:
            curve.keyframe_points.insert(0, initial_mode).interpolation = "CONSTANT"
        for point in curve.keyframe_points:
            if point.co.x == frame:
                point.interpolation = "CONSTANT"
                break
        else:
            raise RuntimeError("現在フレームに切替キーがありません")
        curve.update()
    finally:
        rig.pose.use_mirror_x = mirror
    return rotations


class POSE_OT_arp_assist_keyed_switch(bpy.types.Operator):
    bl_idname = "pose.arp_assist_keyed_switch"
    bl_label = "IK/FKを切り替えてキーを挿入"
    bl_description = "アクティブな腕・脚を現在の姿勢で切り替え、同期したコントローラーをキーに記録"
    bl_options = {"UNDO"}

    @classmethod
    def poll(cls, context):
        return context.mode == "POSE" and context.active_pose_bone is not None

    def execute(self, context):
        try:
            arp, limb, side, goal = selected_limb(context)
        except (ValueError, ImportError, AttributeError) as error:
            self.report({"WARNING"}, str(error))
            return {"CANCELLED"}

        old = goal["ik_fk_switch"]
        frame = context.scene.frame_current
        rig = context.object
        try:
            rotations = switch_limb(context, arp, limb, side, goal)
        except ValueError as error:
            self.report({"WARNING"}, str(error))
            return {"CANCELLED"}
        except Exception as error:
            self.report({"WARNING"}, f"切替を完了できませんでした。元に戻すで取消 ({error})")
            return {"FINISHED"}
        mode = "FK" if old == 0 else "IK"
        context.view_layer.update()
        differences = []
        for name, before in rotations.items():
            angle = before.rotation_difference(rig.pose.bones[name].matrix.to_quaternion()).angle
            differences.append((degrees(min(angle, 2 * pi - angle)), name))
        delta, bone = max(differences, default=(0, ""))
        message = f"{goal.name} → {mode}  ({frame}F)"
        if delta > 1:
            self.report({"WARNING"}, f"{message}  姿勢差 {delta:.2f}° ({bone})  Ctrl+Zで取消")
        else:
            self.report({"INFO"}, message)
        return {"FINISHED"}


def smart_ik_bones(context):
    arp, limb, side, goal = selected_limb(context)
    if not arp.is_object_arp(context.object) or goal != context.active_pose_bone or goal["ik_fk_switch"] != 0:
        raise ValueError("IKモードの手または足のIKコントローラーを選んでください")
    keep_angle = context.window_manager.arp_assist_keep_angle
    if any(goal.lock_location) or (keep_angle and (any(goal.lock_rotation) or
            (goal.lock_rotations_4d and goal.lock_rotation_w))):
        raise ValueError("IKコントローラーの移動・回転チャンネルにロックがあります")
    proximal, end = ("forearm_stretch", "hand") if limb == "arm" else ("leg_stretch", "foot")
    bones = context.object.pose.bones
    if proximal + side not in bones or end + side not in bones:
        raise ValueError("対応する手首または足首のボーンが見つかりません")
    chain = None
    if context.window_manager.arp_assist_follow_pole:
        root, target, pole = (("arm_ik", "c_hand_ik", "c_arms_pole") if limb == "arm"
                              else ("thigh_ik", "foot_ik_target", "c_leg_pole"))
        chain = tuple(bones.get(name + side) for name in (root, target, pole))
        if None in chain or any(chain[2].lock_location):
            raise ValueError("対応する肘・膝のポールまたは移動可能なIKチェーンが見つかりません")
    return goal, bones[proximal + side], bones[end + side], chain


def can_move_ik(context):
    if pivot_rotate.is_active(context.scene, context.object):
        return False
    bone = context.active_pose_bone
    if context.mode != "POSE" or bone is None or not bone.name.startswith(("c_hand_ik", "c_foot_ik")):
        return False
    try:
        smart_ik_bones(context)
    except (ValueError, ImportError, AttributeError):
        return False
    return True


class PoleFollow:
    """Transport the pole about the shoulder–wrist or hip–ankle axis in pose space."""

    def __init__(self, root, target, pole, *, arm=False):
        self.root, self.target, self.pole = root, target, pole
        self.start_location = pole.location.copy()
        self.shoulder_radius = 0.30 * root.length if arm else 0.0
        self.start_upper = root.matrix.to_quaternion() if arm else None
        self.upper_rotation = self.start_upper
        aim = target.head - root.head
        if aim.length < 1e-7:
            aim = root.bone.tail_local - root.bone.head_local
        self.start_axis = aim.normalized()
        offset = pole.head - root.head
        axial = offset.dot(self.start_axis)
        self.axial_fraction = axial / aim.length
        self.start_radial = offset - self.start_axis * axial
        if self.start_radial.length < 1e-7:
            raise ValueError("ポールがリムの軸上にあります。曲げたい側に少し移動してください")
        self.axis = self.start_axis.copy()
        self.radial = self.start_radial.copy()

    def restore(self):
        self.pole.location = self.start_location
        self.upper_rotation = self.start_upper
        self.axis = self.start_axis.copy()
        self.radial = self.start_radial.copy()

    def apply(self):
        aim = self.target.head - self.root.head
        if aim.length_squared > 1e-14:
            axis = aim.normalized()
            dot = self.axis.dot(axis)
            radial = self.radial.copy()
            if dot > -0.999999:
                radial = self.axis.rotation_difference(axis) @ radial
            if self.shoulder_radius and dot < -0.25:
                # Near a reversal the cross product cannot define a reliable turn axis.
                # Transport from the reversed aim, preserving the previous elbow side.
                stable = (-self.axis).rotation_difference(axis) @ self.radial
                weight = min(1.0, (-0.25 - dot) / 0.55)
                if weight == 1.0:
                    radial = stable
                else:
                    angle = atan2(axis.dot(radial.cross(stable)), radial.dot(stable))
                    radial = Quaternion(axis, weight * angle) @ radial
            radial -= axis * radial.dot(axis)
            radial.normalize()
            radial *= self.start_radial.length
            self.axis, self.radial = axis, radial
        else:
            radial = self.radial
        desired = self.root.head + aim * self.axial_fraction + radial
        delta = desired - self.pole.head
        if delta.length_squared < 1e-14:
            return False
        # The pole's local Child Of space can move with the foot during a drag.
        space = self.pole.matrix @ self.pole.matrix_basis.inverted()
        self.pole.location += space.to_3x3().inverted() @ delta
        return True

    def follow(self, layer):
        if self.apply():
            layer.update()
        if self.upper_rotation is None:
            return
        aim = self.target.head - self.root.head
        if 1e-7 < aim.length < self.shoulder_radius:
            # At a folded elbow the solver's roll is underdetermined. Of the
            # rotations around the reach axis choose the nearest evaluated upper arm.
            residual = self.upper_rotation @ self.root.matrix.to_quaternion().inverted()
            vector = Vector((residual.x, residual.y, residual.z))
            twist = 2 * atan2(vector.dot(self.axis), residual.w)
            twist = (twist + pi) % (2 * pi) - pi
            weight = min(1.0, 2 * (1 - aim.length / self.shoulder_radius))
            if abs(weight * twist) > 1e-4:
                self.radial = Quaternion(self.axis, weight * twist) @ self.radial
                if self.apply():
                    layer.update()
        self.upper_rotation = self.root.matrix.to_quaternion()


class SmartIKMove:
    """Store the evaluated end bone's rotation relative to the preceding limb segment."""

    def __init__(self, context):
        goal, self.proximal, self.end, chain = smart_ik_bones(context)
        self.goal = goal
        self.view_layer = context.view_layer
        self.view_layer.update()
        self.keep_angle = context.window_manager.arp_assist_keep_angle
        self.pole = PoleFollow(*chain, arm=goal.name.startswith("c_hand_ik")) if chain else None
        self.start_location = goal.location.copy()
        self.rotation_path = {"QUATERNION": "rotation_quaternion",
                              "AXIS_ANGLE": "rotation_axis_angle"}.get(goal.rotation_mode, "rotation_euler")
        self.start_rotation = tuple(getattr(goal, self.rotation_path))
        self.relative = self.proximal.matrix.to_quaternion().inverted() @ self.end.matrix.to_quaternion()
        # Includes the controller's active Child Of space, rest orientation and parent scale.
        channel_space = goal.matrix @ goal.matrix_basis.inverted()
        self.pose_to_location = channel_space.to_3x3().inverted()
        self.world_to_pose = context.object.matrix_world.to_3x3().inverted()
        self.origin = context.object.matrix_world @ goal.matrix.translation

    def restore(self):
        self.goal.location = self.start_location
        if self.pole:
            self.pole.restore()
        setattr(self.goal, self.rotation_path, self.start_rotation)
        self.view_layer.update()

    def move(self, world_delta):
        if world_delta.length_squared < 1e-16:
            self.restore()
            return
        self.goal.location = self.start_location + self.pose_to_location @ (self.world_to_pose @ world_delta)
        self.view_layer.update()
        self.follow_segment()

    def follow_segment(self):
        if tuple(self.goal.location) == tuple(self.start_location):
            if ((self.pole and tuple(self.pole.pole.location) != tuple(self.pole.start_location))
                    or tuple(getattr(self.goal, self.rotation_path)) != self.start_rotation):
                self.restore()
            return
        # The forearm used for wrist matching must include this drag's shoulder response,
        # including when a short G/gizmo move finishes between body timer callbacks.
        from . import body_pose_assist
        body_pose_assist.preview_hand_move(self.goal)
        for iteration in range(6 if self.pole else 4):
            if self.pole:
                if iteration == 0:
                    self.pole.follow(self.view_layer)
                elif self.pole.apply():
                    self.view_layer.update()
            if not self.keep_angle:
                return
            desired = self.proximal.matrix.to_quaternion() @ self.relative
            correction = desired @ self.end.matrix.to_quaternion().inverted()
            if correction.x**2 + correction.y**2 + correction.z**2 < 0.00005**2:
                return
            local = self.goal.matrix_basis.to_quaternion()
            space = self.goal.matrix.to_quaternion() @ local.inverted()
            rotation = space.inverted() @ correction @ space @ local
            rotation.normalize()
            if self.rotation_path == "rotation_quaternion":
                rotation.make_compatible(self.goal.rotation_quaternion)
                self.goal.rotation_quaternion = rotation
            elif self.rotation_path == "rotation_axis_angle":
                axis, angle = rotation.to_axis_angle()
                self.goal.rotation_axis_angle = (angle, *axis)
            else:
                self.goal.rotation_euler = rotation.to_euler(self.goal.rotation_mode, self.goal.rotation_euler)
            self.view_layer.update()


    def autokey(self, context, *, location):
        from . import body_pose_assist
        body_pose_assist.commit_hand_preview(self.goal)
        kind = context.scene.tool_settings.keyframe_type
        if location or self.keep_angle:
            with AutoKeying.options(keytype=kind, use_loc=location, use_rot=self.keep_angle, use_scale=False):
                AutoKeying.autokey_transformation(context, self.goal)
            if self.keep_angle and self.goal.name.startswith("c_hand_ik"):
                match_hand_key(self.goal, context.scene)
        if self.pole and (self.pole.pole.location - self.pole.start_location).length_squared > 1e-14:
            with AutoKeying.options(keytype=kind, use_rot=False, use_scale=False):
                AutoKeying.autokey_transformation(context, self.pole.pole)


class POSE_OT_arp_assist_move_wrist(bpy.types.Operator):
    bl_idname = "pose.arp_assist_move_wrist"
    bl_label = "スマートIKを直接ドラッグ"
    bl_description = "手足のIKを動かし、設定に応じて手首・足首の角度と肘・膝の曲がる側を保つ"
    bl_options = {"UNDO"}

    @classmethod
    def poll(cls, context):
        return can_move_ik(context)

    def invoke(self, context, event):
        if context.area.type != "VIEW_3D" or context.region.type != "WINDOW" or not context.region_data:
            self.report({"WARNING"}, "3Dビューから実行してください")
            return {"CANCELLED"}
        try:
            self.drag = SmartIKMove(context)
        except (ValueError, ImportError, AttributeError) as error:
            self.report({"WARNING"}, str(error))
            return {"CANCELLED"}
        self.region = context.region
        self.region_view = context.region_data
        self.anchor = None
        self.offset = Vector((0, 0, 0))
        self.axis = None
        context.area.header_text_set("左ドラッグ IKを移動  |  中ドラッグ 視点回転  |  X/Y/Z 軸指定  |  Enter 確定  Esc 取消")
        context.window_manager.modal_handler_add(self)
        return {"RUNNING_MODAL"}

    def start_stroke(self, context, event):
        r = self.region
        x, y = event.mouse_x, event.mouse_y
        if not (r.x <= x < r.x + r.width and r.y <= y < r.y + r.height):
            return
        for region in context.area.regions:
            if region.type in {"UI", "TOOLS", "HEADER", "TOOL_HEADER"} and region.width > 1:
                if region.x <= x < region.x + region.width and region.y <= y < region.y + region.height:
                    return
        self.anchor = Vector((x, y))
        self.stroke_offset = self.offset.copy()
        self.projection = self.region_view.perspective_matrix.copy()

    def update_pointer(self, event):
        current = Vector((event.mouse_x, event.mouse_y))
        view = self.region_view
        if self.projection != view.perspective_matrix:
            # A smooth view change can outlive the navigation event.
            self.projection = view.perspective_matrix.copy()
            self.stroke_offset = self.offset.copy()
            self.anchor = current
            return
        pixels = current - self.anchor
        origin = self.drag.origin + self.stroke_offset
        w = (self.projection @ origin.to_4d()).w
        # Screen displacement at the goal's clip depth. Works for orthographic
        # and perspective views without constructing a ray at the far clip plane.
        clip_dx = 2 * w * pixels.x / self.region.width
        clip_dy = 2 * w * pixels.y / self.region.height
        inverse = self.projection.inverted()
        delta = (inverse @ Vector((clip_dx, clip_dy, 0, 0))).xyz
        if self.axis:
            axis = Vector((self.axis == "X", self.axis == "Y", self.axis == "Z"))
            normal = view.view_rotation @ Vector((0, 0, 1))
            projected = axis - normal * normal.dot(axis)
            if projected.length_squared > 0.001:
                delta = axis * (delta.dot(projected) / projected.length_squared)
            else:
                vertical = (inverse @ Vector((0, clip_dy, 0, 0))).xyz
                delta = axis * copysign(vertical.length, pixels.y)
        self.offset = self.stroke_offset + delta
        self.drag.move(self.offset)

    def cancel(self, context):
        self.drag.restore()
        context.area.header_text_set(None)

    def modal(self, context, event):
        if event.type in {"ESC", "RIGHTMOUSE", "WINDOW_DEACTIVATE"}:
            self.cancel(context)
            return {"CANCELLED"}
        # Blender owns orbit, pan, zoom and preset views. Navigation never
        # contributes pixels to an IK drag, even if it consumes mouse release.
        if event.type.startswith("TIMER"):
            return {"PASS_THROUGH"}
        navigation = (
            event.type in {"MIDDLEMOUSE", "TRACKPADPAN", "TRACKPADZOOM", "MOUSEROTATE", "MOUSESMARTZOOM"}
            or (event.type.startswith(("WHEEL", "NUMPAD_", "NDOF_")) and event.type != "NUMPAD_ENTER")
            or (event.type == "LEFTMOUSE" and event.alt)
        )
        if navigation:
            self.anchor = None
            return {"PASS_THROUGH"}
        if event.type == "LEFTMOUSE":
            if event.value == "PRESS":
                self.start_stroke(context, event)
            elif event.value == "RELEASE" and self.anchor is not None:
                self.update_pointer(event)
                self.anchor = None
            return {"RUNNING_MODAL"}
        if event.type in {"MOUSEMOVE", "INBETWEEN_MOUSEMOVE"}:
            if self.anchor is None:
                return {"PASS_THROUGH"}
            self.update_pointer(event)
        if event.type in {"X", "Y", "Z"} and event.value == "PRESS":
            self.axis = None if self.axis == event.type else event.type
            if self.anchor is not None:
                self.update_pointer(event)
        handoff = event.type == "G" and not any((event.ctrl, event.alt, event.shift, event.oskey))
        if (handoff or event.type in {"RET", "NUMPAD_ENTER"}) and event.value == "PRESS":
            context.area.header_text_set(None)
            moved = self.offset.length_squared >= 1e-16
            if moved and context.scene.tool_settings.use_keyframe_insert_auto:
                try:
                    self.drag.autokey(context, location=True)
                except (RuntimeError, ValueError) as error:
                    self.report({"WARNING"}, f"移動を確定しました。自動キーを完了できませんでした ({error})")
            if handoff:
                # End this modal session so it cannot consume the native grab's mouse events.
                if bpy.ops.pose.arp_assist_translate.poll():
                    bpy.ops.pose.arp_assist_translate("INVOKE_DEFAULT")
                else:
                    bpy.ops.transform.translate("INVOKE_DEFAULT")
            return {"FINISHED"} if moved else {"CANCELLED"}
        return {"RUNNING_MODAL"}


# Pending state lasts only for the native transform. The macro's final step is
# reached on confirmation; on cancellation Blender ends the macro before it.
_native_drags = {}
_keybindings = []


def body_settings(context):
    body = getattr(bpy.types, "POSE_OT_body_pose_assist", None)
    query = getattr(body, "smart_settings", None)
    return query(context) if query else None


def smart_enabled(context):
    if pivot_rotate.is_active(context.scene, context.object):
        return False
    settings = body_settings(context)
    return settings.smart_ik if settings is not None else context.window_manager.arp_assist_follow_g


class GizmoFollow:
    """Keep the pre-drag pose while Blender's native gizmo owns the pointer."""

    def __init__(self, context, identity):
        self.identity = identity
        self.drag = SmartIKMove(context)
        self.moving = False
        self.baseline = self.pose_signature()
        self.last_operator = None

    def pose_signature(self):
        drag = self.drag
        bones = [drag.goal, drag.proximal, drag.end]
        if drag.pole:
            bones.extend((drag.pole.root, drag.pole.target, drag.pole.pole))
        return tuple(tuple(row) for bone in bones for row in bone.matrix)

    def finish(self, context):
        drag = self.drag
        if (drag.goal.location - drag.start_location).length_squared < 1e-12:
            drag.restore()
        else:
            drag.follow_segment()
            if context.scene.tool_settings.use_keyframe_insert_auto and context.scene.frame_current >= 0:
                drag.autokey(context, location=False)


_gizmos = {}


@persistent
def clear_gizmos(*args):
    _gizmos.clear()


def observe_gizmo(context):
    window = context.window
    key = window.as_pointer()
    state = _gizmos.get(key)
    goal = context.active_pose_bone
    if (context.mode != "POSE" or not smart_enabled(context) or window.screen.is_animation_playing
            or not goal or not goal.name.startswith(("c_hand_ik", "c_foot_ik"))
            or goal.get("ik_fk_switch") != 0 or len(context.selected_pose_bones or ()) != 1):
        _gizmos.pop(key, None)
        return
    operators = {op.bl_idname for op in window.modal_operators}
    if operators.intersection({"POSE_OT_arp_assist_translate", "POSE_OT_arp_assist_move_wrist"}):
        # Keyboard moves and the direct-drag tool already own their correction and keys.
        _gizmos.pop(key, None)
        return
    wm = context.window_manager
    last = wm.operators[-1] if wm.operators else None
    last_operator = last.as_pointer() if last else None
    ad = context.object.animation_data
    identity = (context.scene.as_pointer(), context.object.as_pointer(), goal.name,
                context.scene.frame_current_final, ad.action.as_pointer() if ad and ad.action else 0,
                wm.arp_assist_keep_angle, wm.arp_assist_follow_pole)
    moving = "TRANSFORM_OT_translate" in operators or "GIZMOGROUP_OT_gizmo_tweak" in operators
    if state and not moving and state.identity[:4] == identity[:4]:
        # A short native drag can begin and end between two timer callbacks.
        quick_move = (last and last.bl_idname == "TRANSFORM_OT_translate"
                      and last_operator != state.last_operator
                      and (goal.location - state.drag.start_location).length_squared > 1e-14)
        if state.moving or quick_move:
            # Auto Key may allocate the first Action during this release.
            state.finish(context)
            state = None
    if state is None or state.identity != identity or (not moving and state.pose_signature() != state.baseline):
        state = _gizmos[key] = GizmoFollow(context, identity)
    if moving and (state.moving or
                   (goal.location - state.drag.start_location).length_squared > 1e-14):
        state.moving = True
        state.drag.follow_segment()
    state.last_operator = last_operator


def observe_windows():
    c = bpy.context
    live = set()
    for window in c.window_manager.windows:
        key = window.as_pointer()
        live.add(key)
        try:
            with c.temp_override(window=window):
                observe_gizmo(c)
        except (ReferenceError, RuntimeError, ValueError, ImportError, AttributeError):
            _gizmos.pop(key, None)
    for closed in _gizmos.keys() - live:
        del _gizmos[closed]
    return 1 / 60 if _gizmos else .08


def discard_preview(window_id):
    _, drag = _native_drags.pop(window_id)
    try:
        if tuple(drag.goal.location) == tuple(drag.start_location):
            drag.restore()
    except ReferenceError:
        pass  # The window, scene or armature has already been removed.


def preview_native_moves():
    for window_id, (window, drag) in list(_native_drags.items()):
        try:
            if any(op.bl_idname == "POSE_OT_arp_assist_translate" for op in window.modal_operators):
                drag.follow_segment()
            else:
                discard_preview(window_id)
        except ReferenceError:
            _native_drags.pop(window_id, None)
    return 1 / 60 if _native_drags else None


class POSE_OT_arp_assist_begin_translate(bpy.types.Operator):
    bl_idname = "pose.arp_assist_begin_translate"
    bl_label = "IK移動の開始角度を取得"
    bl_options = {"INTERNAL"}

    def execute(self, context):
        window_id = context.window.as_pointer()
        if window_id in _native_drags:
            discard_preview(window_id)
        try:
            drag = SmartIKMove(context)
        except (ValueError, ImportError, AttributeError) as error:
            self.report({"WARNING"}, str(error))
            return {"CANCELLED"}
        _native_drags[window_id] = (context.window, drag)
        if not bpy.app.timers.is_registered(preview_native_moves):
            bpy.app.timers.register(preview_native_moves)
        return {"FINISHED"}


class POSE_OT_arp_assist_end_translate(bpy.types.Operator):
    bl_idname = "pose.arp_assist_end_translate"
    bl_label = "IK移動の角度を確定"
    bl_options = {"INTERNAL"}

    def execute(self, context):
        _, drag = _native_drags.pop(context.window.as_pointer())
        drag.view_layer.update()
        drag.follow_segment()
        settings = context.scene.tool_settings
        if settings.use_keyframe_insert_auto and tuple(drag.goal.location) != tuple(drag.start_location):
            try:
                drag.autokey(context, location=False)
            except (RuntimeError, ValueError) as error:
                self.report({"WARNING"}, f"IKの補正キーを記録できませんでした ({error})")
        return {"FINISHED"}


class POSE_OT_arp_assist_translate(bpy.types.Macro):
    bl_idname = "pose.arp_assist_translate"
    bl_label = "GでスマートIK移動"
    bl_options = {"UNDO"}

    smart_enabled = staticmethod(smart_enabled)

    @classmethod
    def poll(cls, context):
        if not (smart_enabled(context) and context.area
                and context.area.type == "VIEW_3D" and context.region
                and context.region.type == "WINDOW" and context.mode == "POSE"):
            return False
        selected = context.selected_pose_bones or ()
        return len(selected) == 1 and selected[0] == context.active_pose_bone and can_move_ik(context)


classes = (POSE_OT_arp_assist_keyed_switch, POSE_OT_arp_assist_move_wrist,
           POSE_OT_arp_assist_begin_translate, POSE_OT_arp_assist_end_translate,
           POSE_OT_arp_assist_translate)


def register():
    for cls in classes:
        bpy.utils.register_class(cls)
    bpy.types.WindowManager.arp_assist_keep_angle = bpy.props.BoolProperty(
        name="手首・足首の角度を保つ", default=True, options={"SKIP_SAVE"})
    bpy.types.WindowManager.arp_assist_follow_pole = bpy.props.BoolProperty(
        name="肘・膝のポールを追従", default=True, options={"SKIP_SAVE"},
        description="手足を動かす方向に合わせて、リムの曲がる側にポールを運びます")
    bpy.types.WindowManager.arp_assist_follow_g = bpy.props.BoolProperty(
        name="スマートIK移動", default=False, options={"SKIP_SAVE"},
        description="手足のIKを1本選択し、Gまたは移動ギズモで角度保持・ポール追従を使います")
    bpy.app.timers.register(observe_windows, persistent=True)
    for handlers in (bpy.app.handlers.load_pre, bpy.app.handlers.undo_pre, bpy.app.handlers.redo_pre):
        handlers.append(clear_gizmos)
    for step in ("POSE_OT_arp_assist_begin_translate", "TRANSFORM_OT_translate", "POSE_OT_arp_assist_end_translate"):
        POSE_OT_arp_assist_translate.define(step)
    config = bpy.context.window_manager.keyconfigs.addon
    if config:
        km = config.keymaps.new(name="Pose", space_type="EMPTY")
        item = km.keymap_items.new(POSE_OT_arp_assist_translate.bl_idname, "G", "PRESS", head=True)
        _keybindings.append((km, item))


def unregister():
    if bpy.app.timers.is_registered(observe_windows):
        bpy.app.timers.unregister(observe_windows)
    for handlers in (bpy.app.handlers.load_pre, bpy.app.handlers.undo_pre, bpy.app.handlers.redo_pre):
        handlers.remove(clear_gizmos)
    clear_gizmos()
    for km, item in _keybindings:
        km.keymap_items.remove(item)
    _keybindings.clear()
    if bpy.app.timers.is_registered(preview_native_moves):
        bpy.app.timers.unregister(preview_native_moves)
    for window_id in list(_native_drags):
        discard_preview(window_id)
    del bpy.types.WindowManager.arp_assist_follow_pole
    del bpy.types.WindowManager.arp_assist_keep_angle
    del bpy.types.WindowManager.arp_assist_follow_g
    for cls in reversed(classes):
        bpy.utils.unregister_class(cls)


if __name__ == "__main__":
    register()
