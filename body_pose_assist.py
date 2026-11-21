"""Interactive stance and reach corrections for directly posed control rigs.

Enable in Pose Assist, move or rotate a control, and use Blender Auto Key to
record the result. Rig mappings and the optional saved spine chain are per rig.
"""

bl_info = {
    "name": "PoseAssist",
    "author": "Codex",
    "version": (0, 4, 0),
    "blender": (5, 2, 0),
    "location": "3D View > Pose Assist > PoseAssist",
    "category": "Animation",
}

import re
from math import asin, radians, sqrt, tanh

import bpy
from bpy.app.handlers import persistent
from bpy_extras.anim_utils import AutoKeying
from mathutils import Quaternion, Vector

from . import arp_pose_assist, pivot_rotate

UP = Vector((0, 0, 1))
IDENTITY = Quaternion()
EPS = 1e-7
LEG = {
    "hip": ("股関節", "thigh_ik_nostr"),
    "shin": ("すね", "leg_ik_nostr"),
    "ankle": ("足首のIK目標", "foot_ik_target"),
    "foot": ("足IK操作骨", "c_foot_ik"),
}
ARM = {
    "clav": ("鎖骨操作骨", "c_shoulder"),
    "upper": ("計測する上腕", "arm"),
    "fk": ("上腕FK操作骨", "c_arm_fk"),
    "hand": ("手IK操作骨", "c_hand_ik"),
}
_session = None
_workflow = None


class BodyLimbMap(bpy.types.PropertyGroup):
    mode: bpy.props.EnumProperty(
        name="腕のモード",
        items=[("AUTO", "自動", "手IKの切替値を使う"), ("FK", "FK", ""), ("IK", "IK", "")],
    )
    planted: bpy.props.BoolProperty(name="接地", default=True)


for attr, (label, _) in {**LEG, **ARM}.items():
    BodyLimbMap.__annotations__[attr] = bpy.props.StringProperty(name=label)


class BodySettings(bpy.types.PropertyGroup):
    smart_ik: bpy.props.BoolProperty(name="スマートIK移動", default=True)
    legs: bpy.props.BoolProperty(name="脚 → 骨盤", default=True)
    arms: bpy.props.BoolProperty(name="腕 → 肩", default=True)
    torso: bpy.props.BoolProperty(name="背骨にも配分", default=True)
    pelvis: bpy.props.StringProperty(name="骨盤操作骨", default="c_root_master.x")
    preset: bpy.props.StringProperty(name="背骨チェーン")
    chest_bone: bpy.props.StringProperty(name="胸郭（単独で使用）")
    left: bpy.props.PointerProperty(type=BodyLimbMap)
    right: bpy.props.PointerProperty(type=BodyLimbMap)
    reach: bpy.props.FloatProperty(
        name="脚の長さの上限", subtype="FACTOR", min=0.80, max=1.0, default=0.96
    )
    drop: bpy.props.FloatProperty(name="最大下降 / 脚長", subtype="FACTOR", min=0, max=0.4, default=0.16)
    yaw: bpy.props.FloatProperty(
        name="踏み出しの腰回旋", subtype="ANGLE", min=0, max=radians(45), default=radians(18)
    )
    roll: bpy.props.FloatProperty(
        name="骨盤の最大傾き", subtype="ANGLE", min=0, max=radians(40), default=radians(18),
        description="脚が伸びる側へ骨盤を傾ける上限。0°で傾きの補助を解除します",
    )
    lift: bpy.props.FloatProperty(
        name="肩の最大補助", subtype="ANGLE", min=0, max=radians(50), default=radians(25)
    )
    chest: bpy.props.FloatProperty(name="胸の向きを保つ量", subtype="FACTOR", min=0, max=1, default=0.65)
    turn: bpy.props.FloatProperty(
        name="手の移動に伴う胸の回旋", subtype="ANGLE", min=0, max=radians(45), default=radians(22)
    )


def rotpath(b):
    return {"QUATERNION": "rotation_quaternion", "AXIS_ANGLE": "rotation_axis_angle"}.get(
        b.rotation_mode, "rotation_euler"
    )


def write_rot(b, q):
    q.normalize()
    if b.rotation_mode == "QUATERNION":
        q.make_compatible(b.rotation_quaternion)
        b.rotation_quaternion = q
    elif b.rotation_mode == "AXIS_ANGLE":
        axis, angle = q.to_axis_angle()
        b.rotation_axis_angle = (angle, *axis)
    else:
        b.rotation_euler = q.to_euler(b.rotation_mode, b.rotation_euler)


def rotate(b, delta):
    if delta.x**2 + delta.y**2 + delta.z**2 < 1e-12:
        return False
    local = b.matrix_basis.to_quaternion()
    space = b.matrix.to_quaternion() @ local.inverted()
    write_rot(b, space.inverted() @ delta @ space @ local)
    return True


def translate(b, delta):
    if delta.length_squared < 1e-14:
        return False
    space = b.matrix @ b.matrix_basis.inverted()
    b.location += space.to_3x3().inverted() @ delta
    return True


def align_goal(b, target):
    pos = translate(b, target.translation - b.head)
    rot = rotate(b, target.to_quaternion() @ b.matrix.to_quaternion().inverted())
    return pos or rot


def smooth(t):
    t = max(0.0, min(1.0, t))
    return t * t * (3 - 2 * t)


def flat(v, up):
    return v - up * v.dot(up)


def length_of(b):
    return (b.tail - b.head).length


def required_drop(hip, ankle, up, length, limit):
    v = hip - ankle
    horizontal = flat(v, up).length
    # Beyond horizontal reach, lowering can help only as far as ankle height.
    needed = max(0.0, v.dot(up) - sqrt(max(0.0, (limit * length) ** 2 - horizontal**2)))
    return needed * smooth(needed / (0.025 * length))


def extension_demand(distance, length, limit):
    """Ease into the correction as a knee approaches the reach limit."""
    band = 0.18 * length
    excess = max(0.0, distance - (limit * length - band))
    if excess < band:
        return excess * excess / (2 * band)
    return excess - 0.5 * band


def pelvic_roll(legs, supports, reference, lengths, pivot, up, limit, cap):
    if not any(supports) or not cap:
        return up, 0.0
    lateral = flat(legs[0]["hip"].head - legs[1]["hip"].head, up)
    width = lateral.length
    if width < 1e-6:
        return up, 0.0
    axis = up.cross(lateral).normalized()
    error = jacobian = 0.0
    for sign, leg, active, (hip0, ankle0), length in zip((1, -1), legs, supports, reference, lengths):
        if active:
            hip = leg["hip"].head
            span = hip - leg["ankle"].head
            error += sign * (extension_demand(span.length, length, limit)
                             - extension_demand((hip0 - ankle0).length, length, limit))
            # d(distance)/d(roll) for a rotation about the pelvis with a pinned foot.
            jacobian += sign * span.normalized().dot(axis.cross(hip - pivot))
    damping = 0.45 * width
    angle = -jacobian * error / (jacobian * jacobian + damping * damping)
    return axis, cap * tanh(angle / cap)


def arm_lift(upper, chest_up):
    axis = (upper.tail - upper.head).normalized()
    elevation = asin(max(-1.0, min(1.0, axis.dot(chest_up))))
    return smooth((elevation + radians(25)) / radians(110))


def channels(bones):
    return {(b.name, path): tuple(getattr(b, path)) for b in bones for path in ("location", rotpath(b))}


def different(a, b):
    return any(abs(x - y) > EPS for x, y in zip(a, b))


def quat(b, values):
    if b.rotation_mode == "QUATERNION":
        return Quaternion(values).normalized()
    if b.rotation_mode == "AXIS_ANGLE":
        return Quaternion(Vector(values[1:]), values[0]) if abs(values[0]) > EPS else IDENTITY.copy()
    e = b.rotation_euler.copy()
    e[:] = values
    return e.to_quaternion()


def composition(b, path, value, shown, raw):
    if not different(shown, raw):
        return value
    if path == "location":
        return tuple(Vector(value) - Vector(shown) + Vector(raw))
    q = quat(b, value) @ quat(b, shown).inverted() @ quat(b, raw)
    if path == "rotation_quaternion":
        q.make_compatible(Quaternion(raw))
        return tuple(q)
    if path == "rotation_axis_angle":
        axis, angle = q.to_axis_angle()
        return (angle, *axis)
    return tuple(q.to_euler(b.rotation_mode, b.rotation_euler))


class BodyPose:
    def __init__(self, c):
        self.rig = rig = c.object
        self.scene = c.scene
        self.layer = c.view_layer
        self.window = c.window
        self.settings = s = rig.body_pose_settings
        self.pelvis = self.bone(s.pelvis)
        presets = getattr(rig, "chain_motion_presets", None)
        if s.preset:
            group = presets.get(s.preset) if presets else None
            if not group:
                raise ValueError("指定された背骨チェーンが見つかりません")
            self.spine = [self.bone(m.name) for m in group.members if m.name != s.pelvis]
        else:
            self.spine = (
                [self.bone(s.chest_bone)]
                if s.chest_bone
                else sorted(
                    (b for b in rig.pose.bones if re.fullmatch(r"c_spine_\d+\.x", b.name)),
                    key=lambda b: int(re.search(r"\d+", b.name).group()),
                )
            )
        if not self.spine:
            raise ValueError("設定で背骨チェーンを選択してください")
        self.chest = self.spine[-1]
        self.arms, self.legs = [], []
        for side, mapping in ((".l", s.left), (".r", s.right)):
            for enabled, roles, output in ((s.legs, LEG, self.legs), (s.arms, ARM, self.arms)):
                if enabled:
                    output.append(
                        {
                            k: self.bone(getattr(mapping, k) or default + side)
                            for k, (_, default) in roles.items()
                        }
                    )
        if not self.legs and not self.arms:
            raise ValueError("脚または腕の補助を有効にしてください")
        controls = [self.pelvis, *self.spine]
        for leg in self.legs:
            controls.append(leg["foot"])
        for arm in self.arms:
            controls.extend(arm[k] for k in ("clav", "fk", "hand"))
        self.controls = list(dict.fromkeys(controls))
        for b in self.controls:
            if any(b.lock_rotation) or (b.lock_rotations_4d and b.lock_rotation_w):
                raise ValueError(f"{b.name}: 回転可能な操作骨を指定してください")
        for b in [self.pelvis] + [x["foot"] for x in self.legs] + [x["hand"] for x in self.arms]:
            if any(b.lock_location):
                raise ValueError(f"{b.name}: 移動可能な操作骨を指定してください")
        self.up = (rig.matrix_world.to_3x3().inverted() @ UP).normalized()
        self.chest_local_up = self.chest.bone.matrix_local.to_quaternion().inverted() @ UP
        self.capture()

    def bone(self, name):
        b = self.rig.pose.bones.get(name)
        if b is None:
            raise ValueError(f"リグ設定でボーンを指定してください ({name})")
        return b

    def signature(self):
        s = self.settings
        return (
            s.legs,
            s.arms,
            s.pelvis,
            s.preset,
            s.chest_bone,
            tuple(tuple(getattr(m, k) for k in (*LEG, *ARM)) for m in (s.left, s.right)),
        )

    def tuning(self):
        s = self.settings
        return (
            s.reach,
            s.drop,
            s.yaw,
            s.roll,
            s.lift,
            s.turn,
            s.chest,
            s.torso,
            s.left.planted,
            s.right.planted,
            s.left.mode,
            s.right.mode,
        )

    def measured_pose(self):
        points = [self.chest.matrix.to_quaternion()]
        for leg in self.legs:
            points.extend((leg["hip"].head, leg["ankle"].head, leg["foot"].head))
        for arm in self.arms:
            points.extend((arm["upper"].head, arm["upper"].tail, arm["hand"].head))
        return [tuple(v) for v in points]

    def capture(self):
        self.layer.update()
        self.frame = (self.scene.frame_current, self.scene.frame_subframe)
        ad = self.rig.animation_data
        self.action = (ad.action, ad.action_slot) if ad else (None, None)
        self.config = self.signature()
        self.knobs = self.tuning()
        self.switches = tuple(x["hand"].get("ik_fk_switch") for x in self.arms) + tuple(
            x["foot"].get("ik_fk_switch") for x in self.legs
        )
        self.modes = tuple((b.rotation_mode) for b in self.controls)
        self.initial = channels(self.controls)
        self.raw = self.initial.copy()
        self.shown = self.raw.copy()
        self.gesture = None
        self.touched = set()
        self.keyed = set()
        self.foot_space = [leg["foot"].matrix @ leg["foot"].matrix_basis.inverted() for leg in self.legs]
        self.start_foot = [leg["foot"].head.copy() for leg in self.legs]
        self.lengths = [length_of(l["hip"]) + length_of(l["shin"]) for l in self.legs]
        if any(v < 1e-6 for v in self.lengths):
            raise ValueError("脚の長さを計測できません")
        self.rest_legs = [(leg["hip"].head.copy(), leg["ankle"].head.copy()) for leg in self.legs]
        points = [l["hip"].head for l in self.legs] or [a["clav"].tail for a in self.arms]
        self.left = flat(points[0] - points[1], self.up).normalized()
        self.lift0 = [arm_lift(a["upper"], self.chest_up()) for a in self.arms]
        self.hand_start = [a["hand"].head.copy() for a in self.arms]
        self.arm_lengths = [max(length_of(a["upper"]) + (a["hand"].head - a["upper"].tail).length,
                                1e-5) for a in self.arms]
        self.headings = []
        for i, arm in enumerate(self.arms):
            radial = flat(arm["hand"].head - arm["upper"].head, self.up)
            outward = self.left if i == 0 else -self.left
            # A vertical arm has no azimuth. Blend toward its side of the body near the pole.
            weight = smooth(radial.length / (0.25 * self.arm_lengths[i]))
            self.headings.append(radial.normalized() * weight + outward * (1 - weight))
        self.last_amounts = (0.0, 0.0, 0.0, 0.0)
        self.measured = self.measured_pose()

    def chest_up(self):
        return (self.chest.matrix.to_quaternion() @ self.chest_local_up).normalized()

    def set_raw(self):
        for (name, path), values in self.raw.items():
            b = self.rig.pose.bones[name]
            if different(getattr(b, path), values):
                setattr(b, path, values)
        self.layer.update()

    def support(self, leg, mapping):
        return mapping.planted and leg["foot"].get("ik_fk_switch", 0) == 0

    def solve(self):
        self.set_raw()
        s = self.settings
        supports = [self.support(l, m) for l, m in zip(self.legs, (s.left, s.right))]
        foot_targets = [
            space @ leg["foot"].matrix_basis for space, leg in zip(self.foot_space, self.legs)
        ]
        hand_targets = [a["hand"].matrix.copy() for a in self.arms]
        arm_targets = [a["upper"].matrix.to_quaternion() for a in self.arms]
        ik = [
            m.mode == "IK" or (a["hand"].get("ik_fk_switch", 1) == 0 and
                               (m.mode == "AUTO" or (_workflow and _workflow.hand == a["hand"])))
            for a, m in zip(self.arms, (s.left, s.right))
        ]
        chest_q = self.chest.matrix.to_quaternion()
        lift = [
            (arm_lift(a["upper"], self.chest_up()) - base) * s.lift
            for a, base in zip(self.arms, self.lift0)
        ]
        turn = [0.0] * len(self.arms)
        for i, on in enumerate(ik):
            if on:
                travel = (hand_targets[i].translation - self.hand_start[i]) / self.arm_lengths[i]
                # Measure reach on the input goal. Measuring the IK upper arm here
                # feeds the transported pole back into the shoulder on the next tick.
                height = travel.dot(self.up)
                lift[i] = s.lift * tanh(2 * height)
                turn[i] = s.turn * tanh(2 * self.headings[i].cross(travel).dot(self.up))
        chest_turn = max(-s.turn, min(s.turn, sum(turn))) if s.torso else 0.0
        yaw = drop = 0.0
        for i, (leg, on) in enumerate(zip(self.legs, supports)):
            if not on:
                continue
            axis = self.left if i == 0 else -self.left
            stride = foot_targets[i].translation - self.start_foot[i]
            yaw += axis.cross(stride).dot(self.up) / self.lengths[i] * 2.5 * s.yaw
        yaw = max(-s.yaw, min(s.yaw, yaw))
        if rotate(self.pelvis, Quaternion(self.up, yaw)):
            self.layer.update()
        pins = [
            (leg["foot"], target) for leg, target, on in zip(self.legs, foot_targets, supports) if on
        ]
        self.pin(pins)
        chest_after_yaw = self.chest.matrix.to_quaternion()
        if rotate(self.pelvis, Quaternion(self.up, 0.20 * chest_turn)):
            self.layer.update()
            self.pin(pins)
        roll_axis, roll = pelvic_roll(
            self.legs, supports, self.rest_legs, self.lengths, self.pelvis.head, self.up, s.reach, s.roll
        )
        if rotate(self.pelvis, Quaternion(roll_axis, roll)):
            self.layer.update()
            self.pin(pins)
        for i, (leg, on) in enumerate(zip(self.legs, supports)):
            if on:
                initial_drop = required_drop(*self.rest_legs[i], self.up, self.lengths[i], s.reach)
                d = (
                    required_drop(leg["hip"].head, leg["ankle"].head, self.up, self.lengths[i], s.reach)
                    - initial_drop
                )
                drop = max(drop, min(d, s.drop * self.lengths[i]))
        if translate(self.pelvis, -drop * self.up):
            self.layer.update()
        self.pin(pins)
        # Compensate only the rotation actually inherited by the chest.
        inherited = self.chest.matrix.to_quaternion()
        side_bend = (lift[0] - lift[1]) * 0.30 if lift else 0.0
        if s.torso:
            tilt_axis = self.left.cross(self.up)
            if self.arms:
                line = self.arms[0]["clav"].tail - self.arms[1]["clav"].tail
                axis = line.cross(self.chest_up())
                if axis.length_squared > 1e-10:
                    tilt_axis = axis.normalized()
            # Measure inherited rotation only once. Keep the authored chest heading
            # and distribute an additional quarter of the hip roll along the spine.
            wanted = inherited.slerp(chest_q, s.chest)
            if abs(roll) > 1e-6 or abs(chest_turn) > 1e-7:
                wanted = Quaternion(roll_axis, 1.25 * roll) @ chest_after_yaw.slerp(chest_q, s.chest)
                wanted = Quaternion(self.up, chest_turn) @ wanted
            wanted = Quaternion(tilt_axis, side_bend) @ wanted
            share = IDENTITY.slerp(wanted @ inherited.inverted(), 1 / len(self.spine))
            for b in self.spine:
                if rotate(b, share):
                    self.layer.update()
        for i, arm in enumerate(self.arms):
            amount = 0.92 * lift[i] + 0.08 * lift[1 - i]
            clav = arm["clav"]
            axis = (clav.tail - clav.head).cross(self.chest_up()).normalized()
            raising = Quaternion(axis, amount) if axis.length_squared else IDENTITY
            if rotate(clav, Quaternion(self.up, 0.55 * turn[i] + 0.05 * turn[1-i]) @ raising):
                self.layer.update()
        self.pin(
            pins + [(a["hand"], target) for a, target, on in zip(self.arms, hand_targets, ik) if on]
        )
        for arm, wanted, on, amount in zip(self.arms, arm_targets, ik, lift):
            if not on and abs(amount) > 1e-7:
                # Preserve both swing and the animator's existing axial twist.
                for _ in range(3):
                    if not rotate(arm["fk"], wanted @ arm["upper"].matrix.to_quaternion().inverted()):
                        break
                    self.layer.update()
        self.shown = channels(self.controls)
        self.touched = {k for k in self.shown if different(self.raw[k], self.shown[k])}
        self.last_amounts = (drop, yaw, *(lift or [0.0, 0.0]))
        self.knobs = self.tuning()
        self.measured = self.measured_pose()

    def pin(self, pairs):
        changed = False
        for b, target in pairs:
            changed = align_goal(b, target) or changed
        if changed:
            self.layer.update()

    def update(self, modal):
        seen = channels(self.controls)
        changes = [k for k in seen if different(seen[k], self.shown[k])]
        if modal and self.gesture is None:
            self.gesture = (self.raw.copy(), self.shown.copy())
        geometry_changed = any(different(a, b) for a, b in zip(self.measured_pose(), self.measured))
        adjusted = self.knobs != self.tuning()
        if changes or geometry_changed or adjusted:
            before, display = self.gesture or (self.raw, self.shown)
            for k in changes:
                name, path = k
                self.raw[k] = composition(self.bone(name), path, seen[k], display[k], before[k])
            self.solve()
        if not modal and (changes or geometry_changed or adjusted or self.gesture):
            pending = self.gesture
            self.gesture = None
            if pending and all(not different(v, self.shown[k]) for k, v in pending[1].items()):
                return
            if self.scene.tool_settings.use_keyframe_insert_auto and self.frame[0] >= 0:
                if _workflow and _workflow.preview_frame == self.frame[0]:
                    hand = _workflow.hand
                    key = (hand.name, "location")
                    if different(self.initial[key], self.raw[key]) or different(
                        self.initial[hand.name, rotpath(hand)], self.raw[hand.name, rotpath(hand)]
                    ):
                        _workflow.key_preview()
                for name, path in self.keyed | self.touched:
                    b = self.bone(name)
                    with AutoKeying.options(
                        keytype=self.scene.tool_settings.keyframe_type,
                        use_loc=path == "location",
                        use_rot=path != "location",
                        use_scale=False,
                    ):
                        AutoKeying.autokey_transformation(bpy.context, b)
                    if path == "rotation_quaternion" and any(b == a["hand"] for a in self.arms):
                        arp_pose_assist.match_hand_key(b, self.scene)
                self.keyed = self.touched.copy()


def preview_hand_move(hand):
    pose = _session
    if (pose and pose.rig == hand.id_data and pose.frame == (pose.scene.frame_current, 0.0)
            and any(a["hand"] == hand for a in pose.arms)):
        pose.update(True)


def commit_hand_preview(hand):
    if (_workflow and _workflow.hand == hand and
            _workflow.preview_frame == _workflow.scene.frame_current):
        _workflow.key_preview()


class PoseWorkflow:
    """Own the temporary tool state across frame changes and solver rebases."""

    def __init__(self, c, pose):
        self.scene, self.rig, self.layer = c.scene, c.object, c.view_layer
        self.auto = self.scene.tool_settings.use_keyframe_insert_auto
        self.hand = None
        self.start_mode = None
        self.preview_controls = []
        self.action = pose.action
        self.last_frame = self.scene.frame_current
        self.preview_frame = None
        active = c.active_pose_bone
        if not active or not pose.arms:
            return
        try:
            arp, limb, side, goal = arp_pose_assist.selected_limb(c)
        except (ValueError, ImportError):
            return
        if limb == "arm" and active.name in (goal.name, arp.fk_arm[2] + side) and any(
            a["hand"] == goal for a in pose.arms
        ):
            self.hand, self.arp, self.side = goal, arp, side
            self.start_mode = goal["ik_fk_switch"]

    def start(self, c):
        if self.hand and self.start_mode != 0:
            arp_pose_assist.switch_limb(c, self.arp, "arm", self.side, self.hand)
            ad = self.rig.animation_data
            self.action = (ad.action, ad.action_slot)
        self.scene.tool_settings.use_keyframe_insert_auto = True

    def preview(self):
        self.last_frame = self.scene.frame_current
        if self.hand is None or self.hand["ik_fk_switch"] == 0:
            return
        if self.hand["ik_fk_switch"] != 1:
            raise ValueError("腕のIK/FKがブレンド中です")
        # An existing FK key can be visited without editing the Action.
        settings = self.scene.tool_settings
        auto, mirror = settings.use_keyframe_insert_auto, self.rig.pose.use_mirror_x
        controls = [self.rig.pose.bones.get(base + self.side) for base in self.arp.ik_arm]
        before = [(b, b.matrix_basis.copy()) for b in controls if b]
        stretch = self.hand.get("stretch_length")
        try:
            settings.use_keyframe_insert_auto = False
            self.rig.pose.use_mirror_x = False
            self.arp.ik_to_fk_arm(self.rig, self.side, add_keyframe=False)
            self.preview_frame = self.last_frame
            self.preview_controls = [(b, matrix) for b, matrix in before if b.matrix_basis != matrix]
            self.preview_stretch = stretch
        finally:
            settings.use_keyframe_insert_auto = auto
            self.rig.pose.use_mirror_x = mirror

    def discard_preview(self):
        if self.preview_frame is None:
            return
        for bone, matrix in self.preview_controls:
            bone.matrix_basis = matrix
        if self.preview_stretch is not None:
            self.hand["stretch_length"] = self.preview_stretch
        self.hand["ik_fk_switch"] = 1.0
        self.preview_controls = []
        self.preview_frame = None
        self.rig.update_tag()

    def key_preview(self):
        hand = self.hand
        hand.keyframe_insert(data_path=arp_pose_assist.SWITCH)
        curve = arp_pose_assist.switch_curve(self.rig, hand)
        points = [p for p in curve.keyframe_points if p.co.x <= self.preview_frame]
        for point in points[-2:]:
            point.interpolation = "CONSTANT"
        curve.update()
        for base in self.arp.ik_arm[2:4] + self.arp.ik_arm[5:7]:
            bone = self.rig.pose.bones.get(base + self.side)
            if bone:
                for path in ("location", rotpath(bone), "scale"):
                    bone.keyframe_insert(data_path=path)
        for prop in ("stretch_length", "auto_stretch"):
            hand.keyframe_insert(data_path=f'["{prop}"]')
        twist = self.rig.pose.bones.get("c_arm_ik" + self.side)
        if twist:
            twist.keyframe_insert(data_path="rotation_euler", index=1)
        arp_pose_assist.match_hand_key(hand, self.scene)
        self.preview_controls = []
        self.preview_frame = None

    def finish(self):
        try:
            if self.hand is None:
                return
            if self.start_mode == 0:
                if self.preview_frame == self.scene.frame_current and self.rig.animation_data and (
                    self.rig.animation_data.action, self.rig.animation_data.action_slot
                ) == self.action:
                    self.discard_preview()
                    self.layer.update()
                return
            ad = self.rig.animation_data
            current = (ad.action, ad.action_slot)
            scene = self.scene
            frame, sub = scene.frame_current, scene.frame_subframe
            revisit = current != self.action or bool(sub)
            try:
                if current != self.action:
                    ad.action, ad.action_slot = self.action
                if revisit:
                    scene.frame_set(self.last_frame)
                if self.hand["ik_fk_switch"] == 0:
                    with bpy.context.temp_override(scene=scene, view_layer=self.layer,
                                                   object=self.rig, active_object=self.rig):
                        arp_pose_assist.switch_limb(bpy.context, self.arp, "arm", self.side, self.hand)
            finally:
                if current != self.action:
                    ad.action = current[0]
                    if current[1]:
                        ad.action_slot = current[1]
                if revisit:
                    scene.frame_set(frame, subframe=sub)
        finally:
            self.scene.tool_settings.use_keyframe_insert_auto = self.auto


def stop(*, from_timer=False):
    global _session, _workflow
    _session = None
    workflow, _workflow = _workflow, None
    if not from_timer and bpy.app.timers.is_registered(tick):
        bpy.app.timers.unregister(tick)
    if workflow:
        workflow.finish()


def resume_after_pivot(context, rig):
    global _session
    if _session and _session.rig == rig and _session.scene == context.scene:
        if context.object != rig or context.mode != "POSE":
            stop()
            return
        try:
            _session = BodyPose(context)
        except (ReferenceError, RuntimeError, ValueError, KeyError) as error:
            context.window_manager.body_pose_message = str(error)
            stop()


def tick():
    global _session
    c = bpy.context
    session = _session
    if session is None:
        return None
    try:
        w = session.window
        if (
            w.scene != session.scene
            or w.view_layer.objects.active != session.rig
            or session.rig.mode != "POSE"
        ):
            stop(from_timer=True)
            return None
        if pivot_rotate.is_active(session.scene, session.rig):
            return 0.05
        if w.screen.is_animation_playing or session.scene.frame_subframe:
            session.frame = None
            session.gesture = None
            return 0.1
        with c.temp_override(window=w):
            ad = session.rig.animation_data
            action = (ad.action, ad.action_slot) if ad else (None, None)
            if _workflow.action[0] is None or (_workflow.action[0] == action[0] and _workflow.action[1] is None):
                _workflow.action = action
            if _workflow.hand and action != _workflow.action:
                stop(from_timer=True)
                return None
            if session.action[0] is None or (
                session.action[0] == action[0] and session.action[1] is None
            ):
                # Native Auto Key may create the Action/slot at mouse release.
                session.action = action
            switches = tuple(x["hand"].get("ik_fk_switch") for x in session.arms) + tuple(
                x["foot"].get("ik_fk_switch") for x in session.legs
            )
            if (
                session.frame != (session.scene.frame_current, 0.0)
                or action != session.action
                or session.modes != tuple(b.rotation_mode for b in session.controls)
                or switches != session.switches
            ):
                _workflow.preview()
                _session = BodyPose(c)
            elif session.config != session.signature():
                # Recompute the same authored pose with the new set of roles.
                # Establish the new reference before restoring the current input.
                for (name, path), v in session.initial.items():
                    setattr(session.bone(name), path, v)
                session.layer.update()
                try:
                    replacement = BodyPose(c)
                finally:
                    session.set_raw()
                replacement.raw = channels(replacement.controls)
                replacement.solve()
                _session = replacement
            else:
                modal = any(
                    op.bl_idname.startswith("TRANSFORM_OT_")
                    or op.bl_idname
                    in {
                        "GIZMOGROUP_OT_gizmo_tweak",
                        "POSE_OT_arp_assist_translate",
                        "POSE_OT_arp_assist_move_wrist",
                    }
                    for op in w.modal_operators
                )
                session.update(modal)
    except (ReferenceError, RuntimeError, ValueError, KeyError) as error:
        stop(from_timer=True)
        if hasattr(c.window_manager, "body_pose_message"):
            c.window_manager.body_pose_message = str(error)
        return None
    return 1 / 40


@persistent
def before_frame(scene, *args):
    s = _session
    if s and scene == s.scene and not pivot_rotate.is_active(scene, s.rig) and s.frame != (scene.frame_current, scene.frame_subframe):
        # Discard unkeyed offsets before Blender evaluates another animation pose.
        for name, path in s.touched:
            b = s.rig.pose.bones.get(name)
            if b and not different(getattr(b, path), s.shown[name, path]):
                setattr(b, path, s.raw[name, path])
        s.frame = None
        s.gesture = None
        s.touched.clear()
        _workflow.discard_preview()


@persistent
def after_frame(scene, *args):
    global _session
    s = _session
    c = bpy.context
    if (
        s
        and s.frame is None
        and s.scene == scene
        and not pivot_rotate.is_active(scene, s.rig)
        and not scene.frame_subframe
        and s.rig == c.object
        and not (c.screen and c.screen.is_animation_playing)
    ):
        try:
            # Native snap uses transform operators. Defer it to the main timer.
            if not (_workflow.hand and _workflow.hand["ik_fk_switch"] != 0):
                _session = BodyPose(c)
                _workflow.last_frame = scene.frame_current
                _workflow.preview_frame = None
        except (ReferenceError, RuntimeError, ValueError, KeyError):
            stop()


@persistent
def reset(*args):
    stop()


class POSE_OT_body_assist(bpy.types.Operator):
    bl_idname = "pose.body_pose_assist"
    bl_label = "PoseAssist"
    bl_description = "現在の姿勢を基準に、通常の移動・回転に合わせて体の補正を開始・終了します"
    bl_options = {"UNDO"}

    @staticmethod
    def smart_settings(context):
        if _session and _session.rig == context.object and _session.window == context.window:
            return _session.settings
        return None

    @classmethod
    def poll(cls, c):
        return c.mode == "POSE" and c.object is not None

    def execute(self, c):
        global _session, _workflow
        c.window_manager.body_pose_message = ""
        if _session:
            if (_session.frame == (c.scene.frame_current, 0.0) and
                    not pivot_rotate.is_active(c.scene, _session.rig)):
                _session.update(False)
            stop()
        else:
            try:
                if c.screen and c.screen.is_animation_playing or c.scene.frame_subframe:
                    raise ValueError("再生を止め、整数フレームで使用してください")
                pose = BodyPose(c)
                _workflow = PoseWorkflow(c, pose)
                _workflow.start(c)
                _session = BodyPose(c) if _workflow.hand else pose
                bpy.app.timers.register(tick, first_interval=0.01)
            except (ValueError, RuntimeError) as error:
                stop()
                self.report({"WARNING"}, str(error))
                return {"CANCELLED"}
        return {"FINISHED"}


class VIEW3D_PT_body_pose_assist(bpy.types.Panel):
    bl_space_type = "VIEW_3D"
    bl_region_type = "UI"
    bl_category = "BAA"
    bl_label = "PoseAssist"
    bl_parent_id = "VIEW3D_PT_baa_root"
    bl_order = 10

    @classmethod
    def poll(cls, c):
        return c.mode == "POSE" and c.object and c.object.type == "ARMATURE"

    def draw(self, c):
        s = c.object.body_pose_settings
        ui = self.layout
        active = bool(_session and _session.rig == c.object)
        ui.operator(
            "pose.body_pose_assist",
            text="PoseAssist を終了" if active else "PoseAssist を開始", icon="CHECKBOX_HLT" if active else "CHECKBOX_DEHLT",
            depress=active,
        )
        if c.window_manager.body_pose_message:
            ui.label(text=c.window_manager.body_pose_message, icon="ERROR")
        if active:
            target = _workflow.hand if _workflow else None
            mode = ("IK" if target["ik_fk_switch"] == 0 else "FK") if target else ""
            auto = "ON" if c.scene.tool_settings.use_keyframe_insert_auto else "OFF"
            ui.label(text=f"{target.name + ' · ' + mode + '  /  ' if target else ''}自動キー {auto}")
            ui.prop(s, "smart_ik")
        else:
            ui.prop(c.window_manager, "arp_assist_follow_g", toggle=True)
        if not active or s.smart_ik:
            row = ui.row(align=True)
            row.prop(c.window_manager, "arp_assist_keep_angle")
            row.prop(c.window_manager, "arp_assist_follow_pole")
        if not active:
            row = ui.row()
            row.operator_context = "INVOKE_REGION_WIN"
            row.operator("pose.arp_assist_move_wrist", icon="ORIENTATION_LOCAL")
        ui.operator("pose.arp_assist_keyed_switch", icon="KEY_HLT")
        ui.operator("pose.arp_assist_mirror_keyed")
        from . import pose_borrow
        pose_borrow.draw(ui, c)
        header, settings = ui.panel("baa_assist_settings", default_closed=True)
        header.label(text="PoseAssist 設定")
        if not settings:
            return
        ui = settings
        if not active:
            ui.prop(s, "smart_ik", text="PoseAssist 中のスマートIK移動")
        row = ui.row(align=True)
        row.prop(s, "legs")
        row.prop(s, "arms")
        if s.legs:
            row = ui.row(align=True)
            row.label(text="接地するIK足")
            row.prop(s.left, "planted", text="左")
            row.prop(s.right, "planted", text="右")
        ui.prop(s, "torso")
        head, box = ui.panel("body_pose_tuning", default_closed=True)
        head.label(text="補助量")
        if box:
            for key in ("reach", "drop", "yaw", "roll", "lift", "turn", "chest"):
                box.prop(s, key)
        head, box = ui.panel("body_pose_rig_map", default_closed=True)
        head.label(text="リグ設定")
        if box:
            box.prop_search(s, "pelvis", c.object.pose, "bones")
            if hasattr(c.object, "chain_motion_presets"):
                box.prop_search(s, "preset", c.object, "chain_motion_presets")
            else:
                box.prop(s, "preset")
            if not s.preset:
                box.prop_search(s, "chest_bone", c.object.pose, "bones")
            for side, label, m in ((".l", "左側", s.left), (".r", "右側", s.right)):
                h, part = box.panel("body_pose_mapping" + side, default_closed=True)
                h.label(text=label)
                if part:
                    part.prop(m, "mode")
                    for key, (title, default) in {**LEG, **ARM}.items():
                        part.prop_search(m, key, c.object.pose, "bones", text=title)
                        if not getattr(m, key):
                            part.label(text="  " + default + side)


classes = (BodyLimbMap, BodySettings, POSE_OT_body_assist, VIEW3D_PT_body_pose_assist)
hooks = (
    (bpy.app.handlers.frame_change_pre, before_frame),
    (bpy.app.handlers.frame_change_post, after_frame),
    (bpy.app.handlers.load_pre, reset),
    (bpy.app.handlers.undo_pre, reset),
    (bpy.app.handlers.redo_pre, reset),
)


def register():
    for cls in classes:
        bpy.utils.register_class(cls)
    bpy.types.Object.body_pose_settings = bpy.props.PointerProperty(type=BodySettings)
    bpy.types.WindowManager.body_pose_message = bpy.props.StringProperty(options={"SKIP_SAVE"})
    for handler, fn in hooks:
        handler.append(fn)


def unregister():
    stop()
    for handler, fn in hooks:
        handler.remove(fn)
    del bpy.types.WindowManager.body_pose_message
    del bpy.types.Object.body_pose_settings
    for cls in reversed(classes):
        bpy.utils.unregister_class(cls)


if __name__ == "__main__":
    register()
