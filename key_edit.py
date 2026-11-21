"""Rollback only the animation channels touched by one BAA operation."""
from contextlib import contextmanager

import bpy
from bpy_extras.anim_utils import action_get_channelbag_for_slot


def channelbag(rig):
    ad = rig.animation_data
    if ad and ad.action and ad.action_slot:
        return action_get_channelbag_for_slot(ad.action, ad.action_slot)
    return None


_FIELDS = ("type", "interpolation", "easing", "amplitude", "back", "period",
           "handle_left_type", "handle_right_type", "select_control_point",
           "select_left_handle", "select_right_handle")


def key_state(p):
    return (tuple(p.co), tuple(p.handle_left), tuple(p.handle_right),
            tuple(getattr(p, k) for k in _FIELDS))


@contextmanager
def key_transaction(rig, paths):
    ad = rig.animation_data
    action = ad.action if ad else None
    slot = ad.action_slot if ad else None
    bag = channelbag(rig)
    saved = {(fc.data_path, fc.array_index): [key_state(p) for p in fc.keyframe_points]
             for fc in bag.fcurves if fc.data_path in paths} if bag else {}
    groups = {g.name for g in bag.groups} if bag else set()
    try:
        yield
    except Exception:
        current = channelbag(rig)
        if current:
            for fc in list(current.fcurves):
                if fc.data_path not in paths:
                    continue
                points = saved.get((fc.data_path, fc.array_index))
                if points is None:
                    current.fcurves.remove(fc)
                    continue
                if points == [key_state(p) for p in fc.keyframe_points]:
                    continue
                fc.keyframe_points.clear()
                fc.keyframe_points.add(len(points))
                for p, (co, left, right, attrs) in zip(fc.keyframe_points, points):
                    p.co = co
                    for k, value in zip(_FIELDS, attrs):
                        setattr(p, k, value)
                    p.handle_left, p.handle_right = left, right
                fc.update()
            for g in list(current.groups):
                if g.name not in groups and not len(g.channels):
                    current.groups.remove(g)
        ad = rig.animation_data
        if ad and not action and ad.action:
            created = ad.action
            ad.action = None
            if not created.users:
                bpy.data.actions.remove(created)
        elif ad and action and ad.action_slot != slot:
            created_slot = ad.action_slot
            ad.action_slot = slot
            if created_slot and not created_slot.users():
                action.slots.remove(created_slot)
        raise
