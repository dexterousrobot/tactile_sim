import pybullet as p
from tactile_sim.utils.transforms import inv_transform_eul


def draw_link_frame(parent_id, link_id, lifetime=0.1):
    p.addUserDebugLine(
        [0, 0, 0],
        [0.1, 0, 0],
        [1, 0, 0],
        parentObjectUniqueId=parent_id,
        parentLinkIndex=link_id,
        lifeTime=lifetime,
    )
    p.addUserDebugLine(
        [0, 0, 0],
        [0, 0.1, 0],
        [0, 1, 0],
        parentObjectUniqueId=parent_id,
        parentLinkIndex=link_id,
        lifeTime=lifetime,
    )
    p.addUserDebugLine(
        [0, 0, 0],
        [0, 0, 0.1],
        [0, 0, 1],
        parentObjectUniqueId=parent_id,
        parentLinkIndex=link_id,
        lifeTime=lifetime,
    )


def draw_frame(frame, lifetime=0.1):
    # self.workframe_to_worldframe([0.1, 0, 0], rpy)[0],

    pos = frame[:3]
    p.addUserDebugLine(
            pos,
            inv_transform_eul([0.1, 0, 0, 0, 0, 0], frame)[:3],
            [1, 0, 0],
            lifeTime=lifetime,
        )
    p.addUserDebugLine(
            pos,
            inv_transform_eul([0, 0.1, 0, 0, 0, 0], frame)[:3],
            [0, 1, 0],
            lifeTime=lifetime,
        )
    p.addUserDebugLine(
            pos,
            inv_transform_eul([0, 0, 0.1, 0, 0, 0], frame)[:3],
            [0, 0, 1],
            lifeTime=lifetime,
        )


def draw_box(workframe, lims):

    p1 = [
        workframe[0] + lims[0, 0],
        workframe[1] + lims[1, 1],
        workframe[2],
    ]
    p2 = [
        workframe[0] + lims[0, 0],
        workframe[1] + lims[1, 0],
        workframe[2],
    ]
    p3 = [
        workframe[0] + lims[0, 1],
        workframe[1] + lims[1, 0],
        workframe[2],
    ]
    p4 = [
        workframe[0] + lims[0, 1],
        workframe[1] + lims[1, 1],
        workframe[2],
    ]

    p.addUserDebugLine(p1, p2, [1, 0, 0])
    p.addUserDebugLine(p2, p3, [1, 0, 0])
    p.addUserDebugLine(p3, p4, [1, 0, 0])
    p.addUserDebugLine(p4, p1, [1, 0, 0])
