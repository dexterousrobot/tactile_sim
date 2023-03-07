import pybullet as p


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
