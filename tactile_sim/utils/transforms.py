import pybullet as pb
import numpy as np


def euler2quat(pose_e):
    """Converts an euler rotation pose to a quaternion rotation pose."""
    pose_e = np.array(pose_e, dtype=np.float64).ravel()
    assert pose_e.size == 6, "Invalid euler pose"
    rot_q = pb.getQuaternionFromEuler(pose_e[3:])
    pose_q = np.concatenate((pose_e[:3], rot_q))
    return pose_q


def quat2euler(pose_q):
    """Converts an euler rotation pose to a quaternion rotation pose."""
    pose_q = np.array(pose_q, dtype=np.float64).ravel()
    assert pose_q.size == 7, "Invalid quaternion pose"
    rot_e_rad = pb.getEulerFromQuaternion(pose_q[3:])
    pose_e = np.concatenate((pose_q[:3], rot_e_rad))
    return pose_e


def transform_eul(pose_a, frame_b_a):
    """Transforms a Euler pose between reference frames.
    Transforms a pose in reference frame A to a pose in reference frame
    B (B is expressed relative to reference frame A).
    """
    pose_a_q = euler2quat(pose_a)
    frame_b_a_q = euler2quat(frame_b_a)
    pose_b_q = transform_quat(pose_a_q, frame_b_a_q)
    pose_b = quat2euler(pose_b_q)
    return pose_b


def transform_quat(pose_a, frame_b_a):
    """Transforms a quaternion pose between reference frames.
    Transforms a pose in reference frame A to a pose in reference frame
    B (B is expressed relative to reference frame A).
    """

    inv_frame_b_a_pos, inv_frame_b_a_rot = pb.invertTransform(
        frame_b_a[:3], frame_b_a[3:],
    )
    pos_b, rot_b = pb.multiplyTransforms(
        inv_frame_b_a_pos, inv_frame_b_a_rot,
        pose_a[:3], pose_a[3:]
    )

    return np.concatenate((pos_b, rot_b))


def inv_transform_eul(pose_b, frame_b_a):
    """Inverse transforms a Euler pose between reference frames.
    Transforms a pose in reference frame A to a pose in reference frame
    B (B is expressed relative to reference frame A).
    """
    pose_b_q = euler2quat(pose_b)
    frame_b_a_q = euler2quat(frame_b_a)
    pose_a_q = inv_transform_quat(pose_b_q, frame_b_a_q)
    pose_a = quat2euler(pose_a_q)
    return pose_a


def inv_transform_quat(pose_b, frame_b_a):
    """Inverse transforms a quaternion pose between reference frames.
    Transforms a pose in reference frame B to a pose in reference frame
    A (B is expressed relative to reference frame A).
    """
    pos_a, rot_a = pb.multiplyTransforms(
        frame_b_a[:3], frame_b_a[3:],
        pose_b[:3], pose_b[3:]
    )

    return np.concatenate((pos_a, rot_a))


def transform_vec_eul(vec_a, frame_b_a):
    """Transforms a vector between reference frames.
    Transforms a vector in reference frame A to a vector in reference frame
    B (B is expressed relative to reference frame A).
    """
    frame_b_a_q = euler2quat(frame_b_a)
    vec_b = transform_vec_quat(vec_a, frame_b_a_q)
    return vec_b


def transform_vec_quat(vec_a, frame_b_a):
    """Transforms a vector between reference frames.
    Transforms a vector in reference frame A to a vector in reference frame
    B (B is expressed relative to reference frame A).
    """

    vec_a = np.array(vec_a)
    inv_frame_b_a_pos, inv_frame_b_a_rot = pb.invertTransform(
        frame_b_a[:3], frame_b_a[3:],
    )
    rot_matrix = np.array(pb.getMatrixFromQuaternion(inv_frame_b_a_rot)).reshape(3, 3)
    vec_b = rot_matrix.dot(vec_a)

    return np.array(vec_b)


def inv_transform_vec_eul(vec_b, frame_b_a):
    """Transforms a vector between reference frames.
    Transforms a vector in reference frame B to a vector in reference frame
    A (B is expressed relative to reference frame A).
    """
    frame_b_a_q = euler2quat(frame_b_a)
    vec_a = inv_transform_vec_quat(vec_b, frame_b_a_q)
    return vec_a


def inv_transform_vec_quat(vec_b, frame_b_a):
    """Transforms a vector between reference frames.
    Transforms a vector in reference frame B to a vector in reference frame
    A (B is expressed relative to reference frame A).
    """

    vec_b = np.array(vec_b)
    rot_matrix = np.array(pb.getMatrixFromQuaternion(frame_b_a[3:])).reshape(3, 3)
    vec_a = rot_matrix.dot(vec_b)

    return np.array(vec_a)


if __name__ == "__main__":

    pose_a = np.array([0.1, 0.1, 0.0, 0.0, np.pi/4, 0.0])
    frame_b_a = np.array([1.0, -1.0, 0.0, np.pi/3, np.pi/2, np.pi/2])

    pose_b = transform_eul(pose_a, frame_b_a)
    new_pose_a = inv_transform_eul(pose_b, frame_b_a)

    float_formatter = "{:.6f}".format
    np.set_printoptions(formatter={"float_kind": float_formatter})

    print("")
    print("Init Pose:        {}".format(pose_a))
    print("Transformed Pose: {}".format(pose_b))
    print("Returned Pose:    {}".format(new_pose_a))
    print("Equal: {}".format(np.allclose(pose_a, new_pose_a, atol=1e-6)))

    vec_a = np.array([0.1, 0.17536, 0.0])
    frame_b_a = np.array([1.0, -1.0, 0.0, -np.pi/3, np.pi/7, np.pi/2])

    vec_b = transform_vec_eul(vec_a, frame_b_a)
    new_vec_a = inv_transform_vec_eul(vec_b, frame_b_a)

    float_formatter = "{:.6f}".format
    np.set_printoptions(formatter={"float_kind": float_formatter})

    print("")
    print("Init Vec:        {}".format(vec_a))
    print("Transformed Vec: {}".format(vec_b))
    print("Returned Vec:    {}".format(new_vec_a))
    print("Equal: {}".format(np.allclose(vec_a, new_vec_a, atol=1e-6)))
