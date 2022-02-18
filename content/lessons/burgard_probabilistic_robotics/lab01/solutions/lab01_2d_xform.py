import numpy as np


def pose_to_transform_matrix_3x3(
        robot_pose_1x3: np.ndarray) -> np.ndarray:

    theta = robot_pose_1x3[0, 2]
    c, s = np.cos(theta), np.sin(theta)

    t_matrix_3x3 = np.array(
        [[c, -1.0 * s, robot_pose_1x3[0, 0]],
         [s, c, robot_pose_1x3[0, 1]],
         [0, 0, 1]])
    return t_matrix_3x3


def robot_to_global_frame(
        robot_pose_1x3: np.ndarray,
        l_wrt_robot_frame_1x2: np.ndarray) -> np.ndarray:

    t_matrix_3x3 = pose_to_transform_matrix_3x3(
        robot_pose_1x3)

    l_wrt_robot_frame_homogenous_3x1 = np.append(
        l_wrt_robot_frame_1x2, [1]).reshape(
        (3, 1))

    l_wrt_global_frame_homogenous_3x1 = np.matmul(
        t_matrix_3x3,
        l_wrt_robot_frame_homogenous_3x1)

    l_wrt_global_frame_2x1 = (
        l_wrt_global_frame_homogenous_3x1 /
        l_wrt_global_frame_homogenous_3x1[2, 0])[
        0: 2, :]
    return np.transpose(l_wrt_global_frame_2x1)


def global_to_robot_frame(
        robot_pose_1x3: np.ndarray,
        l_wrt_global_frame_1x2: np.ndarray) -> np.ndarray:

    t_matrix_3x3 = pose_to_transform_matrix_3x3(
        robot_pose_1x3)

    l_wrt_robot_frame_homogenous_3x1 = np.append(
        l_wrt_global_frame_1x2, [1]).reshape(
        (3, 1))

    l_wrt_robot_frame_homogenous_3x1 = np.matmul(
        np.linalg.inv(t_matrix_3x3),
        l_wrt_robot_frame_homogenous_3x1)

    l_wrt_robot_frame_2x1 = (
        l_wrt_robot_frame_homogenous_3x1 /
        l_wrt_robot_frame_homogenous_3x1[2, 0])[
        0: 2, :]
    return np.transpose(l_wrt_robot_frame_2x1)


def xform_1_2(
        robot_pose_1_1x3: np.ndarray,
        robot_pose_2_1x3: np.ndarray) -> np.ndarray:

    t_matrix_1_3x3 = pose_to_transform_matrix_3x3(
        robot_pose_1_1x3)
    t_matrix_2_3x3 = pose_to_transform_matrix_3x3(
        robot_pose_2_1x3)
    t_1_2_matrix_3x3 = np.matmul(
        np.linalg.inv(t_matrix_1_3x3),
        t_matrix_2_3x3)
    return t_1_2_matrix_3x3


def local_frame_xform(
        robot_pose_1_1x3: np.ndarray,
        l_wrt_robot_frame_1_1x2: np.ndarray,
        robot_pose_2_1x3: np.ndarray) -> np.ndarray:
    t_1_2 = xform_1_2(
        robot_pose_1_1x3, robot_pose_2_1x3)

    l_wrt_robot_frame_1_homogenous_3x1 = np.append(
        l_wrt_robot_frame_1_1x2, [1]).reshape(
        (3, 1))

    l_wrt_robot_frame_2_homogenous_3x1 = np.matmul(
        t_1_2, l_wrt_robot_frame_1_homogenous_3x1)

    l_wrt_robot_frame_2_2x1 = (
        l_wrt_robot_frame_2_homogenous_3x1 /
        l_wrt_robot_frame_2_homogenous_3x1[2, 0])[
        0: 2, :]
    return np.transpose(l_wrt_robot_frame_2_2x1)


def main():
    rp = np.array([[1, 2, np.pi * 0.25]])
    l_wrt_robot_frame = np.array([[1, 0]])
    l_wrt_global_frame = robot_to_global_frame(
        rp, l_wrt_robot_frame)
    assert(l_wrt_global_frame.shape == (1, 2))
    print(
        f"Landmark L in the global frame: {np.around(l_wrt_global_frame, 5)}")

    l_wrt_robot_frame = global_to_robot_frame(
        rp, l_wrt_global_frame)
    print(
        "Landmark L transformed back in the local frame: "
        f"{np.around(l_wrt_robot_frame, 5)}")

    rp_2 = rp
    t_1_2 = xform_1_2(rp, rp_2)
    print(
        f"Tansform matrix T_1_2:\n{np.around(t_1_2, 5)}")

    l_wrt_robot_frame_2 = local_frame_xform(
        rp, l_wrt_robot_frame, rp_2)
    print(
        f"Landmark L in robot pose 2 frame: {np.around(l_wrt_robot_frame_2)}")


if __name__ == "__main__":
    main()
