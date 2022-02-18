import numpy as np


def robot_to_global_frame(
        robot_pose_1x3: np.ndarray,
        l_wrt_robot_frame_1x2: np.ndarray) -> np.ndarray:
    # Replace next line, and implement exercise
    l_wrt_global_frame_2x1 = np.zeros((2, 1))

    return np.transpose(l_wrt_global_frame_2x1)


def global_to_robot_frame(
        robot_pose_1x3: np.ndarray,
        l_wrt_global_frame_1x2: np.ndarray) -> np.ndarray:
    # Replace next line, and implement exercise
    l_wrt_robot_frame_2x1 = np.zeros((2, 1))

    return np.transpose(l_wrt_robot_frame_2x1)


def xform_1_2(
        robot_pose_1_1x3: np.ndarray,
        robot_pose_2_1x3: np.ndarray) -> np.ndarray:
    # Replace next line, and implement exercise
    t_1_2_matrix_3x3 = np.zeros((3, 3))

    return t_1_2_matrix_3x3


def local_frame_xform(
        robot_pose_1_1x3: np.ndarray,
        l_wrt_robot_frame_1_1x2: np.ndarray,
        robot_pose_2_1x3: np.ndarray) -> np.ndarray:
    # Replace next line, and implement exercise
    l_wrt_robot_frame_2_2x1 = np.zeros((2, 1))

    return np.transpose(l_wrt_robot_frame_2_2x1)


def main():
    # Some test values
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
