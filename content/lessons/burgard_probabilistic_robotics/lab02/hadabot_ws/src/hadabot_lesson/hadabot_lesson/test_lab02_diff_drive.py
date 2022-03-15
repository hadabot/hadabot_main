import numpy as np
from solution_lab02_diff_drive import diffdrive as solution_diffdrive
from lab02_diff_drive import diffdrive


class DriveCommand:
    def __init__(self, v_l, v_r, t):
        self.v_l = v_l
        self.v_r = v_r
        self.t = t

    def __str__(self):
        return (
            "Drive Command (v_l, v_r, t): "
            f"({self.v_l}, {self.v_r}, {self.t})")


def main():
    wb = 0.14  # wheelbase is 14cm
    x_0 = 1.5
    y_0 = 2.0
    theta_0 = np.pi / 2.0

    drive_cmds = [
        DriveCommand(0.3, 0.3, 3),
        DriveCommand(0.1, -0.1, 1),
        DriveCommand(0.2, 0.0, 2)]

    poses = [(x_0, y_0, theta_0)]

    for idx, drive_cmd in enumerate(drive_cmds):
        poses.append(diffdrive(
            poses[idx][0], poses[idx][1], poses[idx][2],
            drive_cmd.v_l, drive_cmd.v_r, drive_cmd.t, wb))

    solution_poses = [(x_0, y_0, theta_0)]

    for idx, drive_cmd in enumerate(drive_cmds):
        solution_poses.append(
            solution_diffdrive(
                solution_poses[idx][0],
                solution_poses[idx][1],
                solution_poses[idx][2],
                drive_cmd.v_l, drive_cmd.v_r, drive_cmd.t, wb))

    print(
        "Upon executing the 3 drive commands: "
        f"{[str(dc) for dc in drive_cmds]}")

    print(
        "Your fwd kinematics yielded the following poses: "
        f"{str(poses[1:])}")

    print(
        "Compared against the solution poses: "
        f"{str(solution_poses[1:])}")

    print(
        "Do your poses match the solutions?: "
        f"{np.allclose(np.array(poses), np.array(solution_poses))}")


if __name__ == "__main__":
    main()
