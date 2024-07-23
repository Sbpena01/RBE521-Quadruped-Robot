#!/usr/bin/env python3
# license removed for brevity
import rospy
import numpy as np
import time
import matplotlib.pyplot as plt
from std_msgs.msg import Float64MultiArray

# hw == hybrid walk
# _t == trot 
# _gt == gaitTransition


def quintic_solve(b, T):
    vec = np.zeros(6)
    vec[0] = b[0]
    vec[1] = b[2]
    vec[2] = 0.5 * b[4]
    vec[3] = (1 / (2 * T**3)) * (20 * (b[1] - b[0]) - (8 * b[3] + 12 * b[2]) * T - (3 * b[5] - b[4]) * T**2)
    vec[4] = (1 / (2 * T**4)) * (30 * (b[0] - b[1]) + (14 * b[3] + 16 * b[2]) * T + (3 * b[5] - 2 * b[4]) * T**2)
    vec[5] = (1 / (2 * T**5)) * (12 * (b[1] - b[0]) - 6 * (b[3] + b[2]) * T - (b[5] - b[4]) * T**2)
    return vec

def generateTrajectory():
    # Walking Gait
    t_end_hw = 20
    T_hw = 4  # seconds
    iter_hw = 1
    t_hw = np.linspace(0, T_hw * (iter_hw + 0.75), int((T_hw * (iter_hw + 0.75) - 0) * 100))

    stride_length_hw = 10  # cm
    L_hw = stride_length_hw / 2
    H = 5  # cm

    duty_cycle = 0.75
    beta_hw = 1 - duty_cycle
    w = 1  # unit unknown

    ts_arg_hw = np.zeros((4, len(t_hw)))
    for i in range(len(t_hw)):
        ts_arg_hw[0, i] = t_hw[i] % T_hw
        ts_arg_hw[1, i] = (t_hw[i] + 3 * T_hw / 4) % T_hw
        ts_arg_hw[2, i] = (t_hw[i] + 2 * T_hw / 4) % T_hw
        ts_arg_hw[3, i] = (t_hw[i] + T_hw / 4) % T_hw

    vHW = 2 * L_hw / (beta_hw * T_hw)
    equalizer = 3
    a_x_hw = np.array([-L_hw, (-8/3) * (L_hw / T_hw), (640/3) * (L_hw / T_hw**2), (-5120/3) * (L_hw / T_hw**3), (20480/3) * (L_hw / T_hw**4), (-32768/3) * (L_hw / T_hw**5)])
    a_z_hw = np.array([0, 0, 256 * H / T_hw**2, -2048 * H / T_hw**3, 4096 * H / T_hw**4, 0])
    t_s = np.zeros((4, len(ts_arg_hw[0]), 6))

    Px_hw = np.zeros((4, len(ts_arg_hw[0])))
    Pz_hw = np.zeros((4, len(ts_arg_hw[0])))
    Py_hw = np.zeros((4, len(ts_arg_hw[0])))

    for i in range(4):
        for j in range(len(ts_arg_hw[0])):
            t_s[i, j, :] = [1, ts_arg_hw[i, j], ts_arg_hw[i, j]**2, ts_arg_hw[i, j]**3, ts_arg_hw[i, j]**4, ts_arg_hw[i, j]**5]
            ts_vec = t_s[i, j, :]
            if ts_arg_hw[i, j] < beta_hw * T_hw:
                Px_hw[i, j] = np.dot(a_x_hw, ts_vec)
                Pz_hw[i, j] = np.dot(a_z_hw, ts_vec)
            else:
                Px_hw[i, j] = -vHW * (ts_arg_hw[i, j] - beta_hw * T_hw) / equalizer + L_hw
                Pz_hw[i, j] = 0
            Py_hw[i, j] = -w * np.sin(2 * np.pi * (ts_arg_hw[i, j] - T_hw) / T_hw)

    # Commented this out because we probably want to seperate the graphs (temporary)
    # plt.figure()
    # plt.subplot(3, 1, 1)
    # plt.plot(Px_hw[0, :], Pz_hw[0, :])
    # plt.title('Hybrid Walk Gait Cycle')

    # plt.subplot(3, 1, 2)
    # plt.plot(t_hw, Pz_hw[0, :], label='Hind Right')
    # plt.plot(t_hw, Pz_hw[1, :], label='Front Right')
    # plt.plot(t_hw, Pz_hw[2, :], label='Hind Left')
    # plt.plot(t_hw, Pz_hw[3, :], label='Front Left')
    # plt.title('Z Position in Hybrid Walk')
    # plt.legend()

    # plt.subplot(3, 1, 3)
    # plt.plot(t_hw, Px_hw[0, :], label='Hind Right')
    # plt.plot(t_hw, Px_hw[1, :], label='Front Right')
    # plt.plot(t_hw, Px_hw[2, :], label='Hind Left')
    # plt.plot(t_hw, Px_hw[3, :], label='Front Left')
    # plt.ylim([-6, 12])
    # plt.title('X Position in Hybrid Walk')
    # plt.legend()
    # plt.show()

    # Trotting Gait

    t_end_hw = 20
    T_hw = 4  # seconds
    iter_hw = 1
    t_hw = np.linspace(0, T_hw * (iter_hw + 0.75), int((T_hw * (iter_hw + 0.75) - 0) * 100))

    stride_length_hw = 10  # cm
    L_hw = stride_length_hw / 2
    H = 5  # cm

    duty_cycle = 0.75
    beta_hw = 1 - duty_cycle
    w = 1  # unit unknown
    beta_t = 0.5
    T_t = 1
    vHW = 2 * L_hw / (beta_hw * T_hw)
    ts_arg_hw = np.zeros((4, len(t_hw)))
    t_s = np.zeros((4, len(ts_arg_hw[0]), 6))
    vT = 2 * vHW
    adjustment = 0.8
    vT_h = vT * adjustment
    L_tf = vT * beta_t * T_t / 2
    L_th = adjustment * L_tf
    a_x_tf = np.array([-L_tf, -4 * (L_tf / T_t), 80 * (L_tf / T_t**2), -320 * (L_tf / T_t**3), 640 * (L_tf / T_t**4), -512 * (L_tf / T_t**5)])
    a_x_th = np.array([-L_th, -4 * (L_th / T_t), 80 * (L_th / T_t**2), -320 * (L_th / T_t**3), 640 * (L_th / T_t**4), -512 * (L_th / T_t**5)])
    a_z_t = np.array([0, 0, 64 * (H / T_t**2), -256 * (H / T_t**3), 256 * (H / T_t**4), 0])
    iter = 8
    t_t = np.linspace(t_hw[-1] + 1, t_hw[-1] + T_t * iter, int((t_hw[-1] + T_t * iter - (t_hw[-1] + 1)) * 100))

    ts_arg_t = np.zeros((4, len(t_t)))
    for i in range(len(t_t)):
        ts_arg_t[0, i] = t_t[i] % T_t
        ts_arg_t[1, i] = t_t[i] % T_t
        ts_arg_t[2, i] = t_t[i] % T_t
        ts_arg_t[3, i] = t_t[i] % T_t

    Px_t = np.zeros((4, len(ts_arg_t[0])))
    Pz_t = np.zeros((4, len(ts_arg_t[0])))
    Py_t = np.zeros((4, len(ts_arg_t[0])))

    for i in range(4):
        if i == 1 or i == 3:
            for j in range(len(ts_arg_t[0])):
                t_s[i, j, :] = [1, ts_arg_t[i, j], ts_arg_t[i, j]**2, ts_arg_t[i, j]**3, ts_arg_t[i, j]**4, ts_arg_t[i, j]**5]
                ts_vec = t_s[i, j, :]
                if ts_arg_t[i, j] < beta_t * T_t:
                    Px_t[i, j] = np.dot(a_x_tf, ts_vec)
                    Pz_t[i, j] = np.dot(a_z_t, ts_vec)
                else:
                    Px_t[i, j] = -vT * (ts_arg_t[i, j] - beta_t * T_t) + L_tf
                    Pz_t[i, j] = beta_t * H * 0.5 * (1 - np.cos(2 * np.pi * (ts_arg_t[i, j] - beta_t * T_t) / (beta_t * T_t)))
                Py_t[i, j] = 0
        else:
            for j in range(len(ts_arg_t[0])):
                t_s[i, j, :] = [1, ts_arg_t[i, j], ts_arg_t[i, j]**2, ts_arg_t[i, j]**3, ts_arg_t[i, j]**4, ts_arg_t[i, j]**5]
                ts_vec = t_s[i, j, :]
                if ts_arg_t[i, j] < beta_t * T_t:
                    Px_t[i, j] = np.dot(a_x_th, ts_vec)
                    Pz_t[i, j] = np.dot(a_z_t, ts_vec)
                else:
                    Px_t[i, j] = -vT_h * (ts_arg_t[i, j] - beta_t * T_t) + L_th
                    Pz_t[i, j] = beta_t * H * 0.5 * (1 - np.cos(2 * np.pi * (ts_arg_t[i, j] - beta_t * T_t) / (beta_t * T_t)))
                Py_t[i, j] = 0

    # trotGait = plt.figure()
    # trotGait.suptitle("Trot Gait")
    # plt.subplot(3, 2, 1)
    # plt.plot(Px_t[0, :], Pz_t[0, :])
    # plt.title('Hind Gait Cycle in XZ Plane (Stationary)')
    # plt.axhline(y=H, color='r', linestyle='--')
    # plt.axhline(y=0, color='r', linestyle='--')
    # plt.axvline(x=-L_th, color='r', linestyle='--')
    # plt.axvline(x=L_th, color='r', linestyle='--')

    # plt.subplot(3, 2, 2)
    # plt.plot(Px_t[1, :], Pz_t[1, :])
    # plt.title('Front Gait Cycle in XZ Plane (Stationary)')
    # plt.axhline(y=H, color='r', linestyle='--')
    # plt.axhline(y=0, color='r', linestyle='--')
    # plt.axvline(x=-L_tf, color='r', linestyle='--')
    # plt.axvline(x=L_tf, color='r', linestyle='--')

    # plt.subplot(3, 2, 3)
    # plt.plot(t_t, Px_t[0, :], t_t, Px_t[2, :])
    # plt.title('Hind Gait Position in XZ Direction')
    # plt.xlim([t_t[0], t_t[0] + T_t * 2])
    # plt.xlabel('Time, t (s)')
    # plt.ylabel('X Position, x (cm)')
    # plt.grid(True)

    # plt.subplot(3, 2, 4)
    # plt.plot(t_t + T_t / 2, Px_t[1, :], t_t + T_t / 2, Px_t[3, :])
    # plt.title('Front Gait Position in XZ Direction')
    # plt.xlim([t_t[0], t_t[0] + T_t * 2])
    # plt.xlabel('Time, t (s)')
    # plt.ylabel('X Position, x (cm)')
    # plt.grid(True)

    # plt.subplot(3, 2, 5)
    # plt.plot3(t_t, Px_t[0, :], Pz_t[0, :], t_t, Px_t[2, :], Pz_t[2, :])
    # plt.title('Hind Gait Position in XZ Direction')
    # plt.xlim([t_t[0], t_t[0] + T_t * 2])
    # plt.xlabel('Time, t (s)')
    # plt.ylabel('X Position, x (cm)')
    # plt.zlabel('Z Position, z (cm)')
    # plt.grid(True)

    # plt.subplot(3, 2, 6)
    # plt.plot3(t_t + T_t / 2, Px_t[1, :], Pz_t[1, :], t_t + T_t / 2, Px_t[3, :], Pz_t[3, :])
    # plt.title('Front Gait Position in XZ Direction')
    # plt.xlim([t_t[0], t_t[0] + T_t * 2])
    # plt.xlabel('Time, t (s)')
    # plt.ylabel('X Position, x (cm)')
    # plt.zlabel('Z Position, z (cm)')
    # plt.grid(True)

    # plt.show()

    Px_t = [Px_t[0, 50:], Px_t[1, :-50], Px_t[2, :-50], Px_t[3, 50:]]
    Pz_t = [Pz_t[0, 50:], Pz_t[1, :-50], Pz_t[2, :-50], Pz_t[3, 50:]]
    t_t = t_t[:-50]

    # Gait Transition 2
    vHWx = 10 / 3
    vTx = 20

    t_1 = np.linspace(0, 1, 100)
    t_05 = np.linspace(0, 0.5, 50)

    FL_bx = np.array([Px_hw[3, -1], L_tf, -vHWx, -vTx, 0, 0])
    FR_bx = np.array([Px_hw[1, -1], -L_tf, -vHWx, 0, 0, 0])
    HL_bx = np.array([Px_hw[2, -1], -L_th, 0, 0, 0, 0])
    HR_bx = np.array([Px_hw[0, -1], 0.5 * L_hw, -vHWx, vTx, 0, 0])

    FL_ax = quintic_solve(FL_bx, 1)
    FR_ax = quintic_solve(FR_bx, 1)
    HL_ax = quintic_solve(HL_bx, 1)
    HR_ax = quintic_solve(HR_bx, 0.5)

    Px_gt_FL = [np.dot(FL_ax, [1, t_num, t_num**2, t_num**3, t_num**4, t_num**5]) for t_num in t_1]
    Px_gt_FR = [np.dot(FR_ax, [1, t_num, t_num**2, t_num**3, t_num**4, t_num**5]) for t_num in t_1]
    Px_gt_HL = [np.dot(HL_ax, [1, t_num, t_num**2, t_num**3, t_num**4, t_num**5]) for t_num in t_1]
    Px_gt_HR = [np.dot(HR_ax, [1, t_num, t_num**2, t_num**3, t_num**4, t_num**5]) for t_num in t_05]

    HR_bx = np.array([0.5 * L_hw, L_th, vTx, -vTx, 0, 0])
    HR_ax = quintic_solve(HR_bx, 0.5)
    Px_gt_HR = np.concatenate((Px_gt_HR, [np.dot(HR_ax, [1, t_num, t_num**2, t_num**3, t_num**4, t_num**5]) for t_num in t_05]))

    T_gt = 4 * 1
    FL_az = np.array([0, 0, 256 * H / T_gt**2, -2048 * H / T_gt**3, 4096 * H / T_gt**4, 0])
    Pz_gt_FL = [np.dot(FL_az, [1, t_num, t_num**2, t_num**3, t_num**4, t_num**5]) for t_num in t_1]
    Pz_gt_FR = np.zeros(len(t_1))
    Pz_gt_HL = np.zeros(len(t_1))

    Pz_gt_HR = np.zeros(len(t_05))
    T_gt = 4 * 0.5
    HR_az = np.array([0, 0, 256 * H / T_gt**2, -2048 * H / T_gt**3, 4096 * H / T_gt**4, 0])
    Pz_gt_HR = np.concatenate((Pz_gt_HR, [np.dot(HR_az, [1, t_num, t_num**2, t_num**3, t_num**4, t_num**5]) for t_num in t_05]))

    # plt.figure()
    # plt.title("Gait Transition 2")
    # plt.plot(t_1, Px_gt_HR, label='Hind Right')
    # plt.plot(t_1, Px_gt_FR, label='Front Right')
    # plt.plot(t_1, Px_gt_HL, label='Hind Left')
    # plt.plot(t_1, Px_gt_FL, label='Front Left')
    # plt.legend()
    # plt.show()

    Px_gt = [Px_gt_HR, Px_gt_FR, Px_gt_HL, Px_gt_FL]
    Pz_gt = [Pz_gt_HR, Pz_gt_FR, Pz_gt_HL, Pz_gt_FL]

    # Total Motion Plots
    Px1 = np.concatenate((Px_hw[0], Px_gt[0], Px_t[0]))
    Px2 = np.concatenate((Px_hw[1], Px_gt[1], Px_t[1]))
    Px3 = np.concatenate((Px_hw[2], Px_gt[2], Px_t[2]))
    Px4 = np.concatenate((Px_hw[3], Px_gt[3], Px_t[3]))

    Py1 = np.zeros((len(Px1)))
    Py2 = np.zeros((len(Px2)))
    Py3 = np.zeros((len(Px3)))
    Py4 = np.zeros((len(Px4)))

    Pz1 = np.concatenate((Pz_hw[0], Pz_gt[0], Pz_t[0]))
    Pz2 = np.concatenate((Pz_hw[1], Pz_gt[1], Pz_t[1]))
    Pz3 = np.concatenate((Pz_hw[2], Pz_gt[2], Pz_t[2]))
    Pz4 = np.concatenate((Pz_hw[3], Pz_gt[3], Pz_t[3]))

    t_gt = t_1 + t_hw[-1]
    t = np.concatenate((t_hw, t_gt, t_t))

    # plt.figure()
    # plt.suptitle('Total Motion Plots in X')
    # plt.subplot(2, 1, 1)
    # plt.plot(t, Px1, label='Hind Right')
    # plt.plot(t, Px2, label='Front Right')
    # plt.plot(t, Px3, label='Hind Left')
    # plt.plot(t, Px4, label='Front Left')
    # plt.title("X Axis Motion")
    # plt.xlabel("Time, t (s)")
    # plt.ylabel("X Position, x (cm)")
    # plt.legend()

    # plt.subplot(2, 1, 2)
    # plt.plot(t, Px1, label='Hind Right')
    # plt.plot(t, Px2, label='Front Right')
    # plt.plot(t, Px3, label='Hind Left')
    # plt.plot(t, Px4, label='Front Left')
    # plt.title("X Axis Motion Cut")
    # plt.xlabel("Time, t (s)")
    # plt.ylabel("X Position, x (cm)")
    # plt.axvline(x=t_hw[-1], color='r', linestyle='--')
    # plt.axvline(x=t_hw[-1] + 1, color='r', linestyle='--')
    # plt.axvline(x=t_hw[-1] + 1.5, color='r', linestyle='--')
    # plt.xlim([t_hw[-1] - 1, t_hw[-1] + 3])
    # plt.legend()
    # plt.show()

    # plt.figure()
    # plt.suptitle('Total Motion Plots in Z')
    # plt.subplot(2, 1, 1)
    # plt.plot(t, Pz1, label='Hind Right')
    # plt.plot(t, Pz2, label='Front Right')
    # plt.plot(t, Pz3, label='Hind Left')
    # plt.plot(t, Pz4, label='Front Left')
    # plt.title("Z Axis Motion")
    # plt.xlabel("Time, t (s)")
    # plt.ylabel("Z Position, z (cm)")
    # plt.legend()

    # plt.subplot(2, 1, 2)
    # plt.plot(t, Pz1, label='Hind Right')
    # plt.plot(t, Pz2, label='Front Right')
    # plt.plot(t, Pz3, label='Hind Left')
    # plt.plot(t, Pz4, label='Front Left')
    # plt.title("Z Axis Motion Cut")
    # plt.xlabel("Time, t (s)")
    # plt.ylabel("Z Position, z (cm)")
    # plt.axvline(x=t_hw[-1], color='r', linestyle='--')
    # plt.axvline(x=t_hw[-1] + 1, color='r', linestyle='--')
    # plt.axvline(x=t_hw[-1] + 1.5, color='r', linestyle='--')
    # plt.xlim([t_hw[-1] - 1, t_hw[-1] + 3])
    # plt.legend()
    # plt.show()

    # plt.figure()
    # plt.suptitle('Total Motion Plots in Time')
    # plt.subplot(2, 2, 1)
    # plt.plot3(Px1, t, Pz1)
    # plt.title("Hind Right Motion")
    # plt.xlabel("X Position, x (cm)")
    # plt.ylabel("Time, t (s)")
    # plt.zlabel("Z Position, z (cm)")
    # plt.ylim([t_hw[-1] - 1, t_hw[-1] + 3])
    # plt.grid(True)

    # plt.subplot(2, 2, 2)
    # plt.plot3(Px2, t, Pz2)
    # plt.title("Front Right Motion")
    # plt.xlabel("X Position, x (cm)")
    # plt.ylabel("Time, t (s)")
    # plt.zlabel("Z Position, z (cm)")
    # plt.ylim([t_hw[-1] - 1, t_hw[-1] + 3])
    # plt.grid(True)

    # plt.subplot(2, 2, 3)
    # plt.plot3(Px3, t, Pz3)
    # plt.title("Hind Left Motion")
    # plt.xlabel("X Position, x (cm)")
    # plt.ylabel("Time, t (s)")
    # plt.zlabel("Z Position, z (cm)")
    # plt.ylim([t_hw[-1] - 1, t_hw[-1] + 3])
    # plt.grid(True)

    # plt.subplot(2, 2, 4)
    # plt.plot3(Px4, t, Pz4)
    # plt.title("Front Left Motion")
    # plt.xlabel("X Position, x (cm)")
    # plt.ylabel("Time, t (s)")
    # plt.zlabel("Z Position, z (cm)")
    # plt.ylim([t_hw[-1] - 1, t_hw[-1] + 3])
    # plt.grid(True)
    # plt.show(

    return [Px1, Px2, Px3, Px4, Py1, Py2, Py3, Py4, Pz1, Pz2, Pz3, Pz4]

def runTrajectory():
    [Px1, Px2, Px3, Px4, Py1, Py2, Py3, Py4, Pz1, Pz2, Pz3, Pz4] = generateTrajectory()
    rate = rospy.Rate(10)
    i = 0
    while not i >= len(Px1):
        fl_msg = Float64MultiArray()
        fl_msg.data = [Px1[i], Py1[i], Pz1[i]]
        fl_pub.publish(fl_msg)

        fr_msg = Float64MultiArray()
        fr_msg.data = [Px2[i], Py2[i], Pz2[i]]
        # fr_pub.publish(fr_msg)

        bl_msg = Float64MultiArray()
        bl_msg.data = [Px3[i], Py3[i], Pz3[i]]
        # bl_pub.publish(bl_msg)

        br_msg = Float64MultiArray()
        br_msg.data = [Px4[i], Py4[i], Pz4[i]]
        # br_pub.publish(br_msg)

        i += 1
        time.sleep(0.1)
    rospy.loginfo("Trajectory has finished!")
        


if __name__ == '__main__':
    try:
        rospy.init_node("leg_trajectories")
        fl_pub = rospy.Publisher('/front_left_leg_target', Float64MultiArray, queue_size=10)
        fr_pub = rospy.Publisher('/back_left_leg_target', Float64MultiArray, queue_size=10)
        bl_pub = rospy.Publisher('/front_right_leg_target', Float64MultiArray, queue_size=10)
        br_pub = rospy.Publisher('/back_right_leg_target', Float64MultiArray, queue_size=10)
        rospy.loginfo("Running trajectory nope...")
        runTrajectory()
    except rospy.ROSInterruptException:
        pass