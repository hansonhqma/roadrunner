import kinematics
import numpy as np
from hardware_interface import hardware_interface
from hid_interface import hid
from controller import pid

def normalize(val):
    return min(127, max(-128, val))

# define some robot model objects
kinematics_model = kinematics.robot_model(0.03, 0.18, 0.2)
hw_interface = hardware_interface()
hid_proc = hid()

# controller objects
vtx_control = pid(1, 1, 1)
vty_control = pid(1, 1, 1)
vrz_control = pid(1, 1, 1)

# start multiprocessing runtimes
hw_interface.start_ipc()
hid_proc.start_ipc()

# wheel motion vector

drive_vector = np.zeros((4))
while True:

    # grab current z rotation
    rpos = hw_interface.rot.value



    # compute rotation matrix
    pose_transform = np.array([[np.cos(rpos), np.sin(rpos), 0],
                               [-np.sin(rpos), np.cos(rpos), 0],
                               [0, 0, 1]], dtype=np.float64)
    
    # grab desired robot frame velocity vector
    vel_desired = pose_transform @ np.array([hid_proc.vtx.value, hid_proc.vty.value, hid_proc.vrz.value])

    # grab current velocity vector
    vel_current = np.array([hw_interface.t_vel[0], hw_interface.t_vel[1], hw_interface.r_vel.value], dtype=np.float64)

    # compute error vector
    vel_error = vel_desired - vel_current

    step_vector = np.array([vtx_control.gain(vel_error[0]),
                            vty_control.gain(vel_error[1]),
                            vrz_control.gain(vel_error[2])])


    drive_step = kinematics_model.inverse_matrix @ step_vector
    drive_vector += drive_step

    drive_input = [normalize(x) for x in drive_vector]

    print("vtx err: {:.3f}\tvty err: {:.3f}\tvrz err:{:.3f}\t\t".format(vel_error[0], vel_error[1], vel_error[2]))

    hw_interface.drive(int(drive_input[0]), int(drive_input[1]), int(drive_input[2]), int(drive_input[3]))


