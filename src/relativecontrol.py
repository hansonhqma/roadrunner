import kinematics
import numpy as np
from hardware_interface import hardware_interface
from hid_interface import hid


# define some robot model objects
kinematics_model = kinematics.robot_model(0.03, 0.18, 0.2)
hw_interface = hardware_interface()
hid_proc = hid()

# start multiprocessing runtimes
hw_interface.start_ipc()
hid_proc.start_ipc()

pose_transorm = np.array([[1,0],[0,1]], dtype=np.float64)

def normalize(val):
    return min(127, max(-128, val))

while True:

    rpos = hw_interface.rot.value

    pose_transform = np.array([[np.cos(rpos), np.sin(rpos), 0],
                               [-np.sin(rpos), np.cos(rpos), 0],
                               [0, 0, 1]], dtype=np.float64)
    
    steer_desired_rel = pose_transform @ np.array([hid_proc.vtx.value, hid_proc.vty.value, hid_proc.vrz.value])
    
    print("Vx: {:.5f}, Vy: {:.5f}, Vrz: {:.5f}, Rz(deg): {:.3f}".format(steer_desired_rel[0], steer_desired_rel[1], steer_desired_rel[2], np.rad2deg(hw_interface.rot.value)))

    drive_input = (kinematics_model.inverse_matrix @ steer_desired_rel)

    drive_input = [normalize(x) for x in drive_input]

    hw_interface.drive(int(drive_input[0]), int(drive_input[1]), int(drive_input[2]), int(drive_input[3]))


