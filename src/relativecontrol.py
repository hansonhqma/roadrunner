import kinematics
import numpy as np
import evdev
import serial
from hardware_interface import hardware_interface

controller = evdev.InputDevice('/dev/input/event4')

kinematics_model = kinematics.robot_model(0.03, 0.18, 0.2)

hw_interface = hardware_interface()
hw_interface.start_ipc()

axis_codes = (0, 1, 3)

AXIS_MAX = 1<<15

deadzone = 0.2
drive_gain = 1.5
rotate_gain = 4

# vx, vy, vrz
steer_desired = np.array([0, 0, 0], dtype=np.float64)
pose_transorm = np.array([[1,0],[0,1]], dtype=np.float64)

for event in controller.read_loop():

    if event.type is not evdev.ecodes.EV_ABS:
        continue

    if event.code not in axis_codes:
        continue

    if event.code == 0:
        throttle = event.value/AXIS_MAX
        if abs(throttle) > deadzone:
            steer_desired[0] = throttle
        else:
            steer_desired[0] = 0


    elif event.code == 1:
        throttle = -event.value/AXIS_MAX
        if abs(throttle) > deadzone:
            steer_desired[1] = throttle
        else:
            steer_desired[1] = 0

    elif event.code == 3:
        throttle = -event.value/AXIS_MAX
        if abs(throttle) > deadzone:
            steer_desired[2] = throttle*rotate_gain
        else:
            steer_desired[2] = 0

    else:
        continue

    rpos = hw_interface.rot.value

    pose_transform = np.array([[np.cos(rpos), np.sin(rpos), 0], [-np.sin(rpos), np.cos(rpos), 0], [0, 0, 1]], dtype=np.float64)
    
    steer_desired = pose_transform @ steer_desired
    
    print("Vx: {:.5f}, Vy: {:.5f}, Vrz: {:.5f}, Rpos: {:.3f}".format(steer_desired[0], steer_desired[1], steer_desired[2], hw_interface.rot.value))

    drive_input = drive_gain * (kinematics_model.inverse_matrix @ steer_desired)
    hw_interface.drive(int(drive_input[0]), int(drive_input[1]), int(drive_input[2]), int(drive_input[3]))


