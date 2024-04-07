import numpy as np

class robot_model:
    
    def __init__(self, radius, x_pitch, y_pitch):
        self.radius = radius
        self.x_pitch = x_pitch
        self.y_pitch = y_pitch

        fwd_rot = 2/(self.x_pitch + self.y_pitch)
        inv_rot = (self.x_pitch + self.y_pitch)/2
        self.forward_matrix = np.array([[1, -1, -1, 1],
                                        [1, 1, 1, 1],
                                        [-fwd_rot, fwd_rot, -fwd_rot, fwd_rot]])*self.radius/4

        self.inverse_matrix = np.array([[1, 1, -inv_rot],
                                        [-1, 1, inv_rot],
                                        [-1, 1, -inv_rot],
                                        [1, 1, inv_rot]])/self.radius


