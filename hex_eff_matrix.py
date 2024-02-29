import numpy as np
import math 

class parameters:
  def __init__(self, mu, ct, cm, l_arm, l0):
    self.l_arm = l_arm    # motor distance to c.g.
    self.mu = mu          # Inclination angle of rotors in [deg]
    self.ct = ct          # Rotor thrust constant
    self.cm = cm          # Rotor torque constant


def motor_matrix(param):

    l = param.l_arm
    sin_60 = math.sin(60*math.pi/180)
    cos_60 = math.cos(60*math.pi/180)

    sin_mu = math.sin(param.mu*math.pi/180)
    cos_mu = math.cos(param.mu*math.pi/180)

    # rotation directions (CCW = 1, CW = -1) 
    #dirs = np.array([1, -1, 1, -1, 1, -1])
    dirs = np.array([1, 1, -1, -1, 1, -1])

    # rotor positions (vs. CG in body coordinates)
    pos = np.array([[l*sin_60, 0, -l*sin_60, -l*sin_60,  0,  l*sin_60],
                    [l*cos_60, l,  l*cos_60, -l*cos_60, -l, -l*cos_60],
                    [       0, 0,         0,         0,  0,         0]])

    # rotation axes 
    axes = np.array([[-sin_mu*cos_60,  sin_mu, -sin_mu*cos_60, -sin_mu*cos_60,  sin_mu,  -sin_mu*cos_60],
                     [ sin_mu*sin_60,     0.0, -sin_mu*sin_60,  sin_mu*sin_60,     0.0,  -sin_mu*sin_60],
                     [       -cos_mu, -cos_mu,        -cos_mu,        -cos_mu, -cos_mu,         -cos_mu]])

    torque = np.transpose(param.ct * np.cross(np.transpose(pos), np.transpose(axes))) - param.cm * axes @ np.diag(dirs)
    thrust = param.ct * axes

    B = np.vstack((torque, thrust))

    return B

