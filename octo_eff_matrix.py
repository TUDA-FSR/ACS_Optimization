import numpy as np
import math 

class parameters:
  def __init__(self, mu, ct, cm, l1, l2):
    self.l1 = l1    	# odd motor arm length
    self.l2 = l2    	# even motor arm length
    self.mu = mu        # Inclination angle of rotors in [deg]
    self.ct = ct        # Rotor thrust constant
    self.cm = cm        # Rotor torque constant


def motor_matrix(param):

    l1 = param.l1
    l2 = param.l2
    sin_45 = 0.5*math.sqrt(2)
    
    sin_mu = math.sin(param.mu*math.pi/180)
    cos_mu = math.cos(param.mu*math.pi/180)

    # rotation directions (CCW = 1, CW = -1) 
    dirs = np.array([-1, 1, -1, 1, -1, 1, -1, 1])

    # rotor positions (vs. CG in body coordinates)
    pos = np.array([[l1, sin_45*l2,  0, -sin_45*l2, -l1, -sin_45*l2,   0,  sin_45*l2],
                    [ 0, sin_45*l2, l1,  sin_45*l2,   0, -sin_45*l2, -l1, -sin_45*l2],
                    [ 0,         0,  0,          0,   0,          0,   0,          0]])

    # rotation axes 
    axes = np.array([[      0, -sin_45*sin_mu,  sin_mu, -sin_45*sin_mu,       0,  sin_45*sin_mu, -sin_mu, sin_45*sin_mu],
                     [-sin_mu,  sin_45*sin_mu,       0, -sin_45*sin_mu,  sin_mu, -sin_45*sin_mu,       0, sin_45*sin_mu],
                     [-cos_mu,        -cos_mu, -cos_mu,        -cos_mu, -cos_mu,        -cos_mu, -cos_mu,       -cos_mu]])

    torque = np.transpose(param.ct * np.cross(np.transpose(pos), np.transpose(axes))) - param.cm * axes @ np.diag(dirs)
    thrust = param.ct * axes

    B = np.vstack((torque, thrust))

    return B

