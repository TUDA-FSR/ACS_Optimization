#    Motor matrix calculation for octocopter UAV
#
#    Copyright (C) 2024  Uwe Klingauf, Tilman Strampe
#
#    This program is free software: you can redistribute it and/or modify
#    it under the terms of the GNU General Public License as published by
#    the Free Software Foundation, either version 3 of the License, or
#    (at your option) any later version.
#
#    This program is distributed in the hope that it will be useful,
#    but WITHOUT ANY WARRANTY; without even the implied warranty of
#    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
#    GNU General Public License for more details.
#
#    You should have received a copy of the GNU General Public License
#    along with this program.  If not, see <https://www.gnu.org/licenses/>.


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

