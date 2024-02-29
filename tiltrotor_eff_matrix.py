import numpy as np
import math 

class parameters:
  def __init__(self, tilt, ct, cm, dxy, s):
    self.tilt = tilt                                # Vector of tilt angles
    self.ct = ct                                    # Rotor thrust constant
    self.cm = cm                                    # Rotor torque constant
    self.dxy = dxy                                  # Position of rotors relative to cg in x- and y-direction
    self.s = s                                      # Wing span (assuming rotors and wing tips)


def motor_matrix(param):

    tilt = np.copy(param.tilt)

    dxy = param.dxy      # 0.32
    dz = 0.0
    s = param.s/2  # wing span  1.827/2

    # Define constant position and thrust directions for multicopter mode
    origin = np.array([[dxy, dxy, 0, -dxy, -dxy, 0],    # body-KOS; MC mode
                       [dxy, -dxy, -s, -dxy, dxy, s],
                       [-dz, -dz,  -dz, -dz, -dz, -dz]])
    
    direction_MC = np.array([[ 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],   # body-KOS; MC mode
                             [ 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
                             [-1.0, -1.0, -1.0, -1.0, -1.0, -1.0]])

    # Define thrust direction for fixed wing mode
    direction_FW = np.copy(direction_MC)
    direction_FW[0,2] = 1.0
    direction_FW[2,2] = 0.0
    direction_FW[0,5] = 1.0
    direction_FW[2,5] = 0.0

    # Calculate direction of thrust vectors with tilted rotors for transition mode
    delta_imax = 90*math.pi/180
    direction = np.copy(direction_MC)
    

    direction[0:3,2] = math.sin((1-tilt[0])*delta_imax)/math.sin(delta_imax) * direction_MC[0:3,2] + math.sin(tilt[0]*delta_imax)/math.sin(delta_imax) * direction_FW[0:3,2]
    direction[0:3,5] = math.sin((1-tilt[1])*delta_imax)/math.sin(delta_imax) * direction_MC[0:3,5] + math.sin(tilt[1]*delta_imax)/math.sin(delta_imax) * direction_FW[0:3,5]

    
    # Calculate input matrix

    rotation_direction = np.array([1, -1, -1, 1, -1, 1])
    moments = np.transpose(param.cm*np.dot(np.diag(rotation_direction),np.transpose(direction))) 

    B = np.zeros((6,6))
    for i in range(6):
        # Forces and torque from rotor forces
        B[0:3,i] = np.cross(origin[0:3,i], param.ct*direction[0:3,i])  
        B[3:6,i] = param.ct*direction[0:3,i]                        # Force x,y,z (body) 

        # Torque from rotor torques
        B[0:3,i] = B[0:3,i] + moments[0:3,i]

    return B

