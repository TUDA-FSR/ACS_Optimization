## Optimization of Attainable Control Sets and Weighted Pseudo-Inverse Control Allocation Matrices

This repository contains examples to optimize the Attainable Control Set (ACS) and the weighted Pseudo-Inverse Matrix. 
The files 'xx_eff_matrix' are used to create input matrices for different multirotor unmanned aircraft. Different parameters such as rotor tilt angles or rotor inclinations can be passed to the function motor_matrix in these files.

The Jupyter notebooks 'tiltcopter' and 'octocopter' show examples of the optimization for two types of UAVs. In both cases the radius in the roll-pitch plane is optimized in order to find the largest attainable control set. A constant control allocation matrix is optimized in the Jupyter notebooks as well. The Jupyter notebook 'tiltcopter_ellipsoid' shows the optimization of ACS and control allocation matrix when the ellipsoid as a geometric measure. 
