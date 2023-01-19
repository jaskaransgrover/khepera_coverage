#!/usr/bin/env python3
# Import demo infrastructure
from KheperaDemo import KheperaDemo
from helpers import cartesian2unicycle
from constants import ARENA_X_LOWER, ARENA_Y_LOWER, ARENA_X_UPPER, ARENA_Y_UPPER
# Import actual controller implementation
from primitives import RectangularRegion
from coverage_control import all_robot_coverage_control, create_phenomenon
import numpy as np
import matlab.engine
import os
from time import sleep
import time
from datetime import datetime

# Connect to matlab engine and render graphics
matlab_engine = matlab.engine.start_matlab()
matlab_engine.cd(os.path.dirname(__file__))
print("Matlab engine started!")

Q = RectangularRegion([ARENA_X_LOWER, ARENA_Y_LOWER], [ARENA_X_UPPER, ARENA_Y_UPPER])
phi = create_phenomenon(Q)

X, Y = Q.discretize(0.005)
Z = phi(np.dstack((X, Y)))
X = matlab.double(X.tolist())
Y = matlab.double(Y.tolist())
Z = matlab.double(Z.tolist())
matlab_engine.display_heatmap(X, Y, Z, nargout=0)

h_old = 1

robot_ips = ['108','111','112','113','114','115','117']
N = len(robot_ips)
demo = KheperaDemo()

# Initialize ROS and all relevant subscribers/publishers
demo.connect_to_robots(robot_ips)
sleep(5)
count = 1;

while demo.alive():
    robot_poses = demo.get_robot_poses()

    if h_old is not None:
        h_old = matlab_engine.display_voronoi(matlab.double(robot_poses[0,:].tolist()), matlab.double(robot_poses[1,:].tolist()),h_old, nargout=1);
   
    xy_velocities = all_robot_coverage_control(robot_poses)
    # convert cartesian controls to unicycle and send control signals
    unicycle_controls = cartesian2unicycle(robot_poses, xy_velocities)
    demo.send_control_signals(unicycle_controls)
    count += 1
    demo.step()
   
    