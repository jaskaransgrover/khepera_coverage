#!/usr/bin/env python3
# Import demo infrastructure
from KheperaDemo import KheperaDemo
from helpers import cartesian2unicycle
from constants import ARENA_X_LOWER, ARENA_Y_LOWER, ARENA_X_UPPER, ARENA_Y_UPPER
# Import actual controller implementation
from biased_flocking import *
import numpy as np
import matlab.engine
import os
from time import sleep

R_R = 0.3 
R_H = 0.6
R_A = 5.0
h = None

robot_ips = ['108','111','112','114','118']
N_active_robots = len(robot_ips)
demo = KheperaDemo()

robot_formation = np.array([[0.5, 0.5, -0.5, -0.5, 0.0], [0.5,-0.5,-0.5,0.5,0.0]])

# Initialize ROS and all relevant subscribers/publishers
demo.connect_to_robots(robot_ips)
count = 1;
sleep(2)
while demo.alive():
    # get current (x, y, theta) for each robot (shape is 3 x N)
    robot_poses = demo.get_robot_poses()
    q = np.array([1.0,0.0]).T

    # convert cartesian controls to unicycle and send control signals
    unicycle_controls = biased_flocking_function(robot_poses, q, R_R, R_H, R_A, 0.1, N_active_robots)
    # unicycle_controls = rendezvous(robot_poses, -0.5, R_A,N_active_robots)
    # unicycle_controls = formation_control(robot_poses,0.4, robot_formation, R_A,N_active_robots)
    demo.send_control_signals(unicycle_controls)

    count += 1
    # sleep for a short time
    demo.step()