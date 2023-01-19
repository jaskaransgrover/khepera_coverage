import numpy as np
from math import sin, cos

from constants import ROBOT_RADIUS


def quaternion_to_euler(x, y, z, w):
	t3 = +2.0 * (w * z + x * y)
	t4 = +1.0 - 2.0 * (y * y + z * z)
	yaw = np.arctan2(t3, t4)
	yaw = (yaw + np.pi) % (2 * np.pi) - np.pi
	return yaw


def cartesian2unicycle(robot_poses, xy_vels):
	n = robot_poses.shape[1]
	hD = np.zeros([2,n])
	for k in range(n):
		ego_robot_pose = robot_poses[:,k]
		thetaD         = ego_robot_pose[2]
		Rmat           = np.array([[cos(thetaD),-sin(thetaD)],[sin(thetaD),cos(thetaD)]])
		Mmat           = np.array([[1,0],[0, ROBOT_RADIUS]])
		Jmat    	   = Rmat@Mmat
		hD[:,k]        = np.linalg.inv(Jmat)@xy_vels[:,k]

	return hD

