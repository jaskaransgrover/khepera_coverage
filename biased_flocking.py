import numpy as np

def get_relative_distance_neighbors(robot_poses, i):
	all_p = robot_poses[0:2, :].copy()
	ego_pose = robot_poses[0:2, i]
	relative_p = all_p - ego_pose.reshape((2, 1))
	relative_p_norm = np.linalg.norm(relative_p, axis=0)
	return relative_p_norm

def get_neighboring_robot_total(relative_p_norm, R_a):
	a = np.where(relative_p_norm < R_a) ;
	return a[0]

def get_robots_in_radius(robot_poses, i, R_r, R_h, R_a):
	relative_p_norm = get_relative_distance_neighbors(robot_poses, i)
	N_i = len(get_neighboring_robot_total(relative_p_norm, R_a)) - 1 # Should not include itself
	N_r_indices = []
	N_h_indices = []
	N_a_indices = []
	for (j, dist) in enumerate(relative_p_norm):
		if i != j:
			if dist < R_r:
				N_r_indices.append(j)
			elif dist < R_h:
				N_h_indices.append(j)
			elif dist < R_a:
				N_a_indices.append(j)
	return N_r_indices, N_h_indices, N_a_indices, N_i

def unit_bij(pi,pj):
	v = (pj-pi)/np.linalg.norm(pj-pi);
	return v

def angle_phi(pi,pj):
	w = (pi.T@pj)/(np.linalg.norm(pi)*np.linalg.norm(pj))
	c = np.arccos(w)
	v = (pi[0]*pj[1]) -  (pi[1]*pj[0])
	phi_ij = np.sign(v)*c
	return phi_ij

def biased_flocking_function(robot_poses, q, R_r, R_h, R_a, uv_default,N_active_robots):
	"""Compute the linear and rotational velocites of robots to cause flocking behavior

	Args:
		np.array: 3xN array of active robot poses in the form [x, y, theta].T per robot
		np.array: unit vector in the direction of travel in the form [x, y].T
		int: repulsion radius
		int: heading radius
		int: attraction radius
		float: minimum speed for the robots

	Returns:
		np.array: 2xN array of the linear and rotational velocities of the active robots in the form [uv, uw].T per robot
	"""

	controls = np.zeros((2,N_active_robots))
	for i in range(N_active_robots):
		ego_pose = robot_poses[:,i]
		p_i         = ego_pose[0:2]
		theta_i     = ego_pose[2]
		b_i         = np.array([np.cos(theta_i),np.sin(theta_i)]).T
		

		N_r_indices, N_h_indices, N_a_indices, N_i = get_robots_in_radius(robot_poses, i, R_r, R_h, R_a)

		w  = np.array([0.0,0.0]).T
		phi = 0.0
		for j in N_r_indices:
			robot_j_pose = robot_poses[:,j]
			p_j = robot_j_pose[0:2]
			theta_j = robot_j_pose[2]
			w += -1.0*unit_bij(p_i,p_j)/(np.linalg.norm(p_i-p_j)**2);
			phi += angle_phi(b_i,-1.0*unit_bij(p_i,p_j))

		for j in N_a_indices:
			robot_j_pose = robot_poses[:,j]
			p_j = robot_j_pose[0:2]
			theta_j = robot_j_pose[2]
			w += p_j-p_i
			phi += angle_phi(b_i,unit_bij(p_i,p_j))

		for j in N_h_indices:
			theta_j = robot_poses[2,j]
			b_j  = np.array([np.cos(theta_j),np.sin(theta_j)]).T
			phi += angle_phi(b_i,b_j)


		v_i = w/N_i
		gamma_i = phi/N_i
		s = ((v_i + q).T)@b_i

		uv_i = max(s, uv_default)
		if(np.abs(uv_i)>0.4):
			uv_i = 0.4*(uv_i/np.abs(uv_i))

		uw_i = gamma_i + angle_phi(b_i,q)

		controls[0,i] = uv_i
		controls[1,i] = uw_i
	return controls

def rendezvous(robot_poses, Kp, R_a,N_active_robots):
	"""Compute the linear and rotational velocites of robots to cause (anti)rendezvous behavior

	Args:
		np.array: 3xN array of active robot poses in the form [x, y, theta].T per robot
		float: Kp > 0 robots move toward each other, Kp < 0 robots move away from each other, Kp = 0 robots don't move
		int: attraction radius

	Returns:
		np.array: 2xN array of the linear and rotational velocities of the active robots in the form [uv, uw].T per robot
	"""
	controls = np.zeros((2,N_active_robots))
	for i in range(N_active_robots):
		ego_pose = robot_poses[:, i]
		p_i      = ego_pose[0:2]
		theta_i     = ego_pose[2]
		b_i         = np.array([np.cos(theta_i),np.sin(theta_i)]).T
		relative_p_norm = get_relative_distance_neighbors(robot_poses, i)
		N_i = get_neighboring_robot_total(relative_p_norm, R_a)
		N_i_mag = len(N_i)-1 #Should not include itself
		p_i_dot = np.zeros((2, 1))
		for j in N_i:
			p_j = robot_poses[0:2, j]
			p_i_dot += (p_j - p_i).reshape((2, 1))
		p_i_dot *= (Kp / N_i_mag)
		uv_i = b_i.T @ p_i_dot
		uw_i = angle_phi(b_i, p_i_dot)

		controls[0, i] = uv_i
		controls[1, i] = uw_i
	return controls

def formation_control(robot_poses,Kp, robot_formation, R_a,N_active_robots):
	"""Compute the linear and rotational velocites of robots to cause the robots to recover their formation 

	Args:
		np.array: 3xN array of active robot poses in the form [x, y, theta].T per robot
		np.array: 2xN array of formation vectors in the form [x, y].T per robot
		int: attraction radius

	Returns:
		np.array: 2xN array of the linear and rotational velocities of the active robots in the form [uv, uw].T per robot
	"""
	controls = np.zeros((2,N_active_robots))
	for i in range(N_active_robots):
		ego_pose = robot_poses[:, i]
		p_i      = ego_pose[0:2]
		theta_i     = ego_pose[2]
		b_i         = np.array([np.cos(theta_i),np.sin(theta_i)]).T
		z_i = robot_formation[:, i]
		relative_p_norm = get_relative_distance_neighbors(robot_poses, i)
		N_i = get_neighboring_robot_total(relative_p_norm, R_a)
		N_i_mag = len(N_i)-1 #Should not include itself
		p_i_dot = np.zeros((2, 1))
		for j in N_i:
			p_j = robot_poses[0:2, j]
			z_j = robot_formation[:,j]
			p_i_dot += ((p_j - p_i) - (z_j - z_i)).reshape((2, 1))
		p_i_dot *= (Kp / N_i_mag)
		uv_i = b_i.T @ p_i_dot
		uw_i = angle_phi(b_i, p_i_dot)

		controls[0, i] = uv_i
		controls[1, i] = uw_i
	return controls