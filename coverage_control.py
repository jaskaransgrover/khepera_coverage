import numpy as np
from scipy.stats import multivariate_normal



from constants import ARENA_X_LOWER, ARENA_Y_LOWER, ARENA_X_UPPER, ARENA_Y_UPPER, ROBOT_MAX_SPEED

from voronoi import voronoi_cell_centroids
from primitives import RectangularRegion, Robot





Q = RectangularRegion([ARENA_X_LOWER, ARENA_Y_LOWER], [ARENA_X_UPPER, ARENA_Y_UPPER])


def create_phenomenon(Q):
    # num_sources = np.random.randint(3, 5)
    # sources = []
    # for _ in range(num_sources):
    #     center = [np.random.uniform(Q.minx[0], Q.maxx[0]), np.random.uniform(Q.minx[1], Q.maxx[1])]
    #     A = np.random.uniform(-1, 1, size=(2, 2))
    #     covariance = np.dot(A, A.T)
    #     sources.append(multivariate_normal(center, covariance))


    # base = np.random.uniform(0.01, 0.03)
    # phi = lambda x: base + sum(source.pdf(x) for source in sources)

    # return phi


    source1 = multivariate_normal([-1.0,-0.8], 0.2*np.eye(2))
    source2 = multivariate_normal([1.3, 0.4], 0.2*np.eye(2))
    source3 = multivariate_normal([0.3, -0.6], 0.2*np.eye(2))
    

    phi = lambda x: 0.02  + source1.pdf(x)+ source2.pdf(x)+ source3.pdf(x)
    return phi


phi = create_phenomenon(Q)


def all_robot_coverage_control(robot_poses):
    k = 0.4     # control gain

    n = robot_poses.shape[1]
    robots = [Robot(robot_poses[:2,i].T) for i in range(n)]

    xy_vels = np.zeros((2, n))

    centroids = voronoi_cell_centroids(Q, phi, robots)
    for i, robot, centroid in zip(range(n), robots, centroids):
        # move towards centroid
        vel = k * (centroid - robot.x)

        # clamp speed
        speed = np.linalg.norm(vel)
        if speed > ROBOT_MAX_SPEED:
            vel *= ROBOT_MAX_SPEED / speed

        xy_vels[:,i] = vel

    return xy_vels
