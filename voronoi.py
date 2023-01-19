import numpy as np
import scipy

def voronoi(Q, robots):
    """
    Compute the finite voronoi diagram of the given robot conditions bounded to Q.

    Returns: tuple of the form `(vertices, regions)`
    """
    # add dummy points by reflecting given points across edges of Q
    # this insures voronoi cells coincide with edges of Q
    dummy_points = []
    for r in robots:
        p = r.x
        l = [Q.minx[0] - (p[0] - Q.minx[0]), p[1]]
        r = [Q.maxx[0] + (Q.maxx[0] - p[0]), p[1]]
        d = [p[0], Q.minx[1] - (p[1] - Q.minx[1])]
        u = [p[0], Q.maxx[1] + (Q.maxx[1] - p[1])]
        dummy_points.extend([l, r, d, u])
    vor = scipy.spatial.Voronoi(np.array([r.x for r in robots] + dummy_points))

    # print(vor.vertices)

    final_vertex_indices = []
    final_regions = []

    # remove dummy vertices and re-index the regions accordingly
    for i in range(len(robots)):
        region_index = vor.point_region[i]
        region = vor.regions[region_index]
        final_region = []
        for vertex_index in region:
            if vertex_index not in final_vertex_indices:
                final_vertex_indices.append(vertex_index)
            final_region.append(final_vertex_indices.index(vertex_index))
        final_regions.append(final_region)

    final_vertices = [vor.vertices[i] for i in final_vertex_indices]
    return np.array(final_vertices), final_regions



def voronoi_discrete(Q, robots, grid_resolution):
    """
    Compute the finite voronoi diagram of the given robot conditions bounded to Q by discretization.

    Returns: array of grid points of shape (N, 2), array of robot assignments of shape (N, 1), array of distances to nearest robot of shape (N, 1)
    """

    grid_points = np.vstack(np.dstack(Q.discretize(grid_resolution)))
    
    robot_positions = np.vstack([r.x for r in robots])

    distances = scipy.spatial.distance.cdist(grid_points, robot_positions)
    assignments = np.argmin(distances, axis=1)
    shortest_distances = distances[np.arange(len(distances)), assignments]

    return grid_points, assignments, shortest_distances



def voronoi_cell_centroids(Q, phi, robots, grid_resolution=0.25):
    """
    Approximate the centroids of the voronoi cells weighted by `phi` via the rule:
    by summing over a discretization of `Q`
    """
    n = len(robots)

    grid_points, assignments, _ = voronoi_discrete(Q, robots, grid_resolution=grid_resolution)

    # Compute centroids for each voronoi cell as `(cell_distribution_sums/cell_masses)`
    cell_masses = np.zeros((n,))
    cell_distribution_sums = np.zeros((n, 2))

    for point, nearest_robot_index in zip(grid_points, assignments):
        cell_masses[nearest_robot_index] += phi(point)
        cell_distribution_sums[nearest_robot_index] += point * phi(point)
    
    centroids = cell_distribution_sums / cell_masses[:, np.newaxis]
    return centroids



def loss(Q, phi, robots):
    """
    Compute the value of the loss (aka cost) function for the given configuration
    """

    grid_points, _, distances = voronoi_discrete(Q, robots, grid_resolution=0.25)
    return np.dot(
        phi(grid_points),
        distances**2
    )




if __name__ == "__main__":
    from primitives import Robot, RectangularRegion
    from scipy.stats import multivariate_normal

    Q = RectangularRegion([0, 0], [10, 10])

    source1 = multivariate_normal([2, 7], np.eye(2))
    source2 = multivariate_normal([8, 2], 0.5*np.eye(2))

    phi = lambda x: 0.02 + source1.pdf(x) + source2.pdf(x)

    n = 5
    robots = [
        Robot([np.random.uniform(1, 9), np.random.uniform(1, 9)])
        for _ in range(n)
    ]

    # TESTING HERE

