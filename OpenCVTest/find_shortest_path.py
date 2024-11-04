import numpy as np

def find_shortest_path(points: np.ndarray) -> np.ndarray:
    '''
    Calculate the shortest path between points

    returns: np.ndarray with the sorted points where [0] is the start point
    '''

    # Initialize the matrices
    num_points = points.shape[0]

    start_point = np.arange(num_points)

    distance = np.zeros((num_points, num_points))

    total_distance = np.zeros(num_points)

    route = np.zeros((num_points, num_points))

    # Calculate the distance between points
    for i in range(num_points):
        for j in range(num_points):
            distance[i, j] = np.sqrt((points[i, 0] - points[j, 0])**2 + (points[i, 1] - points[j, 1])**2)
    distance = np.triu(distance)
    print(distance)

    for i, n in enumerate(start_point):
        route[i, 0] = n
        dist = distance
        for j in range(num_points):
            
        

    return points




if __name__ == "__main__":
    points = np.array([[0, 0], [1, 1], [2, 2], [3, 3]])

    sort_points = find_shortest_path(points)

    print(sort_points)