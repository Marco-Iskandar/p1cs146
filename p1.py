from p1_support import load_level, show_level, save_level_costs
from math import inf, sqrt
from heapq import heappop, heappush
import queue

def dijkstras_shortest_path(initial_position, destination, graph, adj):
    frontier = queue.PriorityQueue()
    frontier.put(initial_position, 0)
    visited = []
    came_from = {}
    cost_so_far = {}
    came_from[initial_position] = None
    cost_so_far[initial_position] = 0
    while not frontier.empty():
        current = frontier.get()
        if current == destination:
            break
       
        for currnode in adj(graph,current):
            if currnode in visited: continue
            new_cost = cost_so_far[current] + currnode[1]
            if currnode not in cost_so_far or new_cost < cost_so_far[currnode]:
                cost_so_far[currnode[0]] = new_cost
                priority = new_cost
                if currnode[0] not in visited:
                    frontier.put(currnode[0], priority)
                    visited.append(currnode)
                came_from[currnode[0]] = current
                
     
    """queue=[]
    heappush(queue, initial_position)
    while queue:
        adj(graph,queue[0])
        heappop(queue)"""
    """ Searches for a minimal cost path through a graph using Dijkstra's algorithm.

    Args:
        initial_position: The initial cell from which the path extends.
        destination: The end location for the path.
        graph: A loaded level, containing walls, spaces, and waypoints.
        adj: An adjacency function returning cells adjacent to a given cell as well as their respective edge costs.

    Returns:
        If a path exits, return a list containing all cells from initial_position to destination.
        Otherwise, return None.

    """
    print(came_from.__class__)
    return came_from
    pass


def dijkstras_shortest_path_to_all(initial_position, graph, adj):   
    
    frontier = queue.PriorityQueue()
    frontier.put(initial_position, 0)
    visited = []
    came_from = {}
    cost_so_far = {}
    came_from[initial_position] = None
    cost_so_far[initial_position] = 0
    while not frontier.empty():
        current = frontier.get()
       
        for currnode in adj(graph,current):
            if currnode in visited: continue
            new_cost = cost_so_far[current] + currnode[1]
            print (new_cost)
            if currnode not in cost_so_far or new_cost < cost_so_far[currnode]:
                cost_so_far[currnode[0]] = new_cost
                priority = new_cost
                if currnode[0] not in visited:
                    frontier.put(currnode[0], priority)
                    visited.append(currnode)
                came_from[currnode[0]] = graph['spaces'][current]
    """ Calculates the minimum cost to every reachable cell in a graph from the initial_position.

    Args:
        initial_position: The initial cell from which the path extends.
        graph: A loaded level, containing walls, spaces, and waypoints.
        adj: An adjacency function returning cells adjacent to a given cell as well as their respective edge costs.

    Returns:
        A dictionary, mapping destination cells to the cost of a path from the initial_position.
    """
    return cost_so_far
    pass


def navigation_edges(level, cell):
    # A cell is a 'tuple' that is used to store an x/y coordinate
    dirs = [[1, 0], [0, 1], [-1, 0], [0, -1]]
    diags = [[1, -1], [1, 1], [-1, 1], [-1, -1]]
    result = []
    #basic directions
    for currdir in dirs:
        neighborpos = (cell[0] + currdir[0], cell[1] + currdir[1])
        if neighborpos in level['spaces']:
            neighbor = (neighborpos, (level['spaces'][neighborpos]/2)+(level['spaces'][cell]/2))
            result.append(neighbor)
    #diagonals   
    for currdir in diags:
        neighborpos = (cell[0] + currdir[0], cell[1] + currdir[1])
        if neighborpos in level['spaces']:
            neighbor = (neighborpos, (level['spaces'][neighborpos])*(sqrt(2))/2+(level['spaces'][cell])+(sqrt(2))/2)
            result.append(neighbor)
    """ Provides a list of adjacent cells and their respective costs from the given cell.

    Args:
        level: A loaded level, containing walls, spaces, and waypoints.
        cell: A target location.

    Returns:
        A list of tuples containing an adjacent cell's coordinates and the cost of the edge joining it and the
        originating cell.

        E.g. from (0,0):
            [((0,1), 1),
             ((1,0), 1),
             ((1,1), 1.4142135623730951),
             ... ]
    """
    return result
    pass


def test_route(filename, src_waypoint, dst_waypoint):
    """ Loads a level, searches for a path between the given waypoints, and displays the result.

    Args:
        filename: The name of the text file containing the level.
        src_waypoint: The character associated with the initial waypoint.
        dst_waypoint: The character associated with the destination waypoint.

    """

    # Load and display the level.
    level = load_level(filename)
    show_level(level)

    # Retrieve the source and destination coordinates from the level.
    src = level['waypoints'][src_waypoint]
    dst = level['waypoints'][dst_waypoint]

    # Search for and display the path from src to dst.
    path = dijkstras_shortest_path(src, dst, level, navigation_edges)
    if path:
        show_level(level, path)
    else:
        print("No path possible!")


def cost_to_all_cells(filename, src_waypoint, output_filename):
    """ Loads a level, calculates the cost to all reachable cells from 
    src_waypoint, then saves the result in a csv file with name output_filename.

    Args:
        filename: The name of the text file containing the level.
        src_waypoint: The character associated with the initial waypoint.
        output_filename: The filename for the output csv file.

    """
    
    # Load and display the level.
    level = load_level(filename)
    show_level(level)

    # Retrieve the source coordinates from the level.
    src = level['waypoints'][src_waypoint]
    
    # Calculate the cost to all reachable cells from src and save to a csv file.
    costs_to_all_cells = dijkstras_shortest_path_to_all(src, level, navigation_edges)
    save_level_costs(level, costs_to_all_cells, output_filename)


if __name__ == '__main__':
    filename, src_waypoint, dst_waypoint = 'example.txt', 'a', 'e'
    

    # Use this function call to find the route between two waypoints.
    test_route(filename, src_waypoint, dst_waypoint)

    # Use this function to calculate the cost to all reachable cells from an origin point.
    cost_to_all_cells(filename, src_waypoint, 'my_costs.csv')
