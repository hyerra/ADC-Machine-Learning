# Traveling Salesman Problem
# Solves the Traveling Salesman Problem with the available drones to determine the most efficient route for a package.

# Calculate the distances between two points using the haversine formula.
import math
def distance(lat1, long1, lat2, long2):
    # Note: The formula used in this function is not exact, as it assumes the Earth is a perfect sphere.
    
    # Mean radius of Earth in miles.
    radius_earth = 3959
    
    # Convert latitude and longitude to spherical coordinates in radians.
    degrees_to_radians = math.pi/180.0
    phi1 = lat1 * degrees_to_radians
    phi2 = lat2 * degrees_to_radians
    lambda1 = long1 * degrees_to_radians
    lambda2 = long2 * degrees_to_radians
    dphi = phi2 - phi1
    dlambda = lambda2 - lambda1
    
    a = haversine(dphi) + math.cos(phi1) * math.cos(phi2) * haversine(dlambda)
    c = 2 * math.atan2(math.sqrt(a), math.sqrt(1-a))
    d = radius_earth * c
    return d

def haversine(angle):
  h = math.sin(angle / 2) ** 2
  return h

# Distance callback
class CreateDistanceCallback(object):
  """Creates a callback to calculate distances between points."""
  
  def __init__(self, hubs):
    """
    Latitudes and longitudes of the selected drone hubs.
    Hubs consist of warehouses, drone hubs, and the final destination.
    Once initialized, a matrix will be created containing the distances between all the hubs.
    """
    self.hubs = hubs
    
    # Create the distance matrix.
    size = len(hubs)
    self.matrix = {}

    for from_node in range(size):
      self.matrix[from_node] = {}
      for to_node in range(size):
        if from_node == to_node:
          self.matrix[from_node][to_node] = 0
        else:
          x1 = hubs[from_node][0]
          y1 = hubs[from_node][1]
          x2 = hubs[to_node][0]
          y2 = hubs[to_node][1]
          self.matrix[from_node][to_node] = distance(x1, y1, x2, y2)

  def Distance(self, from_node, to_node):
    return int(self.matrix[from_node][to_node])

from ortools.constraint_solver import pywrapcp
from ortools.constraint_solver import routing_enums_pb2
def computeShortestPath(hubs):
    """
    Computes the shortest path between the hubs.
    The hubs should include the origin and destination along with any warehouses/drone hubs in between.
    Provide the hubs as an array of coordinates of the hubs.
    """
    tsp_size = len(hubs) # The number of nodes for TSP.
    num_vehicles = 1 # The number of vehicles flying conjunction with each other to deliver the package.
    start_location = [0] # The first coordinate provided is assumed to be the starting point.
    end_location = [len(hubs) - 1] # The end coordinate is assumed to be the ending point.
    
    if tsp_size <= 0:
        raise ValueError('There needs to be at least 1 point for TSP to work. Make sure to at least input the origin and destination.')
    
    # Initialize the routing model.
    routing = pywrapcp.RoutingModel(tsp_size, num_vehicles, start_location, end_location)
    
    # Setup the search parameters.
    search_parameters = pywrapcp.RoutingModel.DefaultSearchParameters()
    
    # The callback to the distance function which determines the distances between the hubs.
    dist_between_locations = CreateDistanceCallback(hubs)
    dist_callback = dist_between_locations.Distance
    routing.SetArcCostEvaluatorOfAllVehicles(dist_callback)
    
    # Computes the shortest path between the hubs.
    shortest_path = routing.SolveWithParameters(search_parameters)
    return extractShortestPath(routing, shortest_path)
        
def extractShortestPath(routing, shortest_path):
    """
    Extracts the shortest path into two arrays: hub_order and distance which contain
    information about the best order of hubs to travel and distances.
    """
    
    if shortest_path:
        # Iterate through the nodes.
        hub_order = []
        distances = []
        
        route_number = 0 # Only one route should be returned so get the first one.
        previous_index = None # Used for computing the distances between nodes.
        index = routing.Start(route_number) # Get the starting node.
        
        while not routing.IsEnd(index):
            # Convert variable indices to node indices in the displayed route.
            # Compute the hub order.
            hub_order.append(hubs[index])
            
            # Compute the distance between the hubs.
            if previous_index is not None:
                from_hub = hubs[previous_index]
                to_hub = hubs[index]
                distance_between_nodes = distance(from_hub[0], from_hub[1], to_hub[0], to_hub[1])
                distances.append(distance_between_nodes)
                
            previous_index = index
            index = shortest_path.Value(routing.NextVar(index))
        
        # Compute the order and distance for the last point.
        hub_order.append(hubs[len(hubs) - 1])
        
        from_hub = hubs[len(hubs) - 2]
        to_hub = hubs[len(hubs) - 1]
        distance_between_nodes = distance(from_hub[0], from_hub[1], to_hub[0], to_hub[1])
        distances.append(distance_between_nodes)
        
        return (hub_order, distances)
    else:
        raise RuntimeError('No solution found for these hubs.')
    
# Compute the Traveling Salesman Problem from a hubs file on the server.
import json
hubs_data = json.load(open('hubs.json'))
hubs_array = hubs_data["hubs"]
hubs = []

for hub in hubs_array:
    hubs.append((hub["lat"], hub["lon"]))

print(computeShortestPath(hubs))