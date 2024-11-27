# This file contains all the required routines to make an A* search algorithm.
#
__authors__ = '1682347'
__group__ = 'DM.12'
# _________________________________________________________________________________________
# Intel.ligencia Artificial
# Curs 2022 - 2023
# Universitat Autonoma de Barcelona
# _______________________________________________________________________________________

from SubwayMap import *
from utils import *
import os
import math
import copy

def expand(path, map):
    """
      It expands a SINGLE station and returns the list of class Path.
      Format of the parameter is:
        Args:
            path (object of Path class): Specific path to be expanded
            map (object of Map class):: All the information needed to expand the node
        Returns:
            path_list (list): List of paths that are connected to the given path.
    """
        
    #To do until 9th of March    
    
    # Get the last station of the path
    lastStation = path.last
    
    # Create the list of paths that I will return at the end
    pathList = []
    
    # Get all the children of the last station in the given path from the map
    children = map.connections[lastStation].keys()

    for child in children:
        
        newPath = copy.deepcopy(path)
        newPath.add_route(child)
        pathList.append(newPath)

    return pathList

    pass


def remove_cycles(path_list):
    """
      It removes from path_list the set of paths that include some cycles in their path.
      Format of the parameter is:
        Args:
            path_list (LIST of Path Class): Expanded paths
        Returns:
            path_list (list): Expanded paths without cycles.
    """
    
    #To do until 9th of March  
    
    # Creating a new list to store paths without cycles
    newList = []

    for path in path_list:
        
        # Checking if the last station in the path has been previously visited
        if path.last not in path.route[:-1]:
            newList.append(path)

    return newList
    
    pass


def insert_depth_first_search(expand_paths, list_of_path):
    """
      expand_paths is inserted to the list_of_path according to DEPTH FIRST SEARCH algorithm
      Format of the parameter is:
        Args:
            expand_paths (LIST of Path Class): Expanded paths
            list_of_path (LIST of Path Class): The paths to be visited
        Returns:
            list_of_path (LIST of Path Class): List of Paths where Expanded Path is inserted
    """
    
    #To do until 9th of March
    
    #Concatenate at the beggining of the list_of_path
    list_of_path = expand_paths + list_of_path
    return list_of_path
    pass


def depth_first_search(origin_id, destination_id, map):
    """
      Depth First Search algorithm
      Format of the parameter is:
        Args:
            origin_id (int): Starting station id
            destination_id (int): Final station id
            map (object of Map class): All the map information
        Returns:
            list_of_path[0] (Path Class): the route that goes from origin_id to destination_id
    """
    
    #To do until 9th of March  

    listOfPaths = [Path(origin_id)]

    while listOfPaths:
        c = listOfPaths[0]
        
        #Verifying if the path we are checking ends at destination
        #if yes, returning the path and exiting the loop
        if c.last == destination_id:
            break
        
        e = expand(c, map)
        r = remove_cycles(e)
        
        # if the path we are checking is not the goal, we remove it
        listOfPaths.remove(c)
        
        listOfPaths = insert_depth_first_search(r, listOfPaths)
    
    if listOfPaths:
        return listOfPaths[0]
    else:
        return "No solution exists"
    pass


def insert_breadth_first_search(expand_paths, list_of_path):
    """
        expand_paths is inserted to the list_of_path according to BREADTH FIRST SEARCH algorithm
        Format of the parameter is:
            Args:
                expand_paths (LIST of Path Class): Expanded paths
                list_of_path (LIST of Path Class): The paths to be visited
            Returns:
                list_of_path (LIST of Path Class): List of Paths where Expanded Path is inserted
    """
    
    #To do until 9th of March  
    
    # Enough to extend the list_of_path; 
    # the expand_path will be added automatically at the end of the list
    list_of_path.extend(expand_paths)
    
    return list_of_path
    pass


def breadth_first_search(origin_id, destination_id, map):
    """
      Breadth First Search algorithm
      Format of the parameter is:
        Args:
            origin_id (int): Starting station id
            destination_id (int): Final station id
            map (object of Map class): All the map information
        Returns:
            list_of_path[0] (Path Class): The route that goes from origin_id to destination_id
    """
    
    #To do until 9th of March  
    
    listOfPaths = [Path(origin_id)]

    while listOfPaths:
        c = listOfPaths[0]
        
        #Verifying if the path we are checking ends at destination
        #if yes, returning the path and exiting the loop
        if c.last == destination_id:
            break
        
        e = expand(c, map)
        r = remove_cycles(e)
        
        # if the path we are checking is not the goal, we remove it
        listOfPaths.remove(c)
        
        listOfPaths = insert_breadth_first_search(r, listOfPaths)
    
    if listOfPaths:
        return listOfPaths[0]
    else:
        return "No solution exists"
    pass


def calculate_cost(expand_paths, map, type_preference=0):
    """
          Calculate the cost according to type preference
          Format of the parameter is:
            Args:
                expand_paths (LIST of Paths Class): Expanded paths
                map (object of Map class): All the map information
                type_preference: INTEGER Value to indicate the preference selected:
                                0 - Adjacency
                                1 - minimum Time
                                2 - minimum Distance
                                3 - minimum Transfers
            Returns:
                expand_paths (LIST of Paths): Expanded path with updated cost
    """
    
    #To do until 23rd of March

    #Setting the preference; iterating through the expand paths for each case 
    
    #Adjacency
    if type_preference == 0:
        for path in expand_paths:
            path.update_g(1)
    
    #Time --> given by map.connections
    elif type_preference == 1:
        for path in expand_paths:
            penultStation = path.penultimate
            lastStation = path.last
            cost = map.connections[penultStation][lastStation]
            path.update_g(cost)
            
            
    elif type_preference == 2:
            
    # minimum Distance: Using the minimum time from criteria 1 and the velocity
    # assigned to the penultimate station
    
        for path in expand_paths: 
            penultStation = path.penultimate
            lastStation = path.last
            if map.stations[lastStation]['name'] != map.stations[penultStation]['name']:
                time = map.connections[penultStation][lastStation]
                speed = map.stations[penultStation]['velocity']
                cost = time*speed
                path.update_g(cost)
            
    elif type_preference == 3:
            
    # minimum Transfers
        for path in expand_paths:
            penultStation = path.penultimate
            lastStation = path.last
            if map.stations[lastStation]['line'] != map.stations[penultStation]['line']:
                path.update_g(1)

    return expand_paths

    pass


def insert_cost(expand_paths, list_of_path):
    """
        expand_paths is inserted to the list_of_path according to COST VALUE
        Format of the parameter is:
            Args:
                expand_paths (LIST of Path Class): Expanded paths
                list_of_path (LIST of Path Class): The paths to be visited
            Returns:
                list_of_path (LIST of Path Class): List of Paths where expanded_path is inserted according to cost
    """
    
    #To do until 23rd of March
    
    #Concatenate the two lists
    list_of_path = list_of_path + expand_paths
    
    #Sort the list ordered by the cummulative cost
    list_of_path.sort(key=lambda path: path.g)
    
    return list_of_path
    
    pass


def uniform_cost_search(origin_id, destination_id, map, type_preference=0):
    """
      Uniform Cost Search algorithm
      Format of the parameter is:
        Args:
            origin_id (int): Starting station id
            destination_id (int): Final station id
            map (object of Map class): All the map information
            type_preference: INTEGER Value to indicate the preference selected:
                            0 - Adjacency
                            1 - minimum Time
                            2 - minimum Distance
                            3 - minimum Transfers
        Returns:
            list_of_path[0] (Path Class): The route that goes from origin_id to destination_id
    """
    
    #To do until 23rd of March
    
    listOfPaths = [Path(origin_id)]
    
    while listOfPaths:
        c = listOfPaths[0]
        
        #Verifying if the path we are checking ends at destination
        #if yes, returning the path and exiting the loop
        if c.last == destination_id:
            break
        
        e = expand(c, map)
        r = remove_cycles(e)
        
        # if the path we are checking is not the goal, we remove it
        listOfPaths.remove(c)
        
        # computing the cumulative cost
        cost = calculate_cost(r,map,type_preference)
        
        # inserting in list in cumulative cost order
        listOfPaths = insert_cost(cost, listOfPaths)
    
    if listOfPaths:
        return listOfPaths[0]
    else:
        return "No solution exists"
    

def calculate_heuristics(expand_paths, map, destination_id, type_preference=0):
    """
      Calculate and UPDATE the heuristics of a path according to type preference
      WARNING: In calculate_cost, we didn't update the cost of the path inside the function
              for the reasons which will be clear when you code Astar (HINT: check remove_redundant_paths() function).
      Format of the parameter is:
        Args:
            expand_paths (LIST of Path Class): Expanded paths
            map (object of Map class): All the map information
            destination_id (int): Final station id
            type_preference: INTEGER Value to indicate the preference selected:
                            0 - Adjacency
                            1 - minimum Time
                            2 - minimum Distance
                            3 - minimum Transfers
        Returns:
            expand_paths (LIST of Path Class): Expanded paths with updated heuristics
    """
    
    #To do until 23rd of March  
    
    if type_preference == 0:
        
        #Adjacency
        for path in expand_paths:
            lastStation = path.last
            if lastStation == destination_id:
                path.update_h(0)
            else:
                path.update_h(1)
                
    elif type_preference == 1:
        
        # The minimum Time will be computed using the euclidean distance and the maximum speed
        maxSpeed = 0
        for station_id, station in map.stations.items():
            if 'velocity' in station:
                maxSpeed = max(maxSpeed, station['velocity'])
                
                for path in expand_paths:
                    if path.last == destination_id:
                        path.update_h(0)
                    
                    else:
                        cur_station = map.stations[path.last]
                        dest_station = map.stations[destination_id]
                        distance = euclidean_dist([cur_station['x'], cur_station['y']], [dest_station['x'], dest_station['y']])
                        cost = distance / maxSpeed
                        path.update_h(cost)
                    
    elif type_preference == 2:
        
        # minimum Distance: euclidian distance
        for path in expand_paths:
            if path.last == destination_id:
                path.update_h(0)
                
            else:
                curStation = map.stations[path.last]
                destStation = map.stations[destination_id]
                cost = euclidean_dist([curStation['x'], curStation['y']], [destStation['x'], destStation['y']])
                path.update_h(cost)
            
                
                
    elif type_preference == 3:
        
        # minimum Transfers
            for path in expand_paths:
                lastStation = path.last
                if map.stations[lastStation]['line'] != map.stations[destination_id]['line']:
                    path.update_h(1)
                else:
                    path.update_h(0)
    # Return the updated list of paths.
    return expand_paths
    
    pass


def update_f(expand_paths):
    """
      Update the f of a path
      Format of the parameter is:
          Args:
              expand_paths (LIST of Path Class): Expanded paths
          Returns:
              expand_paths (LIST of Path Class): Expanded paths with updated costs
    """
    
    #To do until 23rd of March  
    
    for path in expand_paths:
        path.update_f()
        
    return expand_paths

    pass


def remove_redundant_paths(expand_paths, list_of_path, visited_stations_cost):
    """
      It removes the Redundant Paths. They are not optimal solution!
      If a station is visited and have a lower g-cost at this moment, we should remove this path.
      Format of the parameter is:
          Args:
              expand_paths (LIST of Path Class): Expanded paths
              list_of_path (LIST of Path Class): All the paths to be expanded
              visited_stations_cost (dict): All visited stations cost
          Returns:
              new_paths (LIST of Path Class): Expanded paths without redundant paths
              list_of_path (LIST of Path Class): list_of_path without redundant paths
              visited_stations_cost (dict): Updated visited stations cost
    """
    
    #To do until 23rd of March  
      
    for path in expand_paths:
        lastStation=path.last
        
        if lastStation in visited_stations_cost and path.g >= visited_stations_cost[lastStation]:
            # if is a redundant path if the cost of the path we are at is bigger than the cost of
            # the last station wewe were at

            expand_paths.remove(path)
            
        else:
            #if the cost is smaller, we change the cost of the last station visited
            
            visited_stations_cost[lastStation] = path.g
            
            
            for elem in list_of_path:
                if lastStation in elem.route:
                    # if the lastStation is already in the route/visited, we don't count
                    # it again --> we remove it from the list

                    list_of_path.remove(elem)

    return expand_paths, list_of_path, visited_stations_cost
    pass


def insert_cost_f(expand_paths, list_of_path):
    """
        expand_paths is inserted to the list_of_path according to f VALUE
        Format of the parameter is:
            Args:
                expand_paths (LIST of Path Class): Expanded paths
                list_of_path (LIST of Path Class): The paths to be visited
            Returns:
                list_of_path (LIST of Path Class): List of Paths where expanded_path is inserted according to f
    """
    
    #To do until 23rd of March 
    
    #Concatenate the two lists
    list_of_path = list_of_path + expand_paths
    
    #Sort the list ordered by the total cost
    list_of_path.sort(key=lambda path: path.f)
    
    return list_of_path
    
    pass


def coord2station(coord, map):
    """
        From coordinates, it searches the closest stations.
        Format of the parameter is:
        Args:
            coord (list):  Two REAL values, which refer to the coordinates of a point in the city.
            map (object of Map class): All the map information
        Returns:
            possible_origins (list): List of the Indexes of stations, which corresponds to the closest station
    """
    
    #To do until 9th of March  
    
    # The list of IDs
    listOfClosestStation = []
    
    minimumDistance = float('inf')
    
    for id, station in map.stations.items():
        
        # Computing the distance
        stationCoord = [station['x'],station['y']]
        distance = euclidean_dist(stationCoord, coord)
        
        # If there is a new minimum, we rewrite the list from scratch 
        if distance < minimumDistance:
            listOfClosestStation = [id]
            minimumDistance = distance
        
        # If there is a station with the same distance, we add it to the list
        elif distance == minimumDistance:
            listOfClosestStation.append(id)
    
    return listOfClosestStation
    
    pass


def Astar(origin_id, destination_id, map, type_preference=0):
    """
      A* Search algorithm
      Format of the parameter is:
        Args:
            origin_id (int): Starting station id
            destination_id (int): Final station id
            map (object of Map class): All the map information
            type_preference: INTEGER Value to indicate the preference selected:
                            0 - Adjacency
                            1 - minimum Time
                            2 - minimum Distance
                            3 - minimum Transfers
        Returns:
            list_of_path[0] (Path Class): The route that goes from origin_id to destination_id
    """
    
    #To do until 23rd of March  
    
    origin = map.stations[origin_id]
    destination = map.stations[destination_id]
    
    originCoord = (origin['x'],origin['y'])
    listOfPaths = [Path((coord2station(originCoord,map))[0])]
    
    destCoord = (destination['x'],destination['y'])
    destination = coord2station(destCoord, map)[0]
    costs = {}
    
    while listOfPaths:
        c = listOfPaths[0]
        
        #Verifying if the path we are checking ends at destination
        #if yes, returning the path and exiting the loop
        if c.last == destination_id:
            break
        
        e = expand(c, map)
        r = remove_cycles(e)
        
        # if the path we are checking is not the goal, we remove it
        listOfPaths.remove(c)
        print(listOfPaths)
        
        # computing the total cost step by step
        cumulativeCost = calculate_cost(r, map,type_preference)
        print("cumulTIVE:",cumulativeCost)
        heuristicCost = calculate_heuristics(cumulativeCost, map, destination, type_preference)
        print("h:",heuristicCost)
        totalCost = update_f(heuristicCost)
        
        # removing the redundancy and insterting using total cost order
        noRedundacy, listOfPaths, costs = remove_redundant_paths(totalCost, listOfPaths, costs)
        listOfPaths = insert_cost_f(noRedundacy, listOfPaths)
        
    if listOfPaths:
        return listOfPaths[0]
    else:
        return "No solution exists"
    
    pass


