from SearchAlgorithm import *
from SubwayMap import *
from utils import *

if __name__=="__main__":
    #ROOT_FOLDER = '../CityInformation/Barcelona_City/'
    ROOT_FOLDER = '../CityInformation/Lyon_smallCity/'
    ROOT_FOLDER = '../Barcelona_City/Barcelona_City/'
    map = read_station_information(os.path.join(ROOT_FOLDER, 'Stations.txt'))
    connections = read_cost_table(os.path.join(ROOT_FOLDER, 'Time.txt'))
    map.add_connection(connections)

    infoVelocity_clean = read_information(os.path.join(ROOT_FOLDER, 'InfoVelocity.txt'))
    map.add_velocity(infoVelocity_clean)



    ### BELOW HERE YOU CAN CALL ANY FUNCTION THAT yoU HAVE PROGRAMED TO ANSWER THE QUESTIONS OF THE EXAM ###
    ### this code is just for you, you won't have to upload it after the exam ###


    #this is an example of how to call some of the functions that you have programed
    example_path=uniform_cost_search(9, 3, map, 2)
    print_list_of_path_with_cost([example_path])
    
    """
          example_path=uniform_cost_search(origin_id, destination_id, map, type_pref)
          
          type_pref e 0, 1, 2 sau 3 pentru UCS si Astar
          
          origin_id is destination_id se gasesc in txt-ul Station din foldertul pe care ni-l vor da ei
          
          example_path=uniform_cost_search(9, 3, map, 1)
          
          example_path=breadth_first_search(origin_id, destination_id, map)
          
          example_path=depth_first_search(origin_id, destination_id, map)
          
          example_path=Astar(origin_id, destination_id, map, type_preference)
          
          -------------------------------------------------------------------------------------------------
          
          example_G_cost = calculate_cost(expand_paths, map, type_preference=0) 
          
          example_H_cost = calculate_heuristics(expand_paths, map, destination_id, type_preference=0) 
          
          example_F_cost = update_f(expand_paths) 
          
          expand_paths pare sa ni-l dea ei in intrebare
          
          type_preference = 0 --> adiacenta
                          = 1 --> timp
                          = 2 --> distanta
                          = 3 --> transferuri
    """
    
    # #example = remove_redundant_paths(e, [[2,7,1,3,6],[2,7,1,3,5],[2,7,1,3,2],[2,7,1,3,7],[2,7,1,3,4]], visited_stations_cost)
    # example_path=breadth_first_search(24, 19, map)
    # #example_path=uniform_cost_search(24, 19, map, 1)
    # print(example_path)

    example_path=Astar(12, 17, map, 1)
    print(example_path.h)
    #print_list_of_path_with_cost([example_path])