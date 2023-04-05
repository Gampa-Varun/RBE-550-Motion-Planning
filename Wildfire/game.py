# Bringing both PRM and A* agents
# PRM agent will be the firetruck with kinematic constraints trying to reach the obstacles and extinguish them
# A* agent will be the wumpus trying to reach the obstacles and burn them
# Simulate the environment

import numpy as np
import settings
import pygame
import PRM
import time
import Generate_obstacles
import Wumpus_A_star
from scipy.spatial.distance import euclidean
import reeds_shepp
import tqdm


def find_obstacle_to_burn(obstacles_not_burning,points):
    #Find the point nearest to the obstacle
    obstacle_select = np.random.choice(obstacles_not_burning)
    dist = np.float32('inf')
    goal = None
    for point in points:
        if euclidean((point.vertex[0],point.vertex[1]),obstacle_select.get_center()) < dist:
            dist = euclidean((point.vertex[0],point.vertex[1]),obstacle_select.get_center())
            goal = point

    return goal, obstacle_select

def find_obstacle_to_save(obstacles_to_save,points,current_position):
    #Find the point nearest to the obstacle
    obstacles_to_save.sort(key=lambda x: x.time_left_to_live, reverse=False)
    obstacle_select = obstacles_to_save[0]
    dist = np.float32('inf')
    goal = None
    possible_points = []
    for point in points:
        if euclidean((point.vertex[0],point.vertex[1]),obstacle_select.get_center()) < 20:
            possible_points.append(point)

    for point in possible_points:
        if reeds_shepp.path_length([current_position.vertex[0],current_position.vertex[1],current_position.vertex[2]*np.pi/180],[point.vertex[0],point.vertex[1],point.vertex[2]*np.pi/180],settings.car_wheel_base/np.tan(settings.max_steer_angle*np.pi/180))<dist:
            dist = reeds_shepp.path_length([current_position.vertex[0],current_position.vertex[1],current_position.vertex[2]*np.pi/180],[point.vertex[0],point.vertex[1],point.vertex[2]*np.pi/180],settings.car_wheel_base/np.tan(settings.max_steer_angle*np.pi/180))
            goal = point

    return goal, obstacle_select

def spread_wildfire(burning_tetrominos,unburnt_tetrominos):
    #Find the burning tetrominos and spread the fire to the unburnt tetrominos
    if len(burning_tetrominos) == 0:
        return
    if len(unburnt_tetrominos) == 0:
        return
    temporary_buffer = []
    for tetromino in burning_tetrominos:
        for unburnt_tetromino in unburnt_tetrominos:

            if tetromino.distance_to_obstacle(unburnt_tetromino)<30 and tetromino.time_left_to_live<(settings.time_to_burn-10):
                unburnt_tetromino.update(0,'burn_by_wildfire')
                if unburnt_tetromino.get_status() == 1:
                    unburnt_tetrominos.remove(unburnt_tetromino)
                    temporary_buffer.append(unburnt_tetromino)
                

    for tetromino in temporary_buffer:
        burning_tetrominos.append(tetromino)


if __name__ == "__main__":
    settings.init()
    pygame.display.set_caption('PRM')
    #Generate obstacles
    obstacle_map = Generate_obstacles.generate_obstacles()

    #Generate PRM agent
    prm_agent = PRM.PRM(obstacle_map,1200,5)
    #Generate A* agent
    wumpus_agent = Wumpus_A_star.wumpus_A_star(obstacle_map)
    #Generate  points
    points = prm_agent.generate_random_points()
    #Generate graph
    prm_agent.generate_road_map()
    #Generate goal and obstacle to burn
    unburnt_obstacles = obstacle_map.copy()
    burning_obstacles = []
    burnt_obstacles = []
    goal,obstacle_to_burn = find_obstacle_to_burn(unburnt_obstacles,points)
    start,_ = find_obstacle_to_burn(unburnt_obstacles,points)
    #Generate path
    #truck_path,_ = prm_agent.generate_road_from_graph(start_point = start, goal_node=goal)
    truck_path = []
    wumpus_path = wumpus_agent.get_path([start.vertex[0],start.vertex[1]], [goal.vertex[0],goal.vertex[1]])
    #Generate controls
    for tetromino in obstacle_map:
        tetromino.draw(settings.screen)

    # for local_points in truck_path:
    #     pygame.draw.circle(settings.screen, (255,0,255), (int(local_points[0]*settings.pixels_per_meter), int(local_points[1]*settings.pixels_per_meter)), 2)

    for i in range(len(wumpus_path)-1):
        pygame.draw.line(settings.screen, (0,255,0), (wumpus_path[i][0]*settings.pixels_per_meter, wumpus_path[i][1]*settings.pixels_per_meter), (wumpus_path[i+1][0]*settings.pixels_per_meter, wumpus_path[i+1][1]*settings.pixels_per_meter), 4)

    pygame.display.update()

    wumpus_goal_reached = True
    truck_goal_reached = True
    print(start)
    start_wumpus = goal
    start_truck = start
    obstacle_to_burn.update(0.1,'burn')
    burning_obstacles.append(obstacle_to_burn)
    unburnt_obstacles.remove(obstacle_to_burn)

    num_saved = 0
    num_burned = 0
    wumpus_wait_counter  = 0
    current_wumpus_position = [start_wumpus.vertex[0],start_wumpus.vertex[1]]
    current_truck_position = [start_truck.vertex[0],start_truck.vertex[1],start_truck.vertex[2]]
    print('Starting simulation', len(burning_obstacles))
    obstacle_to_save_updated = False
    obstacle_to_burn_updated = False
    extinguished_counter = 0

    for iteration in tqdm.tqdm(range(36000)):

        settings.screen.fill((255, 255, 255))
        
        for tetromino in obstacle_map:
            tetromino.draw(settings.screen)

        if truck_goal_reached:
            if(len(burning_obstacles) != 0):
                goal_truck,obstacle_to_save = find_obstacle_to_save(burning_obstacles,points,start_truck)
                truck_path,_ = prm_agent.generate_road_from_graph(start_point = start_truck, goal_node=goal_truck)
                if truck_path == None:
                    truck_goal_reached = True
                    truck_path = []
                else:
                    truck_goal_reached = False
            
        
        if wumpus_goal_reached:
            if(len(unburnt_obstacles) != 0):
                goal_wumpus, obstacle_to_burn = find_obstacle_to_burn(unburnt_obstacles,points)
                wumpus_path = wumpus_agent.get_path([start_wumpus.vertex[0],start_wumpus.vertex[1]], [goal_wumpus.vertex[0],goal_wumpus.vertex[1]])
                wumpus_goal_reached = False
        
        if wumpus_wait_counter == settings.wumpus_speed_limiter:
            wumpus_wait_counter = 0
            if len(wumpus_path) > 0:
                wumpus_agent.move(wumpus_path[0])
                current_wumpus_position = wumpus_path[0]
                local_points = wumpus_path[0]
                pygame.draw.circle(settings.screen, (0,255,0), (int(local_points[0]*settings.pixels_per_meter), int(local_points[1]*settings.pixels_per_meter)), 5)
                wumpus_path.pop(0)
        else:
            wumpus_wait_counter += 1
            wumpus_agent.move(current_wumpus_position)
            pygame.draw.circle(settings.screen, (0,255,0), (int(current_wumpus_position[0]*settings.pixels_per_meter), int(current_wumpus_position[1]*settings.pixels_per_meter)), 5)
        
        if len(truck_path) > 0:
            prm_agent.move(truck_path[0])
            current_truck_position = truck_path[0]
            local_points = truck_path[0]
            pygame.draw.circle(settings.screen, (255,0,255), (int(local_points[0]*settings.pixels_per_meter), int(local_points[1]*settings.pixels_per_meter)), 5)
            truck_path.pop(0)

            

        if len(wumpus_path) == 0 and not wumpus_goal_reached:
            wumpus_goal_reached = True
            start_wumpus = goal_wumpus
            if(obstacle_to_burn.status == 0):
                obstacle_to_burn.update(settings.sim_time,'burn')
                obstacle_to_burn_updated = True
                burning_obstacles.append(obstacle_to_burn)
                if obstacle_to_burn in unburnt_obstacles:
                    unburnt_obstacles.remove(obstacle_to_burn)

        elif len(wumpus_path) == 0 and wumpus_goal_reached:
            wumpus_agent.move(current_wumpus_position)
            pygame.draw.circle(settings.screen, (0,255,0), (int(current_wumpus_position[0]*settings.pixels_per_meter), int(current_wumpus_position[1]*settings.pixels_per_meter)), 5)

        
        
        
        if len(truck_path) == 0 and not truck_goal_reached:
            if extinguished_counter == int(settings.time_to_extinguish/settings.sim_time):
                extinguished_counter = 0
                truck_goal_reached = True
                start_truck = goal_truck
                obstacle_to_save.update(settings.sim_time,'save')
                obstacle_to_save_updated = True
                if obstacle_to_save.get_status() != 2:
                    num_saved +=1
                    unburnt_obstacles.append(obstacle_to_save)
                    burning_obstacles.remove(obstacle_to_save)
            else :
                extinguished_counter += 1
                prm_agent.move(current_truck_position)
                pygame.draw.circle(settings.screen, (128,100,255), (int(current_truck_position[0]*settings.pixels_per_meter), int(current_truck_position[1]*settings.pixels_per_meter)), 5)

        elif len(truck_path) == 0 and truck_goal_reached:
            prm_agent.move(current_truck_position)
            pygame.draw.circle(settings.screen, (128,100,255), (int(current_truck_position[0]*settings.pixels_per_meter), int(current_truck_position[1]*settings.pixels_per_meter)), 5)

        for obstacle in obstacle_map:
            if obstacle.status == 2:
                burnt_obstacles.append(obstacle)
                #print("check",len(burning_obstacles))
                if obstacle in burning_obstacles:
                    burning_obstacles.remove(obstacle)


        spread_wildfire(burning_obstacles,unburnt_obstacles)
        

        for tetromino in obstacle_map:
            if tetromino == obstacle_to_burn and obstacle_to_burn_updated:
                obstacle_to_burn_updated = False
                continue
            elif tetromino == obstacle_to_save and obstacle_to_save_updated:
                obstacle_to_save_updated = False
                continue
            tetromino.update(settings.sim_time)

        # if len(burning_obstacles) == 0 and len(unburnt_obstacles) == 0:
        #     break

                

        #print("iteration",iteration)

        pygame.display.update()
        #pygame.time.wait(10)


    for obstacle in obstacle_map:
        if obstacle.status == 2:
            num_burned +=1
        


    print('Number of obstacles saved',num_saved/len(obstacle_map))
    print('Number of obstacles burned',num_burned/len(obstacle_map))
            
        

