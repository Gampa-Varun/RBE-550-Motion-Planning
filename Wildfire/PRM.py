# PRM -A* algorithm
# Class is defined to implement the PRM algorithm
# Start and end positions to be given as input
# Workspace is defined as a rectangle with obstacles
# Algorithm generates random points in the workspace
# The points are connected if they are within a certain distance and the path is collision free
# The algorithm generates a roadmap of the workspace
# The roadmap is used to find the shortest path between the start and end positions

import numpy as np
import settings
import pygame
import Generate_obstacles
import reeds_shepp
import tqdm
import scipy.stats
from queue import PriorityQueue 
import queue
import time

class Node:
    def __init__(self, vertex,car_rect,car_image):
        self.vertex = vertex
        self.parent = None
        self.children = []
        self.cost_to_come = 0
        self.cost_to_go = 0
        self.net_cost = 0
        self.car_rect = car_rect
        self.car_image = car_image
        self.cost_to_neighbours = []
        self.path_to_neighbours = []

    def __lt__(self, other):
        return self.net_cost < other.net_cost
    
    def __eq__(self, other):
        if isinstance(other, Node):
            return self.vertex == other.vertex
        return False
    
class Local_Node:
    def __init__(self, state):
        self.parent = None
        self.control_required = [0,0]
        self.vertex = state
        self.cost_to_come = 0
        self.cost_to_go = 0
        self.total_cost = 0

    def __lt__(self, other):
        return self.total_cost < other.total_cost
    
def blitRotate(image, pos, originPos, angle):
    image_rect = image.get_rect(topleft = (pos[0] - originPos[0], pos[1]-originPos[1]))
    offset_center_to_pivot = pygame.math.Vector2(pos) - image_rect.center
    rotated_offset = offset_center_to_pivot.rotate(-angle)
    rotated_image_center = (pos[0] - rotated_offset.x, pos[1] - rotated_offset.y)
    rotated_image = pygame.transform.rotate(image, angle)
    rotated_image_rect = rotated_image.get_rect(center = rotated_image_center)
    car_rect = rotated_image_rect
    car_image = rotated_image
    
    return car_rect, car_image

class PRM():
    def __init__(self,obstacle_map,number_points, num_neighbours):

        self.obstacle_map = obstacle_map
        self.number_points = number_points
        self.num_neighbours = num_neighbours
        self.roadmap = None
        self.obstacle_map = obstacle_map
        self.points = []
        car = pygame.image.load("truck.png")
        car = pygame.transform.scale(car, (int(settings.car_length*settings.pixels_per_meter), int(settings.car_width*settings.pixels_per_meter)))
        self.car_rect,self.car_image = blitRotate(car, (0, 0), (car.get_width()/2, car.get_height()/2), 0)
        self.car_rect.scale_by_ip(settings.safety_factor)

    def generate_random_points(self):
        points = []
        num_points_added = 0
        while num_points_added < self.number_points:
            in_collision = False
            epsilon = np.random.uniform(0.0,1.0)
            if epsilon < settings.gaussian_dis_probability:
                random_obstacle = np.random.choice(self.obstacle_map)
                random_block = np.random.choice(range(4))
                mux = random_obstacle.x[random_block]
                muy = random_obstacle.y[random_block]
                sigmax = settings.grid_width/15
                sigmay = settings.grid_height/15
                x = int(scipy.stats.truncnorm.rvs((40-mux)/sigmax,(210-mux)/sigmax,loc = mux, scale = sigmax,size = 1))
                y = int(scipy.stats.truncnorm.rvs((40-muy)/sigmay,(210-muy)/sigmay,loc = muy, scale = sigmay,size = 1))
                theta = np.random.randint(-180, 180)
            else:
                x = int(np.random.uniform(0, settings.grid_width))
                y = int(np.random.uniform(0, settings.grid_height))
                theta = np.random.randint(-180, 180)
            car = pygame.image.load("truck.png")
            car = pygame.transform.scale(car, (int(settings.car_length*settings.pixels_per_meter), int(settings.car_width*settings.pixels_per_meter)))
            car_rect,car_image = blitRotate(car, (x*settings.pixels_per_meter, y*settings.pixels_per_meter), (car.get_width()/2, car.get_height()/2), -theta)
            car_rect.scale_by_ip(settings.safety_factor)
            for tetrominos in self.obstacle_map:
                if tetrominos.collide_with_car(car_rect):
                    in_collision = True
                    break

            if not in_collision:
                car_rect.scale_by_ip(1/settings.safety_factor)
                point_node = Node([np.float32(x),np.float32(y),np.float32(theta)],car_rect,car_image)
                points.append(point_node)
                num_points_added += 1

        self.points = points
        return points
    
    def generate_single_point(self):

        in_collision = False
        x = int(np.random.uniform(0, settings.grid_width))
        y = int(np.random.uniform(0, settings.grid_height))
        theta = np.random.randint(-180, 180)
        car = pygame.image.load("truck.png")
        car = pygame.transform.scale(car, (int(settings.car_length*settings.pixels_per_meter), int(settings.car_width*settings.pixels_per_meter)))
        car_rect,car_image = blitRotate(car, (x*settings.pixels_per_meter, y*settings.pixels_per_meter), (car.get_width()/2, car.get_height()/2), theta)
        car_rect.scale_by_ip(settings.safety_factor)
        for tetrominos in self.obstacle_map:
            if tetrominos.collide_with_car(car_rect):
                in_collision = True
                break

        if not in_collision:
            car_rect.scale_by_ip(1/settings.safety_factor)
            point_node = Node([np.float32(x),np.float32(y),np.float32(theta)],car_rect,car_image)
            self.points.append(point_node)

    
    def local_goal_check(self,next_node,goal):
        if np.linalg.norm(np.array([next_node.vertex[0],next_node.vertex[1]]) - np.array([goal.vertex[0],goal.vertex[1]])) < 2.0 and (min(np.abs(next_node.vertex[2] - goal.vertex[2]),(360-np.abs(next_node.vertex[2] - goal.vertex[2])))) < 10:
            return True
        else:
            return False
    
    def simulate_forward(self,sim_time_length,init_state,control_input):
        time_steps = np.arange(0,sim_time_length,0.1)
        x = init_state[0]
        y = init_state[1]
        theta = init_state[2]*np.pi/180

        velocity = control_input[0]
        alpha = control_input[1]
        obstacle_hit = False

        for _ in time_steps:
            x+= velocity*np.cos(theta)*0.1
            y += velocity*np.sin(theta)*0.1
            theta += velocity*np.tan(alpha)*0.1/settings.car_wheel_base

            if theta > np.pi:
                theta = theta - 2*np.pi
            elif theta < - np.pi:
                theta = theta + 2*np.pi

            rotated_car_rect,_ = blitRotate(self.car_image,(x*settings.pixels_per_meter, y*settings.pixels_per_meter),(self.car_rect.width/2,self.car_rect.height/2),-theta)
            for tetrominos in self.obstacle_map:
                if tetrominos.collide_with_car(rotated_car_rect):
                    obstacle_hit = True
                    break
            
            if obstacle_hit:
                break
            
        final_state =[int(10*x)/10,int(10*y)/10,np.float32(int(theta*180/np.pi))]
        return final_state,obstacle_hit

    def generate_controls_matrix(self,velocity_controls, steering_controls):
        control_inputs_velocity, control_inputs_steering = np.meshgrid(velocity_controls, steering_controls, indexing='ij')
        control_inputs_velocity = np.ndarray.flatten(control_inputs_velocity)
        control_inputs_steering = np.ndarray.flatten(control_inputs_steering)
        control_inputs = list(zip(control_inputs_velocity,control_inputs_steering))
        control_inputs = np.array(control_inputs)
        return control_inputs
    
    def present_in(self,queue_list,node_to_check):
        if (node_to_check.vertex[0],node_to_check.vertex[1],node_to_check.vertex[2]) in queue_list:
            return True
        else:
            return False

    def A_star_check(self,local_start_point,local_goal_point):
        
        pq = PriorityQueue()
        visited = {}
        queue = {}

        init_node = Local_Node(local_start_point.vertex)

        pq.put(init_node)
        queue[(init_node.vertex[0],init_node.vertex[1],init_node.vertex[2])] = init_node

        velocity_controls = np.asarray([-settings.max_car_speed,-settings.max_car_speed/10,settings.max_car_speed/10,settings.max_car_speed])
        steering_controls = np.arange(-settings.max_steer_angle*np.pi/180,settings.max_steer_angle*np.pi/180,np.pi/90)
        control_inputs = self.generate_controls_matrix(velocity_controls, steering_controls)
        goal_reached = False
        end_node = None

        start_time = time.time()
        time_delta = 0

        while not pq.empty():
            s = pq.get()
            del queue[(s.vertex[0],s.vertex[1],s.vertex[2])]
            visited[(s.vertex[0],s.vertex[1],s.vertex[2])] = s

            for control_input in control_inputs:
                next_state,obstacle_hit = self.simulate_forward(settings.sim_time,s.vertex,control_input)
                next_local_node = Local_Node(next_state)
                next_local_node.parent = s
                next_local_node.cost_to_come = next_local_node.parent.cost_to_come + reeds_shepp.path_length([next_local_node.vertex[0],next_local_node.vertex[1],next_local_node.vertex[2]*np.pi/180],[s.vertex[0],s.vertex[1],s.vertex[2]*np.pi/180],settings.car_wheel_base/np.tan(control_input[1]))
                next_local_node.cost_to_go = reeds_shepp.path_length([next_local_node.vertex[0],next_local_node.vertex[1],next_local_node.vertex[2]*np.pi/180],[local_goal_point.vertex[0],local_goal_point.vertex[1],local_goal_point.vertex[2]*np.pi/180],settings.car_wheel_base/np.tan(settings.max_steer_angle*np.pi/180))
                next_local_node.total_cost = next_local_node.cost_to_come + 1000*next_local_node.cost_to_go
                next_local_node.control_required = control_input

                if self.local_goal_check(next_local_node,local_goal_point) and not obstacle_hit:
                    goal_reached = True
                    end_node = next_local_node
                    break
                
                if not self.present_in(visited,next_local_node) and not self.present_in(queue,next_local_node) and not obstacle_hit:
                    queue[(next_local_node.vertex[0],next_local_node.vertex[1],next_local_node.vertex[2])] = next_local_node
                    pq.put(next_local_node)

            current_time = time.time()
            time_delta = current_time - start_time

            if time_delta > settings.time_limit:
                break
                
            if goal_reached:
                break
            
        if goal_reached:    
            path = []
            start_goal_distance = end_node.cost_to_come
            while end_node != None:
                path.append(end_node.vertex)
                end_node = end_node.parent
            path.reverse()
            return path, start_goal_distance
        
        else:
            path = []
            return path, np.float32('inf')
    
    def generate_road_map(self,draw=False):
        new_list = self.points.copy()
        for q in tqdm.tqdm(self.points):
            delete_point = False
            unable_to_reach_count = 0
            new_list.sort(key = lambda x: reeds_shepp.path_length([x.vertex[0],x.vertex[1],x.vertex[2]*np.pi/180],[q.vertex[0],q.vertex[1],q.vertex[2]*np.pi/180],settings.car_wheel_base/np.tan(settings.max_steer_angle*np.pi/180)))

            for q1 in new_list:
                if(q1 != q):
                    path, distance = self.A_star_check(q,q1)
                    if distance < np.float32('inf'):
                        q.children.append(q1)
                        q.cost_to_neighbours.append(distance)
                        q.path_to_neighbours.append(path)
                        q1.children.append(q)
                        q1.cost_to_neighbours.append(distance)
                        reversed_path = path.copy()
                        reversed_path.reverse()
                        q1.path_to_neighbours.append(reversed_path)
                        unable_to_reach_count = 0
                    else:
                        unable_to_reach_count += 1
                        #print("Unable to reach count: ",unable_to_reach_count)
                        
                if unable_to_reach_count > 40:
                    delete_point = True
                   # print("deleted")
                    break
                  
                if(len(q.children) >= self.num_neighbours):
                    break
            
            if delete_point:
                for child in q.children:
                    child.cost_to_neighbours.remove(child.cost_to_neighbours[child.children.index(q)])
                    child.path_to_neighbours.remove(child.path_to_neighbours[child.children.index(q)])
                    child.children.remove(q)
                self.points.remove(q)
                self.generate_single_point()
                continue

            if(len(q.children) < self.num_neighbours):
                for q1 in new_list:
                    if(q1 != q):
                        if q1 not in q.children:
                            q.children.append(q1)
                            dist  = reeds_shepp.path_length([q1.vertex[0],q1.vertex[1],q1.vertex[2]*np.pi/180],[q.vertex[0],q.vertex[1],q.vertex[2]*np.pi/180],settings.car_wheel_base/np.tan(settings.max_steer_angle*np.pi/180))
                            q.cost_to_neighbours.append(100*dist)
                        if(len(q.children) >= self.num_neighbours):
                            break

        if draw:
            for q in self.points:
                for child in q.children:
                    pygame.draw.line(settings.screen,(0,0,0),(q.vertex[0]*settings.pixels_per_meter,q.vertex[1]*settings.pixels_per_meter),(child.vertex[0]*settings.pixels_per_meter,child.vertex[1]*settings.pixels_per_meter))
             
    def generate_road_from_graph(self,start_point = None, goal_node = None):
        if start_point == None:
            start_point = np.random.choice(self.points)

        if goal_node == None:
            goal_node = np.random.choice(self.points)

        pq_road = []
        visited = {}
        pq_road.append(start_point)
        goal_reached = False

        end_node = None

        while pq_road:
            pq_road.sort(key = lambda x: x.net_cost)
            s = pq_road.pop(0)
            visited[tuple(s.vertex)] = s
            if s == goal_node:
                goal_reached = True
                #print("Goal Reached")
                end_node = s
                break
            for child in s.children:
                child.cost_to_come = s.cost_to_come + s.cost_to_neighbours[s.children.index(child)]
                child.cost_to_go = reeds_shepp.path_length([child.vertex[0],child.vertex[1],child.vertex[2]*np.pi/180],[goal_node.vertex[0],goal_node.vertex[1],goal_node.vertex[2]*np.pi/180],settings.car_wheel_base/np.tan(settings.max_steer_angle*np.pi/180))
                child.net_cost = child.cost_to_come + 10*child.cost_to_go
                
                if tuple(child.vertex) not in visited and child not in pq_road:
                    child.parent = s
                    pq_road.append(child)
                    

        if goal_reached:
            path = []
            start_goal_distance = end_node.cost_to_come
            detail_path = []
            path_nodes = []
            while end_node != None:
                path.append(end_node.vertex)
                path_nodes.append(end_node)
                # if (end_node.parent != None):
                #     detail_path.extend(end_node.path_to_neighbours[end_node.children.index(end_node.parent)]) # Parent is always a child, vice versa is not true
                
                end_node = end_node.parent
                #print(end_node.vertex)
           
            path.reverse()
            path_nodes.reverse()
            for i in range(len(path_nodes)-1):
                detail_path.extend(path_nodes[i].path_to_neighbours[path_nodes[i].children.index(path_nodes[i+1])])

            for point in path:
                pygame.draw.circle(settings.screen, (255,0,255), (point[0]*settings.pixels_per_meter,point[1]*settings.pixels_per_meter), 5)

            for point in self.points:
                point.parent = None
            
            return detail_path, start_goal_distance
        else:
            print("No path found")
            return None, np.float32('inf')
        
    def move(self,point):
        car_rect,car_image = blitRotate(self.car_image,(point[0]*settings.pixels_per_meter,point[1]*settings.pixels_per_meter), (self.car_image.get_width()/2,self.car_image.get_height()/2) , -point[2])
        settings.screen.blit(car_image, car_rect)



        
    
if __name__ == '__main__':
    settings.init()
    pygame.display.set_caption('PRM')
    obstacle_map = Generate_obstacles.generate_obstacles()
    prm = PRM(obstacle_map,2000, 6)
    points = prm.generate_random_points()
    prm.generate_road_map(draw=True)
    for point in points:
       pygame.draw.rect(settings.screen, (0,255,255), point.car_rect)
       settings.screen.blit(point.car_image, (point.car_rect.center[0]-point.car_image.get_rect().width/2, point.car_rect.center[1]-point.car_image.get_rect().height/2))

    for tetromino in obstacle_map:
        tetromino.draw(settings.screen)

    detail_path, distance = prm.generate_road_from_graph()

    for local_points in detail_path:
        pygame.draw.circle(settings.screen, (0,255,0), (int(local_points[0]*settings.pixels_per_meter), int(local_points[1]*settings.pixels_per_meter)), 2)
       
    
    #pygame.display.flip()
    while True:
        for local_points in detail_path:
            settings.screen.fill((255,255,255))
            print("theta: ",local_points[2])
            prm.move(local_points)
            for tetromino in obstacle_map:
                tetromino.draw(settings.screen)
            pygame.time.wait(100)
            pygame.display.update()