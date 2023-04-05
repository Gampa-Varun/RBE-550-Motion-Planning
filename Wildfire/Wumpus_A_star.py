# A-Star algorithm for the wumpus agent
# Wumpus doesn't have any kinematic constraints
# It's path planning is based on grid search algorithm
# The algorithm chosen is A-Star



import numpy as np
import settings
import pygame
from scipy.spatial.distance import euclidean
import Generate_obstacles
import time
from queue import PriorityQueue


#node class
class Node:
    def __init__(self, x, y):
        self.x = x
        self.y = y
        self.parent = None
        self.g = 0
        self.h = 0
        self.f = self.g + self.h
        self.control_input = None


    def __eq__(self, other):
        return self.x == other.x and self.y == other.y
    
    def __lt__(self, other):
        return self.f < other.f
    
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
    
class wumpus_A_star():
    def __init__(self, obstacle_map):
        #Start and goal node are points, x,y
        #Obstacle map is the list of obstacles(tetrominos) coming from the Generate_obstacles.py file
        self.obstacle_map = obstacle_map
        car = pygame.image.load("truck.png")
        self.car = pygame.transform.scale(car, (int(settings.wumpus_size*settings.pixels_per_meter), int(settings.wumpus_size*settings.pixels_per_meter)))
        self.car_rect = self.car.get_rect()

    def blitRotate(self,image, pos, originPos, angle):
        image_rect = image.get_rect(topleft = (pos[0] - originPos[0], pos[1]-originPos[1]))
        offset_center_to_pivot = pygame.math.Vector2(pos) - image_rect.center
        rotated_offset = offset_center_to_pivot.rotate(-angle)
        rotated_image_center = (pos[0] - rotated_offset.x, pos[1] - rotated_offset.y)
        rotated_image = pygame.transform.rotate(image, angle)
        rotated_image_rect = rotated_image.get_rect(center = rotated_image_center)
        car_rect = rotated_image_rect
        car_image = rotated_image
        return car_rect, car_image
    
    def generate_controls_matrix(self,vertical_controls, horizontal_controls):
        controls = []
        for i in range(len(vertical_controls)):
            for j in range(len(horizontal_controls)):
                controls.append((vertical_controls[i], horizontal_controls[j]))
        return controls

    def get_neighbours(self, node):
        neighbours = []
        obstacle_hit = False
        
        for control_input in self.control_inputs:
            if control_input[0] == 0 and control_input[1] == 0:
                continue
            for obstacle in self.obstacle_map:
                car_rect,_ = self.blitRotate(self.car, ((node.x + control_input[0])*settings.pixels_per_meter, (node.y + control_input[1])*settings.pixels_per_meter), (self.car_rect.width/2,self.car_rect.height/2), 0)
                if obstacle.collide_with_car(car_rect):
                    obstacle_hit = True
                    break
            if not obstacle_hit:
                neighbours.append(Node(node.x + control_input[0], node.y + control_input[1]))
        return neighbours
    

    def goal_check(self, node, goal):
        if euclidean((node.x, node.y), goal) < 2:
            return True
        else:
            return False
        
    def present_in_list(self, node, open_list):
        if (node.x, node.y) in open_list:
            return True
        return False


    def get_path(self, start, goal):
        #A* algorithm
        #Create start and goal node
        self.control_inputs = self.generate_controls_matrix([-1,-0.5,0,0.5,1], [-1,-0.5,0,0.5,1])
        start_node = Node(start[0], start[1])
        start_node.g = 0
        start_node.h = start_node.f = euclidean(start, goal)
        goal_node = Node(goal[0], goal[1])
        goal_node.g = goal_node.h = goal_node.f = 0

        #Initialize both open and closed list
        pq = PriorityQueue()
        open_list = {}
        closed_list = {}
        path = []

        #Add the start node
        open_list[(start_node.x, start_node.y)] = start_node
        pq.put((start_node))

        #Loop until you find the end
        while len(open_list) > 0:
            #Get the current node
            current_node = pq.get()
            closed_list[(current_node.x, current_node.y)] = current_node
            del open_list[(current_node.x, current_node.y)]
            neighbours = self.get_neighbours(current_node)

            #Check if we have reached the goal
            if self.goal_check(current_node, goal):
                current = current_node
                while current is not None:
                    path.append((current.x, current.y))
                    current = current.parent
                path.reverse()
                return path                

            #Loop through the children
            for neighbour in neighbours:
                if not self.present_in_list(neighbour, open_list) and not self.present_in_list(neighbour, closed_list):
                    neighbour.g = current_node.g + euclidean((current_node.x, current_node.y), (neighbour.x, neighbour.y))
                    neighbour.h = euclidean((neighbour.x, neighbour.y), goal)
                    neighbour.f = neighbour.g + 10*neighbour.h
                    neighbour.parent = current_node
                    open_list[(neighbour.x, neighbour.y)] = neighbour
                    pq.put(neighbour)
        return path
    
    def move(self,point):
        car_rect,car_image = blitRotate(self.car, (point[0]*settings.pixels_per_meter,point[1]*settings.pixels_per_meter),(self.car.get_width()/2, self.car.get_height()/2),0)
        settings.screen.blit(car_image, car_rect)
        # draw rectangle around the image
        #pygame.draw.rect(settings.screen, (0, 0, 0), (*car_rect.topleft, *car_image.get_size()),2)
    

if __name__ == "__main__":
    settings.init()
    obstacle_map = Generate_obstacles.generate_obstacles()
    pygame.display.set_caption('Wumpus A*')
    wumpus_agent = wumpus_A_star(obstacle_map)

    start_check = False
    start = (np.random.randint(0,settings.grid_width), np.random.randint(0,settings.grid_width))
    start_check_image = pygame.image.load('truck.png')
    start_check_image = pygame.transform.scale(start_check_image, (4*settings.pixels_per_meter, 4*settings.pixels_per_meter))
    while not start_check:
        start = (np.random.randint(0,settings.grid_width), np.random.randint(0,settings.grid_width))
        start_check_rect,_ = wumpus_agent.blitRotate(start_check_image, (start[0]*settings.pixels_per_meter, start[1]*settings.pixels_per_meter), (start_check_image.get_width()/2,start_check_image.get_height()/2), 0)
        start_check = True

        for obstacle in obstacle_map:
            if obstacle.collide_with_car(start_check_rect):
                start_check = False
                break
           
    goal_check = False
    goal = (np.random.randint(0,settings.grid_width), np.random.randint(0,settings.grid_width))
    goal_check_image = pygame.image.load('truck.png')
    goal_check_image = pygame.transform.scale(goal_check_image, (4*settings.pixels_per_meter, 4*settings.pixels_per_meter))
    print("generating goal")
    while not goal_check:
        goal = (np.random.randint(0,settings.grid_width), np.random.randint(0,settings.grid_width))
        goal_check_rect,_ = wumpus_agent.blitRotate(goal_check_image, (goal[0]*settings.pixels_per_meter, goal[1]*settings.pixels_per_meter), (goal_check_image.get_width()/2,goal_check_image.get_height()/2), 0)
        goal_check = True
        for obstacle in obstacle_map:
            if obstacle.collide_with_car(goal_check_rect):
                goal_check = False
                break
           

    print("start: ", start)
    print("goal: ", goal)
    
    print("generating path")

    time.sleep(3)

    

    path = wumpus_agent.get_path(start, goal)

    print("path generated")

    for tetromino in obstacle_map:
        tetromino.draw(settings.screen)

    for i in range(len(path)-1):
        pygame.draw.line(settings.screen, (0,255,0), (path[i][0]*settings.pixels_per_meter, path[i][1]*settings.pixels_per_meter), (path[i+1][0]*settings.pixels_per_meter, path[i+1][1]*settings.pixels_per_meter), 4)

    pygame.draw.rect(settings.screen, (0,0,255), start_check_rect)
    pygame.draw.rect(settings.screen, (255,0,255), goal_check_rect)

    pygame.display.flip()
    while True:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                pygame.quit()

