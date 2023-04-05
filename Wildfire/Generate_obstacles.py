import numpy as np
import settings
import pygame


#obstacle class
class Obstacle:
    def __init__(self, x, y, width, height,type):
        self.x = [x,x,x,x]
        self.y = [y,y,y,y]
        self.width = width
        self.height = height
        self.status = 0
        self.time_left_to_live = np.float32('inf')
        self.type = type
        self.extinguished = False
        
        # |
        # |
        # |
        # | 

        if type == 0:
            self.x[1] = self.x[0] 
            self.y[1] = self.y[0] + self.height
            self.x[2] = self.x[0] 
            self.y[2] = self.y[0] + 2*self.height
            self.x[3] = self.x[0]
            self.y[3] = self.y[0] + 3*self.height

        # | |
        #   |
        #   |

        elif type == 1:
            self.x[1] = self.x[0] + self.width
            self.y[1] = self.y[0]
            self.x[2] = self.x[0] + self.width
            self.y[2] = self.y[0] + self.height
            self.x[3] = self.x[0] + self.width
            self.y[3] = self.y[0] + 2*self.height

        # |
        # | |
        #   |

        elif type == 2:
            self.x[1] = self.x[0]
            self.y[1] = self.y[0] + self.height
            self.x[2] = self.x[0] + self.width
            self.y[2] = self.y[0] + self.height
            self.x[3] = self.x[0] + self.width
            self.y[3] = self.y[0] + 2*self.height

        #   |
        # | |
        #   |

        elif type == 3:
            self.x[1] = self.x[0] + self.width
            self.y[1] = self.y[0] - self.height
            self.x[2] = self.x[0] + self.width
            self.y[2] = self.y[0]
            self.x[3] = self.x[0] + self.width
            self.y[3] = self.y[0] + self.height

        self.obstacle1 = pygame.Rect(int(settings.pixels_per_meter*(self.x[0] -self.width/2)), int(settings.pixels_per_meter*(self.y[0] -self.height/2)), settings.pixels_per_meter*self.width, settings.pixels_per_meter*self.height)
        self.obstacle2 = pygame.Rect(int(settings.pixels_per_meter*(self.x[1]-self.width/2)), int(settings.pixels_per_meter*(self.y[1]-self.height/2)), settings.pixels_per_meter*self.width, settings.pixels_per_meter*self.height)
        self.obstacle3 = pygame.Rect(int(settings.pixels_per_meter*(self.x[2]-self.width/2)), int(settings.pixels_per_meter*(self.y[2]-self.height/2)), settings.pixels_per_meter*self.width, settings.pixels_per_meter*self.height)
        self.obstacle4 = pygame.Rect(int(settings.pixels_per_meter*(self.x[3]-self.width/2)), int(settings.pixels_per_meter*(self.y[3]-self.height/2)), settings.pixels_per_meter*self.width, settings.pixels_per_meter*self.height)

        self.tetromino = [self.obstacle1.scale_by(settings.safety_factor), self.obstacle2.scale_by(settings.safety_factor), self.obstacle3.scale_by(settings.safety_factor), self.obstacle4.scale_by(settings.safety_factor)]

    def draw(self, screen):
        if self.status == 0:
            color = (255, 0,0)
        elif self.status == 1:
            color = (0, 255, 0)
        elif self.status == 2:
            color = (0, 0, 255)
        pygame.draw.rect(screen, color, self.obstacle1)
        pygame.draw.rect(screen, color, self.obstacle2)
        pygame.draw.rect(screen, color, self.obstacle3)
        pygame.draw.rect(screen, color, self.obstacle4)

    def update(self, time_step, action=None):
        if action == 'save':
            if self.status == 1:
                self.status = 0
                self.time_left_to_live = np.float32('inf')
                self.extinguished = True

        elif action == 'burn':
            if self.status==0:
                self.status = 1
                self.time_left_to_live = settings.time_to_burn
                self.extinguished = False

        elif action == 'burn_by_wildfire':
            if self.status==0 and not self.extinguished:
                self.status = 1
                self.time_left_to_live = settings.time_to_burn
               

        else:
            if self.time_left_to_live != np.float32('inf'):
                self.time_left_to_live -= time_step
            if self.time_left_to_live <= 0:
                self.status = 2
                self.time_left_to_live = np.float32('inf')
    
    def distance_to_obstacle(self, obstacle):
        min_dist = np.float32('inf')
        for i in range(4):
            for j in range(4):
                min_dist = min(min_dist, np.sqrt((self.x[i] - obstacle.x[j])**2 + (self.y[i] - obstacle.y[j])**2))
        return min_dist

    def reset(self):
        self.status = 0
        self.time_left_to_live = np.float32('inf')

    def get_center(self):
        return self.x[0], self.y[0]

    def get_status(self):
        return self.status
    
    def get_time_left_to_live(self):
        return self.time_left_to_live
    
    def get_obstacle(self):
        return self.tetromino
    
    def tetromino_collide(self, obstacle):
        
        for i in range(4):
            for j in range(4):
                if self.tetromino[i].colliderect(obstacle.tetromino[j]):
                    return True
        return False
    
    def collide_with_obstacles(self, tetrominos):
        for obstacle in tetrominos:
            if self.tetromino_collide(obstacle):
                return True
        return False
    
    def collide_with_car(self, car):
        for i in range(4):
            if self.tetromino[i].colliderect(car):
                return True
        return False
 
#Generate Tetromino obstacles
def generate_obstacles():
    area_obstacle = 4*settings.obstacle_height*settings.obstacle_width
    num_obstacles  = int(settings.coverage*settings.grid_width*settings.grid_height/area_obstacle)
    print(num_obstacles)
    tetrominos = []
    tetromino_generated = 0
    while tetromino_generated < num_obstacles:
        x = np.random.randint(10, settings.grid_width-10)
        y = np.random.randint(10, settings.grid_height-20)
        type = np.random.randint(0, 4)
        tetromino = Obstacle(x, y, settings.obstacle_width, settings.obstacle_height, type)
        if not tetromino.collide_with_obstacles(tetrominos):
            tetrominos.append(tetromino)
            tetromino_generated += 1
        

    return tetrominos


#Generate obstacles for the grid
if __name__ == '__main__':
    settings.init()
    pygame.init()
    pygame.display.set_caption('Obstacle_test')
    screen = pygame.display.set_mode((settings.grid_width*settings.pixels_per_meter, settings.grid_height*settings.pixels_per_meter))
    screen.fill((255, 255, 255))
    obstacles = generate_obstacles()
    for tetromino in obstacles:
        tetromino.draw(screen)
    
    pygame.draw.rect(screen, (0,0,0),pygame.Rect(int(settings.pixels_per_meter*125), int(settings.pixels_per_meter*125), settings.pixels_per_meter*2.2, settings.pixels_per_meter*5))
    pygame.display.flip()
    pygame.time.wait(10000)

