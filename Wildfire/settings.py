import pygame

def init():
    global pixels_per_meter
    global grid_width
    global grid_height
    global coverage 
    global obstacle_height 
    global  obstacle_width 
    global screen
    global car_length
    global car_width
    global car_wheel_base
    global safety_factor
    global max_steer_angle
    global sim_time
    global wumpus_step_size
    global wumpus_size
    global max_car_speed
    global gaussian_dis_probability
    global time_limit
    global wumpus_speed_limiter
    global time_to_burn
    global time_to_extinguish

    pixels_per_meter = 4 # 4 pixels per meter
    time_to_extinguish = 5 # 5 seconds
    grid_width = 250 # 250 meters
    grid_height = 250 # 250 meters
    coverage = 0.1 # 10% coverage
    time_to_burn = 300 # 300 seconds
    obstacle_height = 5 # 5 meter
    obstacle_width = 5 # 5 meter
    car_length = 4.9 # 4.9 meters
    car_width = 2.2 # 2.2 meters
    car_wheel_base = 3 # 3 meters
    safety_factor = 1.25 # 1.1 meters
    max_steer_angle = 13 # 40 degrees
    sim_time = 0.1 # 0.1 seconds #### DONOT CHANGE THIS
    wumpus_step_size = 0.5 # wumpus moves 0.1 steps per iteration (Note, don't keep more than one decimal) 
    wumpus_size = 1 # wumpus is 0.5 meters in radius
    max_car_speed = 10 # 10 meter per second
    wumpus_speed_limiter = 2 # Number of loops wumpus waits before moving, corresponds to 0.5 seconds if value is 5 
    time_limit = 0.2 # 0.2 seconds
    gaussian_dis_probability = 0.2 # 10% probability of choosing gaussian distribution
    pygame.init()
    screen = pygame.display.set_mode((grid_width*pixels_per_meter, grid_height*pixels_per_meter))
    screen.fill((255, 255, 255))