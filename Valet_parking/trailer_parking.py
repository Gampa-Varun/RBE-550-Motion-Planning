import numpy as np
from queue import PriorityQueue
import time
import matplotlib.pyplot as plt
import reeds_shepp
import pygame
from pygame.math import Vector2


L_1 = 3
W_1 = 1.75

M = 0.1

L_2 = 4.9
W_2 = 1.75


def cost_to_go(test_state,goal_state):

	x_trailer = test_state[0] - (M + L_2)*np.cos(test_state[3]) 
	y_trailer = test_state[1] - (M + L_2)*np.sin(test_state[3]) 

	x_trailer_goal = goal_state[0] - (M + L_2)*np.cos(goal_state[3])
	y_trailer_goal = goal_state[1] - (M + L_2)*np.sin(goal_state[3])

	return   1*reeds_shepp.path_length(np.array([test_state[0],test_state[1],(test_state[2])*np.pi/180]), np.array([goal_state[0], goal_state[1],(goal_state[2])*np.pi/180]),3/np.sqrt(3))  + 1.5*reeds_shepp.path_length(np.array([x_trailer,y_trailer,(test_state[3])*np.pi/180]), np.array([x_trailer_goal, y_trailer_goal, (goal_state[3])*np.pi/180]),5/np.sqrt(3)) + 0.5*(min(np.absolute(test_state[3])*np.pi/180, 2*np.pi - np.absolute(test_state[3])*np.pi/180))


def cost_to_come(test_state,goal_state):

	x_trailer = test_state[0] - (M + L_2)*np.cos(test_state[3]) 
	y_trailer = test_state[1] - (M + L_2)*np.sin(test_state[3])

	x_trailer_goal = goal_state[0] - (M + L_2)*np.cos(goal_state[3])
	y_trailer_goal = goal_state[1] - (M + L_2)*np.sin(goal_state[3])
	
	return 1*reeds_shepp.path_length(np.array([test_state[0],test_state[1],test_state[2]*np.pi/180]), np.array([goal_state[0], goal_state[1],goal_state[2]*np.pi/180]),3/np.sqrt(3)) + 1.5*reeds_shepp.path_length(np.array([x_trailer,y_trailer,(test_state[3])*np.pi/180]), np.array([x_trailer_goal, y_trailer_goal, (goal_state[3])*np.pi/180]),5/np.sqrt(3)) 


class Node:
	def __init__(self, state):
		self.vertex = state
		self.parent = None
		self.control_required = [0,0]
		self.net_cost = 0
		goal = np.asarray([20,20,0,0,0])
		self.heurestic = cost_to_go(self.vertex,goal)
		self.total_cost = 0

	def __lt__(self, other):
		goal = np.asarray([20,20,0,0,0])
		init_state = np.asarray([0,0,0,0,0])

		return self.total_cost < other.total_cost


def goal_check(test_state,goal_state):
	distance_goal = np.sqrt((test_state[0] - goal_state[0])**2 + (test_state[1] - goal_state[1])**2) + 3*(min(np.absolute(test_state[2])*np.pi/180, 2*np.pi - np.absolute(test_state[2])*np.pi/180)) + 10*(min(np.absolute(test_state[3])*np.pi/180, 2*np.pi - np.absolute(test_state[3])*np.pi/180))  #+ 0.1 * np.sqrt((test_state[2] - goal_state[2])**2 ) + 0.1*np.sqrt((test_state[3] - goal_state[3])**2 )

	if (distance_goal < 3):
		return True
	else :
		return False


def present_in(visited,next_node):

	goal_state = np.asarray([20,20,0,0,0])

	string_node = str(int(next_node.vertex[0])) + ','+str(int(next_node.vertex[1])) +','+str(int(next_node.vertex[2]/10)) + ',' + str((next_node.vertex[3]))

	if string_node in visited:
		return True


	if(next_node.vertex[0]< 0 or next_node.vertex[0] > 120):
		return True

	if(next_node.vertex[1]<0 or next_node.vertex[1] > 120):
		return True
	
	return False


def generate_controls_matrix(velocity_controls, steering_controls):
	control_inputs_velocity, control_inputs_steering = np.meshgrid(velocity_controls, steering_controls, indexing='ij')

	control_inputs_velocity = np.ndarray.flatten(control_inputs_velocity)

	control_inputs_steering = np.ndarray.flatten(control_inputs_steering)

	control_inputs = list(zip(control_inputs_velocity,control_inputs_steering))

	control_inputs = np.array(control_inputs)

	return control_inputs


def blitRotate(surf, image, pos, originPos, angle):
    image_rect = image.get_rect(topleft = (pos[0] - originPos[0], pos[1]-originPos[1]))
    offset_center_to_pivot = pygame.math.Vector2(pos) - image_rect.center
    rotated_offset = offset_center_to_pivot.rotate(-angle)
    rotated_image_center = (pos[0] - rotated_offset.x, pos[1] - rotated_offset.y)
    rotated_image = pygame.transform.rotate(image, angle)
    rotated_image_rect = rotated_image.get_rect(center = rotated_image_center)
    surf.blit(rotated_image, rotated_image_rect)


class Game:
	def __init__(self):
		pygame.init()
		pygame.display.set_caption("Car tutorial")
		width = 720
		height = 720
		self.screen = pygame.display.set_mode((width, height))
		self.clock = pygame.time.Clock()
		self.ticks = 60
		self.exit = False
		self.controls = []

		self.x = 0
		self.y = 0
		self.theta = 0
		self.beta = 0	

		self.L_1 = 3
		self.W_1 = 1.75

		self.M = 0.1

		self.L_2 = 4.9
		self.W_2 = 1.75

		self.car_image = pygame.image.load("truck.png")
		self.trailer_image = pygame.image.load("truck.png")

		self.car_image = pygame.transform.scale(self.car_image,(3*24,self.W_1*24))
		self.trailer_image = pygame.transform.scale(self.trailer_image,(5*24,self.W_2*24))

		self.rect_car = self.car_image.get_rect()
		self.rect_trailer = self.trailer_image.get_rect()

		self.sim_time = 0.5

		self.obstacle = pygame.Rect(290, 152, 240, 150)

		self.obstacle_draw = pygame.Rect(324, 152, 240, 150)

		self.car1 = pygame.Rect(160, 470, 2.8*48,1.5*48)

		self.car1_draw = pygame.Rect(150, 490, 2.8*36,1.5*36)

		self.car2 = pygame.Rect(590, 470, 2.8*48,1.5*48)

		self.car2_draw = pygame.Rect(600, 490, 2.8*36,1.5*36)

		self.block = pygame.Rect(0, 600, 720,1.5*36)

		self.block_draw =  pygame.Rect(0, 600, 720,1.5*36)


	def simulate_forward(self, time, init_state, control_input):

		time_steps = np.arange(0,time,0.1)

		L_1 = 3
		W_1 = 1.75

		M = 0.1

		L_2 = 5
		W_2 = 1.75

		x = init_state[0]
		y = init_state[1]
		theta = init_state[2]*np.pi/180
		beta = init_state[3]*np.pi/180

		velocity = control_input[0]

		alpha = control_input[1]

		obstacle_hit = False

		final_state = []

		for t in time_steps:

				x += velocity*np.cos(theta)*0.1
				y += velocity*np.sin(theta)*0.1
				theta += velocity*np.tan(alpha)*0.1/self.L_1
				beta += velocity*np.sin(theta - beta)*0.1/(self.M+self.L_2) 

				if theta > np.pi:
						theta = theta - 2*np.pi
				elif theta < - np.pi:
						theta = theta + 2*np.pi


				if np.minimum(np.absolute(beta-theta), 2*np.pi - np.absolute(beta-theta)) >= np.pi/3:
					obstacle_hit = True
					final_state.append(int(10*x)/10)
					final_state.append(int(10*y)/10)
					final_state.append(int(theta*180/np.pi))
					final_state.append(int(beta*180/np.pi))
					final_state.append(velocity)
					break
	

				rotated_car = pygame.transform.rotate(self.car_image, -theta*180/np.pi)
				rotated_trailer = pygame.transform.rotate(self.trailer_image, -beta*180/np.pi)
				self.rect_car = rotated_car.get_rect()
				self.rect_trailer = rotated_trailer.get_rect()
				position = Vector2(24*x, 24*y)

				self.rect_car = self.rect_car.move(position - (self.rect_car.width / 2, self.rect_car.height / 2))

				x_trailer = x - (self.M + self.L_2)*np.cos(beta) 
				y_trailer = y - (self.M + self.L_2)*np.sin(beta) 

				position_trailer = Vector2(24*x_trailer, 24*y_trailer)
				self.rect_trailer = self.rect_trailer.move(position_trailer - (self.rect_trailer.width / 2, self.rect_trailer.height / 2))

				if self.rect_car.colliderect(self.block) or self.rect_trailer.colliderect(self.block) or self.rect_car.colliderect(self.car1) or self.rect_car.colliderect(self.car2) or self.rect_car.colliderect(self.obstacle) or self.rect_trailer.colliderect(self.obstacle) or self.rect_trailer.colliderect(self.car1) or self.rect_trailer.colliderect(self.car2):
					obstacle_hit = True
					final_state.append(int(10*x)/10)
					final_state.append(int(10*y)/10)
					final_state.append(int(theta*180/np.pi))
					final_state.append(int(beta*180/np.pi))
					final_state.append(velocity)
					
					return final_state, obstacle_hit
					break
				

		final_state.append(int(10*x)/10)
		final_state.append(int(10*y)/10)
		final_state.append(int(theta*180/np.pi))
		final_state.append(int(beta*180/np.pi))
		final_state.append(velocity)


		final_state = np.array(final_state)
				
		return final_state, obstacle_hit



	def A_star(self,init_state,goal_state):

		pq = PriorityQueue()

		visited = {}
		queue = {}

		init_node = Node(init_state)

		pq.put(init_node)
		queue[str(init_node.vertex[0]) + ','+str(init_node.vertex[1]) +','+str(init_node.vertex[2]) + ',' + str(init_node.vertex[3])] = init_node

		velocity_controls = np.arange(-4.0,4.1,1)

		steering_controls = np.arange(-np.pi/3,np.pi/3,np.pi/45)

		control_inputs = generate_controls_matrix(velocity_controls, steering_controls)

		goal_reached = False

		end_node = None

		start_time = time.time()

		while not pq.empty():
			s = pq.get()

			del queue[str(int(s.vertex[0])) + ','+str(int(s.vertex[1])) +','+str(int(s.vertex[2]/10)) + ',' + str(s.vertex[3])]
			visited[str(int(s.vertex[0])) + ','+str(int(s.vertex[1])) +','+str(int(s.vertex[2]/10)) + ',' + str(s.vertex[3])] = s

			

			sim_time = self.sim_time


			for control_input in control_inputs:

				next_state,obstacle_hit = self.simulate_forward(sim_time,s.vertex,control_input)
				

				next_node = Node(next_state)
				next_node.parent = s
				next_node.control_required = control_input
				next_node.net_cost = next_node.parent.net_cost + 1*cost_to_come(next_node.vertex,next_node.parent.vertex)
				next_node.total_cost = next_node.net_cost + 5*next_node.heurestic


				if(goal_check(next_state,goal_state)):
					goal_reached = True
					end_node = next_node
					break

				if not present_in(visited,next_node) and not present_in(queue,next_node) and not obstacle_hit:
					
					queue[str(int(next_node.vertex[0])) + ','+str(int(next_node.vertex[1])) +','+str(int(next_node.vertex[2]/10)) + ',' + str(next_node.vertex[3])] = next_node
					pq.put(next_node)

			
			if goal_reached:
				break


		end_time = time.time()

		print("total time taken this loop: ", end_time - start_time)

		path = []
		control_law = []

		while end_node != None:
			path.append(end_node.vertex)
			control_law.append(end_node.control_required)
			end_node = end_node.parent


		print(path)

		return path, control_law


	def run(self):

		pygame.time.wait(2000)
		ppu = 1


		for control in reversed(self.controls):
			sim_time = self.sim_time

			
			# Event queue
			for event in pygame.event.get():
				if event.type == pygame.QUIT:
					self.exit = True

			if self.exit:
				break

			# Logic
			velocity = control[0]

			alpha = control[1]

			time_steps = np.arange(0,sim_time,0.1)


			for t in time_steps:

				t_start = pygame.time.get_ticks()

				self.x += velocity*np.cos(self.theta)*0.1
				self.y += velocity*np.sin(self.theta)*0.1

				self.theta += velocity*np.tan(alpha)*0.1/self.L_1
				self.beta += velocity*np.sin( self.theta - self.beta )*0.1/(self.M+self.L_2) 

				if self.theta > np.pi:
						self.theta = self.theta - 2*np.pi
				elif self.theta < - np.pi:
						self.theta = self.theta + 2*np.pi

				if self.beta > np.pi:
						self.beta = self.beta - 2*np.pi
				elif self.beta < - np.pi:
						self.beta = self.beta + 2*np.pi

			# Drawing
				self.screen.fill((255,255,255))
				BLACK = (0, 0, 0)

				pygame.draw.rect(self.screen, BLACK, self.obstacle_draw)
				pygame.draw.rect(self.screen, BLACK, self.car1_draw)
				pygame.draw.rect(self.screen, BLACK, self.car2_draw)
				pygame.draw.rect(self.screen, BLACK, self.block_draw)

				x_car = self.x 
				y_car = self.y 

				block_check = pygame.Rect(24*x_car, 24*y_car, 10,10)

				pygame.draw.rect(self.screen, (255,0,0), block_check)
				

				position = Vector2(24*self.x, 24*self.y)


				x_trailer = self.x - (self.M + self.L_2)*np.cos(self.beta) 
				y_trailer = self.y - (self.M + self.L_2)*np.sin(self.beta)

				block_check_2 = pygame.Rect(24*x_trailer, 24*y_trailer, 10,10)
				pygame.draw.rect(self.screen, (255,255,0), block_check_2)

				position_trailer = Vector2(24*x_trailer, 24*y_trailer)


				blitRotate(self.screen,self.car_image,(24*self.x,24*self.y),(0,24*self.W_1/2),-self.theta*180/np.pi)

				blitRotate(self.screen,self.trailer_image,(24*x_trailer,24*y_trailer),(0,24*self.W_2/2),-self.beta*180/np.pi)

				#print("self.x self.y, x, y ", x_car, y_car, x_trailer, y_trailer, self.theta, self.beta)
				#print(np.sqrt((x_car-x_trailer)**2 + (y_car-y_trailer)**2))
				#print(np.arctan2((y_car-y_trailer),(x_car-x_trailer)))


				self.rect_car = self.rect_car.move(position )
				self.rect_trailer = self.rect_trailer.move(position_trailer)
			
				pygame.display.flip()

				self.clock.tick(self.ticks)

				t_end = pygame.time.get_ticks()

				pygame.time.wait(100-t_end+t_start)

		print("final_position: ",self.x,self.y,self.theta,self.beta)

		pygame.time.wait(10000)

		pygame.quit()



if __name__ == '__main__':
	game = Game()
	path, control_law = game.A_star([0,0,0,0,0],[20,20,0,0,0]) # Start position and end position
	print(control_law)

	x = []
	y = []

	for state in reversed(path):
		x.append(state[0])
		y.append(state[1])


	plt.plot(x,y)
	plt.show()

	
	game.controls =  control_law
	game.run()








