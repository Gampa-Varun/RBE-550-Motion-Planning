import numpy as np
from queue import PriorityQueue
import time
import matplotlib.pyplot as plt
import reeds_shepp
import pygame
from pygame.math import Vector2

L = 2.8

def cost_to_go(test_state,goal_state):
	return  reeds_shepp.path_length((test_state[0],test_state[1],test_state[2]*np.pi/180), (goal_state[0],goal_state[1],goal_state[2]*np.pi/180), 2.8*np.sqrt(3)) + 1*np.exp(-((test_state[0] - 17.5)**2 + (test_state[1] - 12.5)**2)) + 1*np.exp(-((test_state[0] - 12.25)**2 + (test_state[1] - 20)**2))  + 1*np.exp(-((test_state[0] - 23)**2 + (test_state[1] - 20)**2)) 

def cost_to_come(test_state,goal_state):
	return reeds_shepp.path_length((test_state[0],test_state[1],test_state[2]*np.pi/180), (goal_state[0],goal_state[1],goal_state[2]*np.pi/180), 2.8*np.sqrt(3)) 


class Node:
	def __init__(self, state):
		self.vertex = state
		self.parent = None
		self.control_required = [0,0]
		self.net_cost = 0
		goal = np.asarray([20,20,0])
		self.heurestic = cost_to_go(self.vertex,goal)
		self.total_cost = 0

	def __lt__(self, other):
		return self.total_cost < other.total_cost


def goal_check(test_state,goal_state):

	xy_dist = (test_state[0] - goal_state[0])**2 + (test_state[1] - goal_state[1])**2

	distance = 4.5*(min(np.absolute((test_state[2] - goal_state[2])*np.pi/180),2*np.pi - np.absolute((test_state[2] - goal_state[2])*np.pi/180)) ) + 1*np.sqrt(xy_dist)

	if (distance < 1):
		return True
	else :
		return False



def present_in(visited,next_node):

	goal_state = np.asarray([20,20,0])

	string_node = str(int(next_node.vertex[0])) + ','+str(int(next_node.vertex[1])) +','+str(int(next_node.vertex[2]/2))

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

		init_state = [0,0,0]

		self.x = init_state[0]
		self.y = init_state[1]
		self.theta = init_state[2]*np.pi/180
		self.L_1 = 2.8

		self.car_image = pygame.image.load("truck.png")

		self.car_image = pygame.transform.scale(self.car_image,(2.8*36,1.5*36))

		self.rect_car = self.car_image.get_rect()

		self.sim_time = 0.5

		self.obstacle = pygame.Rect(300, 152, 240, 150)

		self.car1 = pygame.Rect(240, 470, 2.8*48,1.5*48)

		self.car1_draw = pygame.Rect(240, 490, 2.8*36,1.5*36)

		self.car2 = pygame.Rect(576, 470, 2.8*48,1.5*48)

		self.car2_draw = pygame.Rect(590, 490, 2.8*36,1.5*36)

		self.block = pygame.Rect(0, 515, 720,1.5*36)

		self.block_draw =  pygame.Rect(0, 560, 720,1.5*36)

	def simulate_forward(self, time, init_state, control_input):

			time_steps = np.arange(0,time,0.1)

			L_1 = 2.8

			x = init_state[0]
			y = init_state[1]
			theta = init_state[2]*np.pi/180
	  
			velocity = control_input[0]
			#velocity = max(-4,min(velocity,4))

			alpha = control_input[1]

			obstacle_hit = False


			for t in time_steps:

					x += velocity*np.cos(theta)*0.1
					y += velocity*np.sin(theta)*0.1

					theta += velocity*np.tan(alpha)*0.1/L_1

					if theta > np.pi:
							theta = theta - 2*np.pi
					elif theta < - np.pi:
							theta = theta + 2*np.pi


					rotated_car = pygame.transform.rotate(self.car_image, -theta*180/np.pi)
					self.rect_car = rotated_car.get_rect()
					position = Vector2(24*x, 24*y)
					self.rect_car = self.rect_car.move(position - (self.rect_car.width / 2, self.rect_car.height / 2))

					if self.rect_car.colliderect(self.obstacle) or self.rect_car.colliderect(self.car1) or self.rect_car.colliderect(self.car2) or self.rect_car.colliderect(self.block) :
						obstacle_hit = True
						final_state = []

						final_state.append(int(x))
						final_state.append(int(y))
						final_state.append(int(theta*180/np.pi))
					 

						final_state = np.array(final_state)
								
						return final_state, obstacle_hit
						break

			final_state = []

			final_state.append(int(x*10)/10)
			final_state.append(int(y*10)/10)
			final_state.append(int(theta*180/np.pi))
		 

			final_state = np.array(final_state)
					
			return final_state, obstacle_hit


	def A_star(self,init_state,goal_state):
		pq = PriorityQueue()

		visited = {}
		queue = {}

		init_node = Node(init_state)

		pq.put(init_node)
		queue[str(init_node.vertex[0]) + ','+str(init_node.vertex[1]) +','+str(init_node.vertex[2])] = init_node

		velocity_controls = np.arange(-5.0,5.1,5)
		print(velocity_controls)

		steering_controls = np.arange(-np.pi/6,np.pi/6,np.pi/45)

		control_inputs = generate_controls_matrix(velocity_controls, steering_controls)

		goal_reached = False

		end_node = None

		start_time = time.time()

		while not pq.empty():
			s = pq.get()

			del queue[str(int(s.vertex[0])) + ','+str(int(s.vertex[1])) +','+str(int(s.vertex[2]/2))]
			visited[str(int(s.vertex[0])) + ','+str(int(s.vertex[1])) +','+str(int(s.vertex[2]/2))] = s

			sim_time = self.sim_time

			for control_input in control_inputs:


				next_state, obstacle_hit = self.simulate_forward(sim_time,s.vertex,control_input)
				

				next_node = Node(next_state)
				next_node.parent = s
				next_node.control_required = control_input
				next_node.net_cost = next_node.parent.net_cost + 1*cost_to_come(next_node.vertex,next_node.parent.vertex)
				next_node.total_cost = next_node.net_cost + 3*next_node.heurestic

				if(goal_check(next_state,goal_state)) and not obstacle_hit:
					goal_reached = True
					print(next_node.vertex)
					end_node = next_node
					break

				if not present_in(visited,next_node) and not present_in(queue,next_node) and not obstacle_hit:
						queue[str(int(next_node.vertex[0])) + ','+str(int(next_node.vertex[1])) +','+str(int(next_node.vertex[2]/2))] = next_node
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
		ppu = 1

		pygame.time.wait(2000)



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

				if self.theta > np.pi:
						self.theta = self.theta - 2*np.pi
				elif self.theta < - np.pi:
						self.theta = self.theta + 2*np.pi

			# Drawing
				self.screen.fill((255,255,255))
				BLACK = (0, 0, 0)
				
				pygame.draw.rect(self.screen, BLACK, self.obstacle)
				pygame.draw.rect(self.screen, BLACK, self.car1_draw)
				pygame.draw.rect(self.screen, BLACK, self.car2_draw)
				pygame.draw.rect(self.screen, BLACK, self.block_draw)

		
				rotated_car = pygame.transform.rotate(self.car_image, -self.theta*180/np.pi)
				self.rect_car = rotated_car.get_rect()
				position = Vector2(24*self.x, 24*self.y)
				self.rect_car = self.rect_car.move(position - (self.rect_car.width / 2, self.rect_car.height / 2))
				
				self.screen.blit(rotated_car, position - (self.rect_car.width / 2, self.rect_car.height / 2))
			
				pygame.display.flip()

				self.clock.tick(self.ticks)

				t_end = pygame.time.get_ticks()

				pygame.time.wait(100-t_end+t_start)

		print("final_position: ",self.x,self.y)

		pygame.time.wait(10000)

		pygame.quit()




if __name__ == '__main__':
	game = Game()
	path, control_law = game.A_star([0,0,0],[20,20,0]) # Start position and end position
	print(control_law)

	x = []
	y = []

	for state in reversed(path):
		x.append(state[0])
		y.append(state[1])


	plt.plot(x,y)
	plt.show()

	
	game.controls = control_law
	game.run()








