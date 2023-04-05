import numpy as np
import matplotlib.pyplot as plt
import random
from math import dist
import pygame

class wildfire:
    def __init__(self, size, density):
        self.size = size
        self.density = density
        self.w = 10 #width of obstacles in pixel

    def blocks(self):   # Function for different types of obstacle
        I = [[0,3],[0,2],[0,1],[0,0]]
        L = [[0,1],[1,1],[1,2],[1,3]]
        Z = [[0,1],[0,2],[1,2],[1,3]]
        T = [[0,2],[1,2],[1,1],[1,3]]

        return [I,L,Z,T]

    def py_blocks(self,obst):
        py_block = []
        for i,j in self.w*obst:
            py_block.append(pygame.Rect(i,j,self.w,self.w))

        return py_block

    def layout(self):   # Generate 2d world with given obstacle densitty        
        tetrominoes = self.blocks()
        obstacles = []

        for k in range(int(self.density*self.size/100)):
                 
            block = np.array(random.choice(tetrominoes))
            
            m = 2
            n = 4

            x = random.randint(0,self.size-m*self.w)   # Randomly selecting x,y to place the obstacle block
            y = random.randint(0,self.size-n*self.w)

            block = self.py_blocks(block + [x/self.w,y/self.w])

            temp = []
            a = 0
            for i in block:
                if pygame.Rect.collidelist(i,obstacles) != -1:
                    temp = []
                    a = 0
                    k-=1
                    
                    break
                else:
                    a+=1
                    temp.append(i)
                if a ==4:
                    obstacles.extend(temp)
                    a = 0
                    temp = []
                    break

        return obstacles

##ppm = 3
##size = 250*ppm
##density = 10
##env = flatland(size,density)
##grid = env.layout()
##pygame.init()
##screen = pygame.display.set_mode((size, size))
##screen.fill((255,255,255))
##
##for i in grid:
##    pygame.draw.rect(screen,(0,0,0),i)
##
##pygame.display.flip()
##
