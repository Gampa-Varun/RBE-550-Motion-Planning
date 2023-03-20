import numpy as np 
import matplotlib.pyplot as plt

grid = np.zeros((128,128))

grid = grid

shapes = {}

shapes[0] = np.asarray([(0,0),(0,1),(0,2),(0,3)])
shapes[1] = np.asarray([(0,0),(0,1),(1,1),(1,0)])
shapes[2] = np.asarray([(0,0),(1,0),(1,1),(2,0)])
shapes[3] = np.asarray([(0,0),(0,1),(0,2),(1,0)])
shapes[4] = np.asarray([(0,0),(1,0),(1,1),(2,1)])

Percentage_coverage = 70

num_box_fill_limit = Percentage_coverage*16384/100

print(f"Percent of grid to be filled:", Percentage_coverage)
print(f"Number of grid cells to be filled:", num_box_fill_limit)

num_box_filled= 0

while(num_box_filled<num_box_fill_limit):

	tetromino_shape = np.random.choice(np.arange(0,5))
	tetromino_reflect = np.random.choice(np.arange(0,2))
	tetromino_flip = np.random.choice(np.arange(0,2))
	anchorx = np.random.choice(np.arange(4,124))
	anchory = np.random.choice(np.arange(4,124))
	collision = False




	tetromino = shapes[tetromino_shape]

	if tetromino_reflect == 1:
		for i in range(0,4):
			tetromino[i][0] *=-1



	if tetromino_flip == 1:
		for i in range(0,4):
			tetromino[i][1] *=-1

	for tetromino_xy in tetromino :
		if(grid[anchory,anchorx + tetromino_xy[0]] == 1 or grid[anchory + tetromino_xy[1],anchorx] == 1):
			collision = True
			break
		else:
			pass
		

	if collision == False:
		for tetromino_xy in tetromino :
			grid[anchory-tetromino_xy[1],anchorx + tetromino_xy[0]] = 1
		num_box_filled +=4



plt.imshow(grid,cmap='gray')
plt.show()