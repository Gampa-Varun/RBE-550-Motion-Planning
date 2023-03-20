import numpy as np 
import matplotlib.pyplot as plt



random_search_iterations = np.log([285054,680686,704472,537088,743780,599650,141367,30974])

BFS_iterations = np.log([16219,14771,13212,11682,10229,8826,4133,2795])

DFS_iterations = np.log([507,508,7177,7368,6934,822,506,505])

Dijkstra_iterations = np.log([17569,15477,13718,11974,10411,8901,4165,2833])

density = [1,10,20,30,40,50,60,70]

plt.plot(density,random_search_iterations,color = 'cornflowerblue', label='random_search')
plt.plot(density,BFS_iterations,color='green', label='BFS')
plt.plot(density,DFS_iterations,color='red', label='DFS')
plt.plot(density, Dijkstra_iterations, color='blue',label='Dijkstra')

plt.xlabel("Density")
plt.ylabel("Number of iterations in logarithmic scale")
plt.title("Number of iterations vs density for different algorithms to reach the goal")

plt.legend()

plt.show()