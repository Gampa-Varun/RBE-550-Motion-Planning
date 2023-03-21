import numpy as np 
import matplotlib.pyplot as plt



class AdjNode: # Defining the node, which contains info of vertex number, it's siblings (unless it is the parent node), and it's parent
    def __init__(self, data):
        self.vertex = data
        self.next = None
        self.parent = None

    def __lt__(self, other):
        return self.vertex < other.vertex
 
 
# A class to represent a graph. A graph
# is the list of the adjacency lists.
# Size of the array will be the no. of the
# vertices "V"
class Graph:
    def __init__(self, vertices):
        self.V = vertices
        self.graph = [None] * self.V
 
    # Function to add an edge in an undirected graph
    def add_edge(self, src, dest):
        # Adding the node to the source node

        node_dest = AdjNode(dest)

        if self.graph[src] is not None:
            head = self.graph[src]
            while head.next:
                head = head.next

            head.next = node_dest

        else:

            node = AdjNode(src)
            node_dest = AdjNode(dest)
            node.next = node_dest
            self.graph[src] = node

 
 
    # Function to print the graph
    def print_graph(self):
        for i in range(self.V):

            print("Adjacency list of vertex " + str(int(i/128))+','+str(int(i%128))+'\n',end="")
            temp = self.graph[i]
            while temp:
                print(" -> "+ str(int(temp.vertex/128))+','+str(int(temp.vertex%128)), end="")
                temp = temp.next
            print(" \n")






grid = np.zeros((128,128))

shapes = {}

shapes[0] = np.asarray([(0,0),(0,1),(0,2),(0,3)])
shapes[1] = np.asarray([(0,0),(0,1),(1,1),(1,0)])
shapes[2] = np.asarray([(0,0),(1,0),(1,1),(2,0)])
shapes[3] = np.asarray([(0,0),(0,1),(0,2),(1,0)])
shapes[4] = np.asarray([(0,0),(1,0),(1,1),(2,1)])

Percentage_coverage = 10

num_box_fill_limit = Percentage_coverage*16384/100

print(f"Percent of grid to be filled:", Percentage_coverage)
print(f"Number of grid cells to be filled:", num_box_fill_limit)

num_box_filled= 0

while(num_box_filled<num_box_fill_limit):

    tetromino_shape = np.random.choice(np.arange(0,5))
    tetromino_reflect = np.random.choice(np.arange(0,2))
    tetromino_flip = np.random.choice(np.arange(0,2))
    anchorx = np.random.choice(np.arange(2,126))
    anchory = np.random.choice(np.arange(2,126))
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


dim = np.shape(grid)

vertices = 128*128


flat_land_graph = Graph(vertices)

def surrounding_vertices(grid_row,grid_col):
    adjacent_nodes = []

    if (grid_row>0 and grid_col>0 and grid_row< 127 and grid_col<127):
        adjacent_nodes = [(grid_row-1)*128+grid_col+1, grid_row*128+grid_col+1, (grid_row+1)*128+grid_col+1, (grid_row-1)*128+grid_col, (grid_row+1)*128+grid_col, (grid_row-1)*128+grid_col-1, (grid_row)*128+grid_col-1, (grid_row+1)*128+grid_col-1] 

    elif(grid_row == 0):
        if(grid_col>0 and grid_col < 127):
            adjacent_nodes = [grid_row*128+grid_col+1,(grid_row+1)*128+grid_col+1, (grid_row+1)*128+grid_col, (grid_row)*128+grid_col-1, (grid_row+1)*128+grid_col-1]
        elif(grid_col == 0):
            adjacent_nodes = [grid_row*128+grid_col+1,(grid_row+1)*128+grid_col+1, (grid_row+1)*128+grid_col]
        elif(grid_col == 127):
            adjacent_nodes = [(grid_row+1)*128+grid_col,(grid_row)*128+grid_col-1,(grid_row+1)*128+grid_col-1]

    elif(grid_row == 127):
        if(grid_col>0 and grid_col < 127):
            adjacent_nodes = [(grid_row-1)*128+grid_col+1, grid_row*128+grid_col+1, (grid_row-1)*128+grid_col, (grid_row-1)*128+grid_col-1,(grid_row)*128+grid_col-1]
        elif(grid_col == 0):
            adjacent_nodes = [(grid_row-1)*128+grid_col+1,grid_row*128+grid_col+1, (grid_row-1)*128+grid_col]
        elif(grid_col == 127):
            adjacent_nodes = [(grid_row-1)*128+grid_col, (grid_row-1)*128+grid_col-1,(grid_row)*128+grid_col-1]

    elif(grid_col==0):
        if(grid_row>0 and grid_row<127):
            adjacent_nodes = [(grid_row-1)*128+grid_col+1,grid_row*128+grid_col+1,(grid_row+1)*128+grid_col+1, (grid_row-1)*128+grid_col,(grid_row+1)*128+grid_col]

    elif(grid_col ==127):
        if(grid_row>0 and grid_row<127):
            adjacent_nodes = [(grid_row-1)*128+grid_col,(grid_row+1)*128+grid_col, (grid_row-1)*128+grid_col-1,(grid_row-1)*128+grid_col-1,(grid_row+1)*128+grid_col-1]


    return adjacent_nodes




def Path(graph,src,end):

    parent = graph.graph[end].parent
    path = []
    path.append(end)
    i = 0
    while parent is not None:
        i+=1
        path.append(parent.vertex)
        parent = graph.graph[parent.vertex].parent


    return path





def BFS(graph, src,dst):
 
        # Mark all the vertices as not visited
        visited = [False] * ((graph.V))
 
        # Create a queue for BFS
        queue = []
 
        # Mark the source node as
        # visited and enqueue it
        queue.append(graph.graph[src])
        visited[src] = True
        #print(graph.graph[src].vertex)

        reached_end = False

        iterations = 0
        
        while queue:
 
            # Dequeue a vertex from
            # queue . First in first out scheme
            s = queue[0]
 
            # Get all adjacent vertices of the
            # dequeued vertex s. If a adjacent
            # has not been visited, then mark it
            # visited and enqueue it

            temp = graph.graph[s.vertex].next
            while temp:
                if visited[temp.vertex] == False:
                    iterations+=1
                    queue.append(temp)
                    visited[temp.vertex] = True
                    graph.graph[temp.vertex].parent = s
                    if temp.vertex == dst:
                        print(temp)
                        reached_end = True
                        break

                temp = temp.next



            if reached_end:
                break

            del queue[0]

        if reached_end:
            print("reached")
        else:
            print("couldn't reach goal")

        path = Path(graph,0,127*129)
        return path,iterations


def DFS(graph, src,dst):
 
        # Mark all the vertices as not visited
        visited = [False] * ((graph.V))
 
        # Create a queue for BFS
        queue = []
 
        # Mark the source node as
        # visited and enqueue it
        queue.append(graph.graph[src])
        visited[src] = True


        reached_end = False
        iterations = 0
        
        while queue:
 
            # Dequeue a vertex from
            # queue and print it. Deque like a stack
            s = queue.pop()

 
            # Get all adjacent vertices of the
            # dequeued vertex s. If a adjacent
            # has not been visited, then mark it
            # visited and enqueue it

            temp = graph.graph[s.vertex].next
            while temp:
                if visited[temp.vertex] == False:
                    iterations +=1
                    queue.append(temp)
                    visited[temp.vertex] = True
                    graph.graph[temp.vertex].parent = s
                    if temp.vertex == dst:
                        print(temp)
                        reached_end = True
                        break

                temp = temp.next


     

            if reached_end:
                break

        if reached_end:
            print("reached")
        else:
            print("couldn't reach goal")

        path = Path(graph,0,127*129)
        return path,iterations



def weight(a,b):
    ax = int(a/128)
    ay = int(a%128)

    bx = int(b/128)
    by = int(b%128)

    distance = np.sqrt((ax-bx)*(ax-bx) + (ay-by)*(ay-by))

    return distance




from queue import PriorityQueue

def Dijkstra(graph, src, dst):
 
        # Mark all the vertices as not visited
        visited = [False] * ((graph.V))

        dist = [float('inf')] * graph.V

        pq = PriorityQueue() # Priority queue used for optimisation

        dist[src] = 0
 

 
        # Mark the source node as
        # visited and enqueue it

        pq.put((dist[src],graph.graph[src]))
        visited[src] = True

        reached_end = False
        iterations = 0
        
        while not pq.empty():
 
            # Dequeue a vertex from
            # queue and print it. Pop the node which has the least distance and has not yet been searched
            s= pq.get()        
            s= s[1]

            temp = graph.graph[s.vertex].next

            while temp:

                if dist[temp.vertex] > dist[s.vertex] + weight(temp.vertex,s.vertex):
                    dist[temp.vertex] = dist[s.vertex] + weight(temp.vertex,s.vertex)
                    graph.graph[temp.vertex].parent = s
                    pq.put((dist[temp.vertex],temp))
                    iterations+=1

                if temp.vertex == dst:
                    print(temp)
                    reached_end = True
                    break

                temp = temp.next

    

            if reached_end:
                break

        if reached_end:
            print("reached")
        else:
            print("couldn't reach goal")

        path = Path(graph,0,127*129)
        return path, iterations






def random_search(graph, src,dst):
 
        # Mark all the vertices as not visited
        visited = [False] * ((graph.V))
 
        # Create a queue for BFS
        queue = []
 
        # Mark the source node as
        # visited and enqueue it
        queue.append(graph.graph[src])
        visited[src] = True

        reached_end = False
        path = []

        random_choice = np.random.choice(np.arange(0,len(queue)))
        s = queue.pop(random_choice)
        path.append(s.vertex)

        iter_max = 10000000
        loops = 0
        
        while not reached_end :

            loops +=1
 
            # Dequeue a vertex from
            # queue and print it

            queue = []



            temp = graph.graph[s.vertex].next
            while temp:

                queue.append(temp) # Fill the possible cells the robot can visit from the current cell
                temp = temp.next


            random_choice = np.random.choice(np.arange(0,len(queue))) # Go to any of those cells randomly
            action = queue.pop(random_choice)

            path.append(action.vertex)


            s = action

            if action.vertex == dst:
                print(temp)
                reached_end = True
                break
 
            if reached_end:
                break

            if loops >= iter_max:
                break

        if reached_end:
            print("reached")
        else:
            print("couldn't reach goal")


        return path, loops




for i in range(dim[0]):
    for j in range(dim[1]):

        adjacent_nodes = surrounding_vertices(i,j)

        for adjacent_node in adjacent_nodes:

            if(grid[int(adjacent_node/128)][int(adjacent_node%128)] != 1 ):
                flat_land_graph.add_edge(i*128+j,adjacent_node)


# flat_land_graph.print_graph()

[path_random,ierations_random] = random_search(flat_land_graph,0,127*129)
[path_BFS,iterations_BFS] = BFS(flat_land_graph,0,127*129)

[path_DFS,iterations_DFS] = DFS(flat_land_graph,0,127*129)


[path_dijkstra,iterations_dijkstra] = Dijkstra(flat_land_graph,0,127*129)


path_map = np.zeros((128,128,3))

for i in path_random: #Shaded blue
  path_map[int(i/128)][int(i%128)] = [0.1,0.3,0.4]
  if grid[int(i/128)][int(i%128)] == 1:
      print("Failed")



for i in path_DFS: #Red
  path_map[int(i/128)][int(i%128)] = [1,0,0]
  if grid[int(i/128)][int(i%128)] == 1:
      print("Failed")



for i in path_BFS: # Green
  path_map[int(i/128)][int(i%128)] = [0,1,0]
  if grid[int(i/128)][int(i%128)] == 1:
      print("Failed")


for i in path_dijkstra: # Blue
    path_map[int(i/128)][int(i%128)] = [0,0,1]
    if grid[int(i/128)][int(i%128)] == 1:
        print("Failed")

grid_color = np.zeros((128,128,3))

for i in range(128):
    for j in range(128):
        if grid[i][j] == 1:
            grid_color[i][j] = [1,1,1]

path_map = path_map+grid_color

plt.imshow(path_map)
plt.show()

print(ierations_random,iterations_BFS,iterations_DFS,iterations_dijkstra)