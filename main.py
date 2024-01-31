
import numpy as np
import math
import matplotlib.pyplot as plt
import networkx as nx
import random as rn
from PIL import Image

OG_array = np.array(Image.open('occupancy_map.png')).astype(int)

class A_STAR():
    def __init__(self):
        # Variables for the class
        self.occupancy_grid = OG_array

    def N(self,v):
        # Define 8 neighbours
        n1 = (v[0]+1,v[1])
        n2 = (v[0]+1,v[1]+1)
        n3 = (v[0],v[1]+1)
        n4 = (v[0]-1,v[1]+1)
        n5 = (v[0]-1,v[1])
        n6 = (v[0]-1,v[1]-1)
        n7 = (v[0],v[1]-1)
        n8 = (v[0]+1,v[1]-1)

        L = [n1,n2,n3,n4,n5,n6,n7,n8]
        L1 = []
        for x in L:
            if(x[0]>=0 or x[1]>=0 or x[0]<680 or x[1]<623):     # Appending the vertex if the vertex is not out of bounds
                if(self.occupancy_grid[x[0]][x[1]] == 255):     # Appending the vertex if the vertex is not occupied
                    L1.append(x)                                # Appending the vertex to the list
        return L1


    def d(self,v1,v2):
         return math.dist(v1,v2)

    def w(self,v1,v2):
        return self.d(v1,v2)

    def h(self,v1,v2):
        return self.d(v1,v2)

    def RecoverPath(self,pred,s,g):
        # Define empty list
        L1 = []

        # Append the goal vertex to the list
        L1.append(g)
        val = g

        # Loop for back tracking the Path from G to S using pred
        while(pred[val] != s):
            L1.append(pred[val])
            val = pred[val]

        # Append the start vertex to the list
        L1.append(s)

        # Reversing the list to get path from s to g
        L1.reverse()
        return L1 

    def get_vertex_set(self):
        V = []
        for i in range(0,680):
            for j in range(0,623):
                if(self.occupancy_grid[i,j] == 255):
                    V.append((i,j))
        return V 

    def A_STAR_SEARCH(self,V,s,g,N,w,h):
        null_path = []

        # Defining empty dictionaries for storing 
        CostTo = {}
        EstTotalCost = {}
        pred ={}

        # Defining priority Queue in the form of a dictionary
        Q = {}
        
        # Defining cost of every vertex in the Vertex set (V) as INF
        for vertex in V:
            CostTo[vertex] = math.inf
            EstTotalCost[vertex] = math.inf

        # Defining Cost to Start from Start is 0
        CostTo[s] = 0

        # Defining Total cost to goal as the heuristic distance
        EstTotalCost[s] = h(s,g)

        # Entering the Start Vertex as first element in the list 
        Q[s] = h(s,g)
        
        while(Q):
            v = list(Q.keys())[0]                                           # Storing the smallest element in Q as v
            Q.pop(list(Q.keys())[0])                                        # Poping the smallest element in the Q
            if v == g:                              
                return self.RecoverPath(pred,s,g)                           # Returning o/p of RecoverPath if v is equal to g
            
            for i in N(v):                                                  # Traverse every neighbour of the the current vertex v
                pvi = CostTo[v] + w(v,i)                                    # Updating distance of neighbour from start
                    
                if pvi < CostTo[i]:                                         # If path to i is better than previous known path to i
                    pred[i] = v                 
                    CostTo[i] = pvi                                         # Update cost of best path to i
                    EstTotalCost[i] = pvi + h(i,g)                  
                    Q[i] = EstTotalCost[i]                                  # Update Q's priority or add element to Q
                    Q = dict(sorted(Q.items(), key=lambda item: item[1]))   # Sort the dictionary according to values
        
        #return null path if no path exists
        return null_path

    def plot(self,path,length):

        #Plot Path
        X = []
        Y = []
        for i in path:
            X.append(i[1])
            Y.append(i[0])

        plt.plot(X,Y,'-r')
        plt.scatter(X[0],Y[0],color='green')
        plt.scatter(X[-1],Y[-1],color='green')
        plt.imshow(self.occupancy_grid)
        plt.title("Shortest Path using A*")
        plt.show()

    def get_path_length(self,path):
        path_length = 0
        for i in range(1,len(path)):
            path_length = path_length + self.d(path[i],path[i-1])
        return path_length

class PROBABILISTIC_ROAD_MAP():
    def __init__(self):
        self.occupancy_grid = OG_array
        self.G = nx.Graph()

    def sample_new_point(self):
        while(True):
            # Sample new point
            new_point = (rn.randint(0,self.occupancy_grid.shape[0]-1), rn.randint(0,self.occupancy_grid.shape[1]-1))

            # Return newly sampled point if it lies in unoccupied space and is not already present as a node
            if self.occupancy_grid [new_point[0]] [new_point[1]] == 255 and new_point not in self.G.nodes():    
                return new_point

    def reachability_check(self,v1,v2):
        x1, y1 = v1[0],v1[1]
        x2, y2 = v2[0],v2[1]

        # If Slope is ND
        if x1 == x2:
            if y2>y1:
                for y in range(y1,y2+1):
                    x = 0
                    if (self.occupancy_grid[x][y] == 0):
                        return 0

            elif y2<y1:
                for y in range(y2,y1+1):
                    x = 0
                    if (self.occupancy_grid[x][y] == 0):
                        return 0
        else:
            slope = (y2 - y1)/(x2 - x1)

            # If line is tilted towards y-axis
            if slope>1 or slope<-1:
                if y2 > y1:
                    for y in range(y1,y2+1):
                        x = ((y - y1)/slope) + x1
                        if (self.occupancy_grid[int(x)][y] == 0):
                            return 0
                else:
                    for y in range(y2,y1+1):
                        x = ((y - y1)/slope) + x1
                        if (self.occupancy_grid[int(x)][y] == 0):
                            return 0

            # If line is tilted towards x-axis
            else:
                if x2 > x1:
                    for x in range(x1,x2+1):
                        y = (slope * (x - x1)) + y1
                        if (self.occupancy_grid[x][int(y)] == 0):
                            return 0
                else:
                    for x in range(x2,x1+1):
                        y = (slope * (x - x1)) + y1
                        if (self.occupancy_grid[x][int(y)] == 0):
                            return 0
        return 1  

    def d(self,v1,v2):
        return math.dist(v1,v2)
    
    def add_vertex(self,v_new,d_max):
        self.G.add_node(v_new)                                          # Add new node to G
        for v in self.G.nodes():
            if v!=v_new and self.d(v,v_new) <= d_max:                   
                if(self.reachability_check(v,v_new)):                   # Check if node is reachable from v
                    self.G.add_edge(v,v_new,weight = self.d(v,v_new))   # Add edge

    def construct_prm(self,N,d_max):

        while(self.G.number_of_nodes() < N):
            v_new = self.sample_new_point()     
            self.add_vertex(v_new,d_max) 

        return self.G
    
    def plot(self,path):

        #plog map
        pos = {}
        for i in self.G.nodes():
            pos[i] = (i[1],i[0])
        nx.draw_networkx(self.G,pos=pos,node_size=1.5,with_labels=0,width=0.2,node_color='blue',edge_color='orange')
        plt.title("Probabilistic Road Map | N = 2500 | d_max = 75")
        plt.imshow(self.occupancy_grid)
        plt.show()

        #Plot Path
        X = []
        Y = []
        for i in path:
            X.append(i[1])
            Y.append(i[0])
            plt.scatter(i[1],i[0],color='red',s=10)
        path_len = self.get_path_length(path)
        plt.title("Shortest Path from s = (635,140) to g = (350,400) | Path Length: " + str(int(path_len)))
        plt.imshow(self.occupancy_grid)
        plt.plot(X,Y,'-r')
        plt.show()

    def a_star_search(self,s,g):
        return nx.astar_path(self.G,s,g)

    def get_path_length(self,path):
        path_length = 0
        for i in range(1,len(path)):
            path_length = path_length + self.d(path[i],path[i-1])
        return path_length


s = (635,140)
g = (350,400)

N = 2500
d_max = 75

# Implement A_Star Search 
a_star = A_STAR()
V = a_star.get_vertex_set()
path = a_star.A_STAR_SEARCH(V,s,g,a_star.N,a_star.w,a_star.h)
a_star.plot(path,a_star.get_path_length(path))


# Implement path search using PRM
prm = PROBABILISTIC_ROAD_MAP()
prm.construct_prm(N,d_max)

prm.add_vertex(g,d_max)
prm.add_vertex(s,d_max)

path = prm.a_star_search(s,g)
prm.plot(path)


