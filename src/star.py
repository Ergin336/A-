import sys
import time
import numpy as np
import matplotlib.pyplot as plt 

class node():
    """
        Class for creation of nodes with their atributes

        param: maps: debug map of previous node
        param: parent: previous node
        param: prismatic: prismatic joint of the robot
        param: final_map: desired map
        param: pallet: if the robot brings any pallet
        param: base: map containing the initial pallet positions
    """

    def __init__(self, maps=None, parent=None, prismatic=False, final_map=None, pallet=False, base=None):
        self.g = 0
        self.f = 0
        self.h = 0  #falta decidir a que pallet le quiero calcular la distancia
        self.map = maps #auxiliar map
        self.state = False #if node is solution
        self.movements = [] #list of movements and rotations
        self.parent = parent #parent
        self.pallet = pallet #if the robot has the pallet
        self.final_pallet = [] #list containing the final pose of pallets
        self.initial_pallet = [] #list containing the start pose of pallets
        self.base = np.copy(np.array(base, dtype=object)) #start map
        self.robot_position = None
        self.prismatic = prismatic  #prismatic joint
        self.req = np.copy(np.array(final_map, dtype=object)) #final map
        self.new_node = np.copy(np.array(maps, dtype=object)) #map for debugging
        self.dimensions = np.shape(np.array(maps, dtype=object)) #shape of the map

    def finalState(self):
            """
                Check if node is the solution

                param: node: node to check if it is a solution
            """

            if (self.robot_position[0:2] == self.final_pallet[0][0:2]) and self.pallet:
                if (self.robot_position[2] == "1" or self.robot_position[2] == "3") and \
                   (self.final_pallet[0][2] == 8):
                    return True

                if (self.robot_position[2] == "2" or self.robot_position[2] == "4") and \
                   (self.final_pallet[0][2] == 9):
                    return True
                
                else:
                    return False

            else:
                return False

    def get_elements_pose(self):
        """
            Gets the position of the robot and pallets
        """

        for i, r in enumerate(self.new_node): #check robot position
            for index, c in enumerate(r): 
                my_index = [i, index]      
                if c == "1" or c == "2" or c == "3" or c == "4": #if robot
                    self.robot_position = my_index
                    self.robot_position.append(c)

                if self.req[i][index] == "8" or self.req[i][index] == "9": #check final pallet position
                    my_index.append(int(self.req[i][index]))
                    if my_index in self.final_pallet:
                        pass
                        
                    else:
                        self.final_pallet.append(my_index)

        for i, r in enumerate(self.base): #check initial or actual pallet position
            for index, c in enumerate(r):
                my_index = [i, index]  
                if c == "5" or c == "6": #if initial position of pallet found
                    my_index.append(int(self.base[i][index]))
                    if my_index in self.initial_pallet:
                        pass
                        
                    else:
                        self.initial_pallet.append(my_index)
                        if self.robot_position[0] == i and self.robot_position[1] == index:
                            pass 
                        
                        else:
                            self.new_node[i][index] = str(c)
                
                
    def create_node(self, rotation):
        """
            Creates a new node

            param: rotation: Where the robot should move or rotate (up, down, left or right)
        """

        def get_h():
            """
                Function to get h of each node (euclidian distance from robot to next pallet)
            """

            if not self.pallet: #distance from robot without pallet to initial pallet
                if len(self.initial_pallet) <= 0 or len(self.initial_pallet) <= 0:
                    distances = [0]
                
                else:
                    distances = []

                for pallet in range(len(self.initial_pallet)): 
                    dist = np.sqrt(((self.robot_position[0] - self.initial_pallet[pallet][0]) ** 2) + \
                    ((self.robot_position[1] - self.initial_pallet[pallet][1]) ** 2))

                    if dist == 0: #if in initial pallet, then distance to final pallet
                        distances.append(np.sqrt(((self.robot_position[0] - self.final_pallet[pallet][0]) ** 2) + \
                        ((self.robot_position[1] - self.final_pallet[pallet][1]) ** 2)))

                    else:
                        distances.append(dist)

            elif self.pallet: #distance from robot with pallet to final position of pallet
                if len(self.final_pallet) <= 0 or len(self.final_pallet) <= 0:
                    distances = [0]
                
                else:
                    distances = []

                for pallet in self.final_pallet: 
                    distances.append(np.sqrt(((self.robot_position[0] - pallet[0]) ** 2) + \
                    ((self.robot_position[1] - pallet[1]) ** 2)))
    
            return distances
        
        self.get_elements_pose() #refresh positions of all elements
        robot_x = self.robot_position[0]
        robot_y = self.robot_position[1]
        robot_r = self.robot_position[2]
        if self.pallet:
            robot_xX = 0
            robot_yY = 0
            robot_Xx = 0
            robot_Yy = 0
            d = 1 #robot with pallet takes 3 squares
            try:
                robot_xX = self.base[self.robot_position[0] +2][robot_y]
                robot_yY = self.base[robot_x][self.robot_position[1] -d]
                robot_Xx = self.base[self.robot_position[0] -2][robot_y]
                robot_Yy = self.base[robot_x][self.robot_position[1] +d]
                progress = True
            
            except:
                progress = False

        else:
            d = 0 #robot without pallet takes 1 square
            robot_xX = 0
            robot_yY = 0
            robot_Xx = 0
            robot_Yy = 0
            progress = True
        
        if rotation == "front":
            #where to move or rotate
            front = False 
            left = False
            right = False
            back = False
            move_front = 0
            
            if robot_r == "1" and robot_x > d: #move to the front
                move_front = self.base[robot_x -1][robot_y]
                front = True

            elif robot_r == "2" and robot_y > d: #move to the left
                move_front = self.base[robot_x][robot_y -1]
                left = True

            elif robot_r == "3" and robot_x < (self.dimensions[0] -1-d): #move to the back
                move_front = self.base[robot_x +1][robot_y]
                back = True

            elif robot_r == "4" and robot_y < (self.dimensions[1] -1-d): #move to the right
                move_front = self.base[robot_x][robot_y +1]
                right = True
           
            if move_front != "7" and robot_Xx != "7" and robot_xX != "7" and robot_Yy != "7" \
               and robot_yY != "7" and progress: #if no obstacle
                if (move_front == "6" and (robot_r == "1" or robot_r == "3")) or \
                    (move_front == "5" and (robot_r == "2" or robot_r == "4")): #pallet unreachable
                    pass #if robot crash against pallet
                
                elif (move_front == 5 or move_front == 6 ) and self.prismatic: #prismatic joint up 
                    pass 
                
                else:
                    if front: #move front
                        aux = self.new_node[robot_x][robot_y]
                        self.new_node[robot_x - 1][robot_y] = str(aux)

                    elif left: #move left
                        aux = self.new_node[robot_x][robot_y]
                        self.new_node[robot_x][robot_y -1] = str(aux)
                
                    elif right: #move right
                        aux = self.new_node[robot_x][robot_y]
                        self.new_node[robot_x][robot_y +1] = str(aux)
 
                    elif back: #move back
                        aux = self.new_node[robot_x][robot_y]
                        self.new_node[robot_x +1][robot_y] = str(aux)
                    
                    if front or left or right or back: #previous position eq to 0
                        self.new_node[robot_x][robot_y]= "0"

            if self.pallet == True and not self.prismatic: 
                self.pallet = False #move to the front without prismatic leaves the pallet on previous position
        
            self.movements.append("Move front") #get the movements
            self.g = self.parent.g +1

        elif rotation == "right_r" or rotation == "left_r":
            if self.pallet:
                if robot_x > d and \
                   robot_y > d and \
                   robot_x < (self.dimensions[0] -1-d) and \
                   robot_y < (self.dimensions[1] -1-d):
                   rot = True
                
                else:
                    rot = False

            if rotation == "right_r": #rotate to the right
                if not self.pallet:
                    if robot_r != "1":
                        aux = self.new_node[robot_x][robot_y]
                        self.new_node[robot_x][robot_y] = str(int(aux) -1)

                    else:
                        self.new_node[robot_x][robot_y] = "4"
                
                else:
                    if rot:
                        if robot_r != "1":
                            aux = self.new_node[robot_x][robot_y]
                            self.new_node[robot_x][robot_y] = str(int(aux) -1)  

                        else:
                            self.new_node[robot_x][robot_y] = "4"
                    
                    else:
                        pass
                
                if (self.pallet and rot) or not self.pallet:
                    self.movements.append("Rotate right")
                    self.g = self.parent.g +2

            elif rotation == "left_r": #rotate to the left
                if not self.pallet:
                    if robot_r != "4":
                        aux = self.new_node[robot_x][robot_y]
                        self.new_node[robot_x][robot_y] = str(int(aux) +1)

                    else:
                        self.new_node[robot_x][robot_y] = "1"

                else:
                    if rot:
                        if robot_r != "4":
                            aux = self.new_node[robot_x][robot_y]
                            self.new_node[robot_x][robot_y] = str(int(aux) +1)

                        else:
                            self.new_node[robot_x][robot_y] = "1"
                    
                    else:
                        pass

                if (self.pallet and rot) or not self.pallet:
                    self.movements.append("Rotate left")
                    self.g = self.parent.g +2
        
        elif rotation == "down": #move down if in pallet and with pallet
            if self.pallet: #if robot carries a pallet
                if robot_r == "2" or robot_r == "4":
                    self.base[robot_x][robot_y] = "5" #map recover horizontal pallet
                
                else:
                    self.base[robot_x][robot_y] = "6" #map recover vertical pallet

            self.prismatic = False
            self.movements.append("Move down")
            self.g = self.parent.g +3

        elif rotation == "up": #move up if in pallet and without pallet
            new_pos = self.robot_position[:2]
            if new_pos == self.initial_pallet[0][:2]:
                self.pallet = True
                try:
                    i = np.where(self.base == str(self.initial_pallet[0][2]))

                except Exception as e:
                    pass
                
                self.base[i] = "0"
                self.initial_pallet.pop()
                    
                self.prismatic = True
                self.movements.append("Move up")
                self.g = self.parent.g +3
            
            else:
                pass

        elif rotation == None: #no movements
            pass
        
        else:
            print("\n\nBad orientation")
            sys.exit()

        self.get_elements_pose()
        self.state = self.finalState()
        self.h = get_h()[0] #get h for h node
        self.f = self.g + self.h #computes h function
        

class Astar(object):
    """
        Class that implements the functions of A*

        param: ini: initial map
        param: req: desired map
        param: stats: file to dump information of statistics
        param: plot: plot f graphic

        Information of objects:
            Object:
                    0: No object
                    7: Object

            Pallet orientation:
                    5: Initial state horizontal
                    6: Initial state vertical
                    8: Final state horizontal
                    9: Final state vertical

            Robot orientation:
                    1: Front
                    2: Left
                    3: Back
                    4: Right  
    """

    def __init__(self, ini, req, stats, plot):
        self.closed = [] #closed list
        self.path = None #path to solution
        self.plot = plot #plot f graphic
        self.time = time.time() #execution time
        self.stats = stats #file to dump statistics
        self.base_map = ini
        self.solution = req #solution 
        self.success = False #success
        self.expanded_nodes = 0 #number of expanded nodes
        self.initial_state = ini #map in first iteration
        self.opened = [node(maps=self.initial_state, final_map=req, base=ini)] #opened list
        self.opened[0].get_elements_pose()
        self.opened[0].create_node(None)

    def extractN(self):
        """
            Extract de N element from opened if not in closed
        """

        for n in self.opened:
            N = n
            found = False
            for m in self.closed:
                comparison1 = n.new_node == m.new_node
                comparison2 = n.prismatic == m.prismatic
                if comparison1.all() and comparison2: #if node exists, pass
                    found = True

            if found == False:
                return N

        if found == False:
            return N
        
        else:
            self.success = False
            print("\n\n")
            print("No hay solucion")
            sys.exit()
  
    def expand(self, previous_node):
        """
            Expand a node (new pose of the robot)

            param: previous_node: node to expand
        """

        list_nodes = []
        my_base = np.copy(previous_node.base)
   
        #go forward
        front = node(maps=previous_node.new_node, parent=previous_node, prismatic=previous_node.prismatic, final_map=self.solution,\
                pallet=previous_node.pallet, base=my_base)
        front.create_node("front")
        comp1 = previous_node.new_node == front.new_node
        comp2 = previous_node.prismatic == front.prismatic
        if comp1.all() and comp2:
            pass #do not create a node if its the same node as father
            
        else:
            list_nodes.append(front)

        #rotate right
        right_r = node(maps=previous_node.new_node, parent=previous_node, prismatic=previous_node.prismatic, final_map=self.solution,\
                  pallet=previous_node.pallet, base=my_base)
        right_r.create_node("right_r")
        comp1 = previous_node.new_node == right_r.new_node
        comp2 = previous_node.prismatic == right_r.prismatic
        if comp1.all() and comp2:
            pass 
            
        else:
            list_nodes.append(right_r)

        #rotate left
        left_r = node(maps=previous_node.new_node, parent=previous_node, prismatic=previous_node.prismatic, final_map=self.solution,\
                 pallet=previous_node.pallet, base=my_base)
        left_r.create_node("left_r")
        comp1 = previous_node.new_node == left_r.new_node
        comp2 = previous_node.prismatic == left_r.prismatic
        if comp1.all() and comp2:
            pass 
            
        else:
            list_nodes.append(left_r)

        if not previous_node.prismatic: #if prismatic down
            up = node(maps=previous_node.new_node, parent=previous_node, prismatic=previous_node.prismatic, final_map=self.solution,\
            pallet=previous_node.pallet, base=my_base)
            up.create_node("up")
            comp1 = previous_node.new_node == up.new_node
            comp2 = previous_node.prismatic == up.prismatic
            if comp1.all() and comp2:
                pass 
            
            else:
                list_nodes.append(up)

        if previous_node.prismatic: #if prismatic up
            down = node(maps=previous_node.new_node, parent=previous_node, prismatic=previous_node.prismatic, final_map=self.solution,\
            pallet=previous_node.pallet, base=my_base)
            down.create_node("down")
            comp1 = previous_node.new_node == down.new_node
            comp2 = previous_node.prismatic == down.prismatic
            if comp1.all() and comp2:
                pass 
            
            else:
                list_nodes.append(down)

        for s in list_nodes:
            if s.state == True:
                self.success = True

        self.expanded_nodes += len(list_nodes)
        return list_nodes

    def sortedAppend(self, child):
        """
            Sort the opened list with f function of each node

            param: child: node to insert in oppened list
        """

        actual = child.f
        inserted = False
        if len(self.opened) == 0:
            self.opened.append(child)

        else:
            for node in range(len(self.opened)):
                previous = self.opened[node]
                if actual < previous.f and not inserted:
                    self.opened.insert(node, child) #in front of previous
                    inserted = True
            
            if not inserted:
                self.opened.append(child)


    def path_to_win(self, node, back_to_origen):
        """
            Return a list with necessary A* movements, if it is success and length of the plan

            param: node: Node to compute the path
        """

        loss = 0
        nodes = 0
        f_list = []
        movement_list = []
        
        while node.parent != None:
            movement_list.append(node.movements)
            nodes +=1
            loss += node.g
            f_list.append(node.f)
            node = node.parent
  
        inward_list = movement_list[::-1]
        inward_list.append(["Move down"])

        if back_to_origen:
            t = time.time()
            total_time = t - self.time
            self.stats.write("F of final node ---> {}".format(node.f))
            self.stats.write("\n")
            self.stats.write("Execution time ---> {}".format(total_time))
            self.stats.write("\n")
            self.stats.write("Total loss ---> {}".format(loss))
            self.stats.write("\n")
            self.stats.write("Number of nodes ---> {}".format(nodes))
            self.stats.write("\n")
            self.stats.write("Number of expanded nodes ---> {}".format(self.expanded_nodes))
            self.stats.write("\n")
            self.stats.close()
            if self.plot: #plot f value
                plt.xlabel('Nodes')
                plt.ylabel('F value')
                plt.plot(f_list[::-1])
                plt.savefig('f_function.png')

            return self.success,  inward_list
        
        else:
            return inward_list