#!/usr/bin/env python

import os
import time
#import rospy
from copy import deepcopy
from numpy.core.fromnumeric import sort
from star import Astar
#from navigation import Navigation

class Practica2:

    def __init__(self):
        #self.nav = Navigation()
        self.astar = None
        self.robot = None

    def execAstar(self, path_to_initial, path_to_requests, path_to_final, path_to_s, show):
        """
            Calls Astar class to use its functions

            param: path_to_initial: path of the the initial map
            param: path_to_requests: path of the requested map
            param: path_to_final: path to file with movements to the solution
            param: path_to_s: path to file of Astar statistics
            param: show: show the best current node in terminal
        """

        def execute(initial, requests, path_of_statistics, back_to_origen, list_of_movements, walk, show):
            """
                Function to call A* for each map

                param: initial: the initial map
                param: requests: the requested map
                param: path_of_movements: path to file with movements to the solution
                param: path_of_statistics: path to file of Astar statistics
                param: back_to_origen: if robot go back to initial position
                param: show: show the best current node in terminal
            """

            maze = Astar(initial, requests, path_of_statistics, True)
            while bool(maze.opened) and maze.success == False:
                N = maze.extractN()
                if show:
                    os.system('cls||clear')
                    print("\n\n")
                    print("h ---> {}".format(N.h))
                    print("g ---> {}".format(N.g))
                    print("f ---> {}".format(N.f))
                    print("Piston ---> {}".format(N.prismatic))
                    print("Debajo de pallet ---> {}".format(N.pallet))
                    print("Nodo preferente: ")
                    print(N.new_node)

                if not N.state:
                    N_child = maze.expand(N)
                    maze.closed.append(N)
                    
                    for child in N_child:
                        maze.sortedAppend(child) #what we need to sort

                elif N.state:
                    break

            if N.state == False: #choose the correct node
                for n in N_child:
                    if n.state == True:
                        N = n

            self.robot = N.robot_position
            if back_to_origen:
                success, Plan = maze.path_to_win(N, back_to_origen)
                for m in Plan:
                    try:
                        list_of_movements.append(m[0])
                    
                    except:
                        pass
                
                for move in list_of_movements[::-1]:
                    try:
                        if move == "Move front":
                            list_of_movements.append("Move back")

                        elif move == "Rotate right":
                            list_of_movements.append("Rotate left")

                        elif move == "Rotate left":
                            list_of_movements.append("Rotate right")

                        elif move == "Move up":
                            pass
                        
                        elif move == "Move down":
                            pass
                    
                    except Exception as e:
                        print(e)
                
                for movement in list_of_movements:
                    walk.write(movement)
                    walk.write("\n")

                walk.close()
                self.astar = list_of_movements
                print("\n")
                print("Solucion: ")
                print("\n")
                print(N.new_node)
                print("\n")
                print("Exito ---> {}".format(success))
                print("\n")
                print("Longitud del plan ---> {}".format(len(list_of_movements)))
                print("\n")
                print("Movimientos ---> {}".format(list_of_movements))
                print("\n")

            else:
                Plan = maze.path_to_win(N, back_to_origen)
                print("plan", Plan)
                for m in Plan:
                    try:
                        list_of_movements.append(m[0])
                    
                    except:
                        pass

        back_to_origen = False
        list_of_movements = []
        list_of_preference = [] #list with sorted pallets

        with open(path_to_initial, 'r') as i: #the initial map
            initial = []
            initial = [line.split() for line in i]

        with open(path_to_requests, 'r') as r: #the requested map
            requests = []
            requests = [line.split() for line in r]

        walk = open(path_to_final, 'w') #file of movements
        path_of_statistics = open(path_to_s, 'w') #file of statistics

        for j, i in enumerate(initial): #fill the list with pallets
            for k, f in enumerate(i):
                if f[0] == "5" or f[0] == "6":
                    list_of_preference.append(int(f[2]))

                if f[0] == "1" or f[0] == "2" or f[0] == "3" or f[0] == "4": #if robot
                    self.robot = [j, k, f[0]]

        list_of_preference = sorted(list_of_preference)
        for back, min_number in enumerate(list_of_preference): #create a new map for each pallet
            added_init = False
            added_req = False
            new_init_map = deepcopy(initial)
            new_req_map = deepcopy(requests)
            for index, r in enumerate(initial):
                for i, c_init in enumerate(r):
                    if c_init[0] == "1" or c_init[0] == "2" or c_init[0] == "3" or c_init[0] == "4":
                        new_init_map[index][i] = "0"

                    elif c_init[0] == "5" or c_init[0] == "6":
                        if not added_init:
                            if int(c_init[2]) == min_number:
                                new_init_map[index][i] = c_init[0]
                                added_init = True

                            else:
                                new_init_map[index][i] = "0"
                            
                        else:
                            new_init_map[index][i] = "0"

                    elif c_init[0] == "7":
                        new_init_map[index][i] = c_init

                    else:
                        new_init_map[index][i] = "0"

                    c_req = new_req_map[index][i]
                    if (c_req[0] == "8" or c_req[0] == "9"):
                        if not added_req:
                            if int(c_req[2]) == min_number:
                                new_req_map[index][i] = c_req[0]
                                added_req = True
                            
                            else:
                                new_req_map[index][i] = "0"
                        
                        else:
                            new_req_map[index][i] = "0"

                    else:
                        new_req_map[index][i] = c_req
            
            if back == len(list_of_preference) -1: #at last 
                back_to_origen = True

            new_init_map[self.robot[0]][self.robot[1]] = str(self.robot[2]) #put the robot on last position
            execute(new_init_map, new_req_map, path_of_statistics, back_to_origen, list_of_movements, walk, show) #Astar for each map
        
    def execMovements(self):
        """
            Function to move the robot in simulation
        """

        for move in self.astar:
            if move == [] or move == "":
                pass

            else:
                if str(move) == "Move front":
                    print("Moving to the front")
                    #self.nav.move(1)
                
                if str(move) == "Move back":
                    print("Moving to the back")
                    #self.nav.move_b(1)
                
                elif str(move) == "Rotate right":
                    print("Rotate right")
                    #self.nav.rotateRight()
                
                elif str(move) == "Rotate left":
                    print("Rotate left")
                    #self.nav.rotateLeft()
                
                elif str(move) == "Move up":
                    print("Moving up")
                    #self.nav.upLift()

                elif str(move) == "Move down":
                    print("Moving down")
                    #self.nav.downLift()


if __name__ == '__main__':
    t = time.time()
    try:
        p = Practica2()
        p.execAstar("D:/Universidad/Universidad-3/Agentes inteligentes/src/mapa_ini.txt", \
                    "D:/Universidad/Universidad-3/Agentes inteligentes/src/mapa_req.txt", \
                    "D:/Universidad/Universidad-3/Agentes inteligentes/src/moves.txt", \
                    "D:/Universidad/Universidad-3/Agentes inteligentes/src/stats.txt", True)
        print("\n")
        p.execMovements()
    
    except Exception as error:
        print("\n\nNo se ha podido completar el algoritmo debido al siguiente error: ")
        print("\n")
        print(error)
    
    final_time = time.time() - t
    print("\n\nExecution time ---> {}".format(final_time))