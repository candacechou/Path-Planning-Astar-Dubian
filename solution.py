#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# {Candace Chou}
# {chchou@kth.se}
import numpy as np
import math
from operator import attrgetter
class Node(object):
    def __init__(self,x,y,car):
        self.x = x
        self.y = y
        self.car = car
        self.cost_G = 0
        self.cost_H = 0
        self.cost = 0
        self.parent = None
        self.time = 0
        self.theta = 0
        self.verified = False
        self.control = 0

        #self.find_cost()

    def find_cost(self):
        origin_distance = ((self.x - self.car.x0)**2 + (self.y - self.car.y0)**2)**0.5
        self.cost_G = origin_distance
        origin_distance = ((self.x - self.car.xt)**2 + (self.y - self.car.yt)**2)**0.5
        self.cost_H = origin_distance
        self.cost = self.cost_G + 2*self.cost_H



    def reach(self):
        distance = ((self.x-self.car.xt)**2 +(self.y-self.car.yt)**2)**0.5
        if distance < 1 :
            return True
        else:
            return False

class mapping(object):

    def __init__(self,car):
        self.x = car.xub
        self.y = car.yub
        self.cars =car
        self.status = np.zeros((int(self.x*5)+1 , int(self.y*5)+1))
        self.steering_status = {}


class set(object):
    def __init__(self):
        self.sets = []

    def removeset(self,node):
        self.sets.remove(node)

    def addset(self,node):
        self.sets.append(node)

    def least_cost(self):
        min_cost = min(self.sets,key = attrgetter('cost'))
        return min_cost


    def check(self):
        if self.sets:
            return True
        else :
             return False

def verify_node(x,y,theta,car,maps):
    if x < car.xlb :
        return False
    elif y < car.ylb:
        return False
    elif x > car.xub:
        return False
    elif y > car.yub :
        return  False
    for w in car.obs:
        if (((w[0]-x)**2 +(w[1]-y)**2)**0.5 > w[2]+0.5) == False :
            return False
    if maps.status[int(x*5)][int(y*5)] == 1:
        return False


    return True

def expanding(car,current,timestep,maps):

    childnode = []
    for delta in [-45,0,45]:
        delta = math.pi/180 * delta
        i = 0
        x1 = current.x
        y1 = current.y
        theta = current.theta
        while (i < timestep):
            x1,y1,theta = car.step(x1,y1,theta,delta)
            if  verify_node(x1,y1,theta,car,maps):
                i = i + 1
            else :
                #maps.status[int(x1)][int(y1)] = 1
                i = 0
                break
        if i != 0:
            temp = Node(x1,y1,car)
            temp.find_cost()
            temp.time = current.time + timestep
            temp.control = delta
            temp.theta = theta
            temp.parent = current


            #print ((temp.x,temp.y))
            k = 0
            if (int(x1*5),int(y1*5)) in maps.steering_status:
                #print((int(x1*10),int(y1*10)),".....")
                #print (len(maps.steering_status[(int(x1*10),int(y1*10))]))
                if len(maps.steering_status[(int(x1*5),int(y1*5))]) < 10:
                    for items in maps.steering_status[(int(x1*5),int(y1*5))]:
                        if abs(items * 180/math.pi - theta* 180/math.pi) < 10 :
                            k = 1
                    if k == 0 :

                            childnode.append(temp)
                            maps.steering_status[(int(x1*5),int(y1*5))].append(theta)
                else :
                    maps.status[int(x1*5)][int(y1*5)] =  1

            else :
                maps.steering_status[(int(x1*5),int(y1*5))]=[]
                maps.steering_status[(int(x1*5),int(y1*5))].append(theta)
                childnode.append(temp)


    return childnode,maps


def solution(car):
    # if car.obs[0][2] == 0.2:
    #    return [0], [0, car.dt]
    print ("-------------------------------------------------------------------------------------------")
    ''' <<< write your code below >>> '''
    controls=[]
    times=[]
    timestep = 40
    beginning = Node(car.x0,car.y0,car)

    openset = set()
    openset.addset(beginning)
    closeset = set()
    maps = mapping(car)
    children = []

    while openset.check():
        #print ("current position:")
        current = openset.least_cost()
        #print ((current.x,current.y))
        if current.reach():
            print ("reach the goal:%s",(current.x,current.y))
            times.insert(0,current.time*car.dt)
            while(current.parent !=None):
                current = current.parent
                controls.insert(0,current.control)
                times.insert(0,current.time*car.dt)
            break
        else:
            children,maps = expanding(car,current,timestep,maps)

            for next_state in children:
                #print("nextstate",next_state.x,next_state.y)
                if next_state in closeset.sets:
                    continue
                else :
                    openset.addset(next_state)

            openset.removeset(current)
            closeset.addset(current)

    return controls, times
