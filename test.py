#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# {Valeria Bladinieres Justo}
# {19950928}
# {vabj@kth.se}

from dubins import *
from math import sqrt
import matplotlib.pyplot as plt
from numpy import sort
show_animation = True


def solution(car):
    time = 0.20
    times = [0]

    if show_animation:  # pragma: no cover
        for ob in car.obs:
            plt.plot(ob[0], ob[1], ".k", ms =ob[2]*100)
        plt.plot(car.x0, car.y0, "og")
        plt.plot(car.xt, car.yt, "xb")
        plt.grid(True)
        plt.axis("equal")

    controls,leng = all(car)

    if show_animation:  # pragma: no cover
        plt.show()

    for i in range(leng):
        time+= 0.20
        times.append(time)

    return controls, times

def sol2(car):
    #solution 2
    waypoint = []
    #print ("New graph:")
    #print car.obs

    if len(car.obs)>18:
        for i in car.obs:
            numObs = obstacles2(car, i[0],i[1])
            if numObs <= 2:
                #checo si esta outof bounds
                if (20 > i[0] and i[0]> 1 and 9.5 > i[1] and i[1] > 1):
                    waypoint.append((i[0],i[1]))
            
    #wpoints = meetnear(waypoint,car)
    return waypoint

def meetnear(wp, node):
    x = node.x0
    y = node.y0
    xx = wp[0]
    yy = wp.x[1]
    g = heuristic2(x,y,xx,yy)
    return g

def obstacles2(car, x,y):
    cont = 0
    for i in car.obs:
        xo = i[0]
        yo = i[1]
        r = (0.41)**2
        res =(x-xo)**2 + (y-yo)**2
        if (res<r):
            cont = cont+1
    return cont

def heuristic2(ax,ay,bx,by):
    # euclidian distance
    aa = ((ax-bx) ** 2)
    bb = ((ay-by) ** 2 )
    g = sqrt(aa + bb)
    return g


class Node():
    #create node 
    def __init__(self, x=None, ts =None,np=None,s=None):
        self.ts = ts    #ts phi discrete
        self.x = x      #x,y position 
        self.g = 0      #real plus estimated
        self.f = 0      #f priority queue (real g and estimated h, costs)
        self.np = np    #np backpointer to parent
        self.s = s      #xdiscrete

def updateNe(car, ol,cl,phil, cnode):
    children = []
    #print "updateNe"
    for i in phil:
        #print "insed"
        #print (cnode.x[0],cnode.x[1])
        #xn, yn, thetan = step(car, cnode.x[0],  cnode.x[1], cnode.ts, i)
        xn, yn, thetan = dist(cnode.x[0],cnode.x[1], cnode.ts, car,i)
        #print (heuristic((cnode.x[0],cnode.x[1]),(xn,yn)))
        #print en
        #xn, yn, thetan = cnode.x[0]+0.1,cnode.x[1]+0.1, cnode.ts+i
        nnode = Node((xn,yn),thetan, cnode,i)
        #print (cnode.x,cnode.ts)
        #print (nnode.x,nnode.ts)
        #raw_input()
        #all succiding states given phil and the node
        if nnode not in cl:
            #check obstacles
            if obstacles(car, nnode.x[0],nnode.x[1]) == False: 
                cl.append(nnode)
                #print ("Found obstacle", nnode.x)
                #raw_input()
                continue
            #check outbounds
            if (car.xlb > nnode.x[0] or nnode.x[0]>= car.xub or car.ylb > nnode.x[1] or nnode.x[1] >= car.yub):
                cl.append(nnode)
                #print ("Out of bounds", nnode.x)  
                continue
            #elif radiopoint(nnode.x, ol):
                #print ("toca el punto")
            
            children.append(nnode)
        #print "len(children) "
        #raw_input()         
        
    for child in children:
        for cchild in cl:
            #print cchild.x
            if cchild.x == child.x:
                continue
        #nnode.g = cnode.g + 0.01
        for ochild in ol:
            if (ochild == child) and (child.g >ochild.g):
                continue
        #print ("hijo:", nnode.x, nnode.f)  
        ol.append(child)

def heuristic(a,b):
    # euclidian distance
    aa = ((a[0]-b[0]) ** 2)
    bb = ((a[1]-b[1]) ** 2 )
    g = sqrt(aa + bb)
    return g

def obstacles(car, x,y):
    # find (x - center_x)^2 + (y - center_y)^2 < radius^2.
    for i in car.obs:
        xo = i[0]
        yo = i[1]
        r = (i[2]+0.5)**2
        res =(x-xo)**2 + (y-yo)**2
        if (res<r):
            #print ("Hits ostacle: ", i)
            return False
    return True

def radiopoint(currx,curry, endx,endy):
    r = 2**2
    res = (currx-endx)**2+(curry-endy)**2
    if (res>r):
        return False
    return True

def dist(x,y, theta,car,phi):
    for i in range(0,20):
        xn, yn, thetan = step(car, x, y, theta, phi)
        x = xn 
        y = yn
        theta = thetan
    return x,y,theta

def all(car):

    xend, yend = car.xt, car.yt
    #wp = sol2(car)
    #wp.append((xend,yend))

    count = 0
    ##
    x, y = car.x0, car.y0
    theta = 0
    #mi code
    phil = [pi/4,0,-pi/4]
    ol = []
    cl = []
    path = []
    add = 0
    
    raw_input()

    n = Node((x,y),theta,None,0)
    ol.append(n)
 
    while len(ol) > 0:
        #print ("Lenght of list: ", len(ol))
        cnode = ol[0]
        cindex = 0
        for index, item in enumerate(ol):
            #print count
            #print  ("Heuristic with", wp[count][0],wp[count][1])
            #print  ("Heuristic with", item.x)
            fc = heuristic(cnode.x, wp[count])
            f = heuristic(item.x, wp[count])
            if f < fc:
                cnode = item
                cindex = index
        ol.pop(cindex)
        cl.append(cnode)
        
        
        #if goal is found return path
        if radiopoint(cnode.x[0],cnode.x[1], xend,yend):
            #print ("Found final point")
            cur = cnode
            while cur is not None:
                path.append(cur.s)
                cur = cur.np
            return path[::-1], len(path) #inverted path
        

         
        #raw_input()
        
        if radiopoint(cnode.x[0],cnode.x[1], wp[count][0],wp[count][1]):
            count +=1
            ol = ol[5:]
            cl = []
            add = 0
            #updateNe(car,ol,cl,phil,cnode)
            #print "encontrado"
            raw_input()
        else: 
            #print ("entra")
            updateNe(car,ol,cl,phil,cnode)
        
        
        if (add>300):
            if count==len(wp)-1:
                print "==========Iam  stuck"
                break
            else:
                count +=1
                add = 0
                print "just stuck and gong to next way point"
            raw_input()
       


        if show_animation:  # pragma: no cover
            plt.plot((cnode.x[0], car.xlb), (cnode.x[1], car.ylb), "xc")
            if len(cl) % 10 == 0:
                plt.pause(0.001)

        add+=1
        #print ("Add",add)
    return [0], 1
