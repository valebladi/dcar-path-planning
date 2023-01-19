#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# {Valeria Bladinieres Justo}

from dubins import *
from math import sqrt

def solution(car):
    time = 0.20
    time2 = 0
    times = [0]
    controls,leng, w = all(car)
    if w == True:
        for i in range(leng):
            time2 += 0.5
            times.append(time2)
    if w == False:
        for i in range(leng):
            time+= 0.2
            times.append(time)


    return controls, times

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
    for i in phil:
        xn, yn, thetan = dist(cnode.x[0],cnode.x[1], cnode.ts, car,i,20)
        nnode = Node((xn,yn),thetan, cnode,i)
        if nnode not in cl:
            if obstacles(car, nnode.x[0],nnode.x[1]) != True: 
                cl.append(nnode)
                continue
            if (car.xlb > nnode.x[0] or nnode.x[0]>= car.xub or car.ylb > nnode.x[1] or nnode.x[1] >= car.yub):
                cl.append(nnode)
                continue
            children.append(nnode)
   
        
    for child in children:
        nnode.g = cnode.g + 0.01
        for ochild in ol:
            if (ochild == child) and (child.g >ochild.g):
                continue
        ol.append(child)
        
def heuristic(a,b):
    aa = ((a[0]-b[0]) ** 2)
    bb = ((a[1]-b[1]) ** 2 )
    g = sqrt(aa + bb)
    return g

def obstacles(car, x,y):
    for i in car.obs:
        xo = i[0]
        yo = i[1]
        r = (i[2]+0.5)**2
        res =(x-xo)**2 + (y-yo)**2
        if (res<r):
            return False
    return True

def radiopoint(currx,curry, endx,endy):
    r = 1.0**2
    res = (currx-endx)**2+(curry-endy)**2
    if (res>r):
        return False
    return True

def dist(x,y, theta,car,phi, n):
    for i in range(0,n):
        xn, yn, thetan = step(car, x, y, theta, phi)
        x = xn 
        y = yn
        theta = thetan
    return x,y,theta

def updateNe2(car, ol,cl,phil, cnode, n,graph):
    children = []
    for i in phil:
        xn, yn, thetan = dist(cnode.x[0],cnode.x[1], cnode.ts, car,i,n)
        #print "Antes",xn,yn
        yyn = int(round(yn*5))
        xxn = int(round(xn*5))
        #print "Despues: ",xn,yn
        if yyn<50 and xxn<100 and graph[yyn][xxn] !=1 and yyn>=0 and xxn>=0 and graph[yyn][xxn] !=4 :
            nnode = Node((xn,yn),thetan, cnode,i)
            children.append(nnode)
   
        
    for child in children:
        addC = 0
        nnode.g = cnode.g 
        for ochild in ol:
            #print child.x
            #print ochild.x
            #raw_input()
            if ochild.x != child.x:
                addC = addC+1
        for cchild in cl:
            #print cchild.x
            #print child.x
            #raw_input()
            if child.x != cchild.x:
                addC = addC+1
        if len(ol) == 0:
            ol.append(child)
            addC = 2
        if addC > 1:
            #print "Se añadee---------", child.x, child.s
            graph[int(round(child.x[1]*5))][int(round(child.x[0]*5))] = 4
            ol.append(child)
    
    #print len(ol)
    return graph

def insertObs(car):
    lx = 100
    ly = 50
    grid = [[0 for x in range(lx)] for y in range(ly)] 

    for i in car.obs:
        x = int(round(i[0]*5))
        y = int(round(i[1]*5))
        r = int(round(i[2]+0.8*5))
        mix = x-r
        mx = x+r
        miy = y-r
        my = y+r

        for k in range(0,ly):
            for g in range(0,lx):
                if g == x and k == y:
                    for i in range(mix,mx):
                        for j in range(miy,my):
                            if i >= 0 and i <100 and j>=0 and j<50:
                                grid[j][i] = 1

    return grid

def secProble(car, ol, cl,xend, yend, path, phil,add):
    grid = insertObs(car)
    while len(ol) > 0:
        cnode = ol[0]
        cindex = 0
        for index, item in enumerate(ol):
            fc = heuristic(cnode.x, (car.xt, car.yt))
            f = heuristic(item.x,(car.xt, car.yt))
            if  (item.g +f) < (cnode.g +fc):
                cnode = item
                cindex = index
        ol.pop(cindex)
        cl.append(cnode)

        if radiopoint(cnode.x[0],cnode.x[1], xend,yend):
            cur = cnode
            while cur is not None:
                path.append(cur.s)
                cur = cur.np
            return path[::-1], len(path), True
        else:
            grid = updateNe2(car,ol,cl,phil,cnode,50, grid)

       
        add+=1
    

def all(car):
    # initial state
    x, y = car.x0, car.y0
    xend, yend = car.xt, car.yt
    theta = 0
    #mi code
    phil = [pi/4,0,-pi/4]
    ol = []
    cl = []
    path = []
    time = [0.01]
    add = 0

    n = Node((x,y),theta,None,0)
    ol.append(n)

    while len(ol) > 0:
        cnode = ol[0]
        cindex = 0
        for index, item in enumerate(ol):
            fc = heuristic(cnode.x, (car.xt, car.yt))
            f = heuristic(item.x,(car.xt, car.yt))
            if f < fc:
                cnode = item
                cindex = index
        ol.pop(cindex)
        cl.append(cnode)


        if radiopoint(cnode.x[0],cnode.x[1], xend,yend):
            cur = cnode
            while cur is not None:
                path.append(cur.s)
                cur = cur.np
            return path[::-1], len(path),False
        else:
            updateNe(car,ol,cl,phil,cnode)
    
        if (add>400):
            ol = []
            cl = []
            ol.append(n)
            a,b, c = secProble(car, ol,cl, xend, yend, path, phil,add)
            if b == 1:
                break
            else:
                return  a,b, c
        add+=1
    return [0], 1