#everything is in mm
import cv2
import numpy as np
import serial as ss
import time, sys, traceback
s=ss.Serial()
s.port = "COM8" #todo check this doesn't change
s.baudrate=57600
s.timeout=.1
s.close()

totalDist=0
    
def movePrinter(x,y):
    s.open()
    s.read(100)
    toSend = "G0 X{0} Y{1}\n".format(x,y)
    print(toSend)
    s.write(bytes(toSend,'utf-8')) #todo turn this into a function
    s.read(100)
    toSend="M552\n" #todo this doesn't work, plan it out with speed
    response = [-1 for x in range(10)]
    while float(response[1]) < x-0.001 and float(response[3])<y-0.001 and float(response[1]) > x+0.001 and float(response[3]) > y+0.001:
        s.write(bytes(toSend,'utf-8'))
        response = s.read(100).strip().split(" ")
        print("waiting...")
        time.sleep(1) #todo this doesn't work, plan it out with speed
    print("reached",x,y)
    s.close()

def getDistanceTest(x,y):
    #puts a box at x:1-2 y:2-3, with an additional square in the top right corner that is .5cm
    if(x<=20 and x>=10 and y>=20 and y<=30):
        if(x>=15 and y>=25):
            return 30
        return 20
    return 0

class Point:
    def __init__(self,x,y,dist):
        self.x=x
        self.y=y
        self.dist=dist

    def __repr__(self):
        return "Point: {0:4.2f} {1:4.2f} {2:4.2f}".format(self.x, self.y, self.dist)

class PointGrid:
    #handles the data structure for a 2d grid of distances
    
    #2d array here
    #Instance variables: minX, maxX, minY, maxY, z, resolution
    
    #constructor here
    def __init__(self, minX, maxX, minY, maxY, z, resolution):
        self.grid = [[Point((minX+x*resolution),(minY+y*resolution),0) for x in range(1+int((maxX-minX)/resolution))]
                         for y in range(1+int((maxY-minY)/resolution))]
        self.minX=minX
        self.maxX=maxX
        self.minY=minY
        self.maxY=maxY
        self.z=z
        self.res=resolution
        self.width=len(self.grid[0])
        self.height=len(self.grid)
        self.maxDist = 40
        self.subgrids = [] #todo use this

    def __repr__(self):
        st = ""
##        for y in range(len(self.grid)):
##            for x in range(len(self.grid[0])):
##                st += "{:6.4f} ".format(self.grid[-1-y][x].dist)
##    
##            st+="\n"
        st+="PointGrid: X:{0} -> {1}, Y:{2} -> {3}, resolution: {4} ({5} x {6} values)".format(
                self.minX,self.maxX,self.minY,self.maxY,self.res,len(self.grid[0]),len(self.grid))
        st+="\n"
        return st
    
    def addPoint(self,x,y,dist):
        self.grid[y][x] = Point(x*self.res,y*self.res,dist)

    def fill(self):
        for y in self.grid:
            for x in y:
                #movePrinter(x.x,x.y) # todo zigzag
                global totalDist
                totalDist+=self.res+2
                x.dist = getDistanceTest(x.x,x.y)
                
    
    def toImage(self, fname='out.png'):
        #generate a bitmap image (for easy viewing and edge detection)
        img = np.zeros((self.height, self.width, 3),np.uint8)
        print(self.height,self.width,len(self.grid[0]),len(self.grid))
        #todo improve performance?
        for y in self.grid:
            for x in y:
                img.itemset((int(x.y/self.res),int(x.x/self.res),2),x.dist*4)
        cv2.namedWindow('wat',cv2.WINDOW_NORMAL)
        cv2.imshow('wat',img)
        cv2.resizeWindow('wat',700,int(700*(self.height/self.width)))
        cv2.waitKey(0)
        cv2.destroyAllWindows()
        pass

    def pointCloudVerts(self):
        s = ""
        for y in self.grid:
            for x in y:
                s+="v {0:7.2f} {1:7.2f} {2:7.2f}\n".format(x.x,x.y,x.dist)
        return s

    def pointCloudFaces(self, offset=0):
        s = ""
        for y in range(len(self.grid)):
            for x in range(len(self.grid[0])):
                n = y*self.width+x+1+offset
                s+="f {} {} {} {}\n".format(n,n+1,n+self.width+1,n+self.width)
        return s
    
    def toPointCloud(self, fname="out.obj", fnamePrev=""):
        #generate an obj file

            

        f = open(fname,"w") #todo make append?
        f.write("#Generated from: (minX, maxX, minY, maxY, z, res, width, height)\n")
        f.write("#vars: {} {} {} {} {} {} {} {} \n".format(self.minX,self.maxX,self.minY,self.maxY,
                                                  self.z,self.res,self.width,self.height))

        #todo write subgrid info here           
        f.write("#verts go row by row, 0->x then 0->y\n")
        f.write(self.pointCloudVerts())
        offset = self.width * self.height
        f.write("#wrote "+str(offset)+" current verts, adding old verts\n")
        if fnamePrev:
            oldG = pointGridFromObj(fnamePrev)
            for y in oldG.grid:
                for x in y:
                    if x.x > self.minX and x.x<self.maxX and x.y>self.minY and x.y<self.maxY:
                        x.dist=0 #our subgrid is better
            f.write(oldG.pointCloudVerts())
            f.write(oldG.pointCloudFaces( offset))
                           
        f.write(self.pointCloudFaces())
        f.close()
        pass

    def getBestRangeAt(x,y):
        #Go through subgrids and take the average of points surrounding this point.
        #This will eventually be Z, from the lowest point of the thing to draw. 
        pass

    def getXYEdges():
        #this will be x and y for homing
        #hopefully the object is fairly square, otherwise we can generate a bounding box
        pass

def pointGridFromObj(fname="out.obj"):
    
        try:
            with open(fname,"r") as pgf:
                for l in pgf:
                    if "#vars" in l[:6]:
                        oldG = PointGrid(*[int(v) for v in l.split(" ")[1:7]])
                    if "#subg" in l[:6]:
                        oldg.subgrids[].append(PointGrid(*[int(v) for v in l.split(" ")[1:7]]))
                        #todo handle offsets and put this in file
                    if l[0]=="v":
                        pVars = l.strip().split(" ")
                        pVars = Point(*[float(v) for v in pVars if v!="" and v!="v"])
                        oldG.addPoint(int(pVars.x/oldG.res), int(pVars.y/oldG.res), pVars.dist) #TODO check floating point precision here
                        #parse subgrids
            return oldG
        except:
            print("error", sys.exc_info(), traceback.format_exc());

#todo: scan small regions and lines to get a more exact picture.
#todo: determine lag time on laser, to determine the max speed we can go. 

p=PointGrid(0,150,0,160,40,4)
#p.addPoint(0,0,1) #distance of 1 at 0,0
#p.addPoint(5,0,2) #distance of 2 in the x direction
#p.addPoint(0,5,3) #distance of 3 in the y direction

p.fill()
f=open("out.txt","w")
f.write(str(p))
f.close()

p.toPointCloud()
print(totalDist)

#p.toImage()
#todo automate edge detection (bounding box is fine unless the object turns).
p2 = PointGrid(5,25,15,35,40,.5)
p2.fill()
p2.toPointCloud("out2.obj","out.obj")
print(totalDist)
