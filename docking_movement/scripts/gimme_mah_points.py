
import math
import matplotlib.pyplot as plt
import numpy as np

class Test_Points:

    def __init__(self,max_angle,min_distance,max_distance,offset) : #angles in degrees
        self.max_angle=math.radians(max_angle)
        self.offset=offset
        self.max_distance=max_distance
        self.min_distance=min_distance
        self.turning_instructions=[]
        self.driving_instructions=[]
        self.carti_points=[]
        self.polar_points=[]
        self.generate_points()
        self.generate_instructions()
    
    def generate_points(self):
        radius=[self.min_distance,(self.min_distance+self.max_distance)/2,self.max_distance]
        angle=[-self.max_angle/2,0,self.max_angle/2]
        for r in range(3):
            for a in range(3):
                (new_r,new_a)=self.coord_transform(radius[r],angle[a])
                x=new_r*math.cos(new_a)
                y=new_r*math.sin(new_a)
                self.carti_points.append([x,y])
                self.polar_points.append([new_r,new_a])
    
    def coord_transform(self,radius,angle):
        new_radius=math.sqrt(radius**2+self.offset**2-2*radius*self.offset*math.cos(angle-0))
        #print(new_radius)
        if new_radius==0:
            new_angle=0
        else:
            new_angle=math.asin(radius/new_radius*math.sin(angle-0))
            if self.offset>=radius*math.cos(angle):
                new_angle=math.pi-new_angle
        return(new_radius,new_angle)

    def generate_instructions(self):
        for i in range(9):
            self.turning_instructions.append(self.modpi(self.polar_points[i][1]+math.pi))
            self.driving_instructions.append(self.polar_points[i][0])

    def modpi(self,angle):#maps values to between -pi and pi
        angle=angle%(math.pi*2)
        if angle>math.pi:
            angle=angle-2*math.pi
        return angle

if __name__ == '__main__':
    test1=Test_Points(90,1.1,2,1)
    print(test1.polar_points)
    print(test1.driving_instructions)
    print(test1.turning_instructions)
    
    p=np.array(test1.carti_points)
    line1=plt.scatter(p[:,0],p[:,1])
    line2=plt.plot(-test1.offset,0,'r*')
    line3=plt.plot(0,0,'ko')
    plt.legend(['Tag Point', 'TARS Home Point', 'Test Points'])
    plt.grid()
    plt.axis('equal')
    plt.show()
    