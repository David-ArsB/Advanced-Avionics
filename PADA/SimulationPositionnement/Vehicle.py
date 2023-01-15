import numpy as np

class vehicle():
    def __init__(self, x, vi, ai, loiterRadius):
        self.loiterRadius = loiterRadius
        self.actPos = x
        self.actXDot = vi
        self.actXDotDot = ai

        self.GPS_Error = 2

    def getTheta(self):
        return np.arctan2(self.actPos[1] , self.actPos[0])

    def moveRad(self, deltaT, otherAcc = 0):

        theta = self.getTheta()
        acc = np.linalg.norm(self.actXDot)**2/self.loiterRadius
        acc = np.multiply([np.cos(theta),np.sin(theta)], -acc)
        newX = self.actPos[0] + self.actXDot[0]*deltaT+acc[0]*deltaT**2/2
        newY = self.actPos[1] + self.actXDot[1]*deltaT+acc[1]*deltaT**2/2
        #newTheta = np.arctan2(self.actPos[1] + newY, self.actPos[0] + newX)
        #newTheta = self.getTheta() + np.linalg.norm(self.actXDot)/self.loiterRadius*deltaT+self.actXDotDot*deltaT**2/2
        #self.actXDot = np.multiply([np.cos(newTheta),np.sin(newTheta)], np.linalg.norm(self.actXDot))
        self.actXDot = self.actXDot+acc*deltaT
        self.actPos = np.array([newX,newY])

    def move(self, deltaT, otherAcc = 0):


        newX = self.actPos[0] + self.actXDot[0]*deltaT+(self.actXDotDot[0]+otherAcc[0])*deltaT**2/2
        newY = self.actPos[1] + self.actXDot[1]*deltaT+(self.actXDotDot[1]+otherAcc[1])*deltaT**2/2
        self.actXDot = self.actXDot+self.actXDotDot*deltaT
        self.actPos = np.array([newX,newY])

    def measureGPS(self):
        error = (np.random.random_sample(2)-0.5)*self.GPS_Error*2
        return [self.actPos[0]+error[0], self.actPos[1]+error[1]]