import numpy as np
from scipy.spatial import distance as dst

class robot:
    def __init__(self, radius, init_position, final_position, K, dt, passive):
        self.radius = radius
        self.init_position_x = init_position[0]
        self.init_position_y = init_position[1]
        self.currentPosition_x = init_position[0]
        self.currentPosition_y = init_position[1]
        self.final_position_x = final_position[0]
        self.final_position_y = final_position[1]
        self.velocity_x = 0
        self.velocity_y = 0
        self.DT = dt
        self.K = K
        self.passive = passive
        self.setVelocity(self.final_position_x, self.final_position_y)

    def getPosition(self):
        return (self.currentPosition_x,self.currentPosition_y)

    def getRadius(self):
        return self.radius


    def getVelocity(self):
        return (self.velocity_x, self.velocity_y)

    def setPosition(self, newPosition):
        self.currentPosition_x = newPosition[0]
        self.currentPosition_y = newPosition[1]

    def setVelocity(self,x_d, y_d):
        self.velocity_x = self.K * (x_d - self.currentPosition_x)
        self.velocity_y = self.K * (y_d - self.currentPosition_y)

    def calculateVelocity(self, fx, fy):
        return (self.K * (self.final_position_x - fx),self.K * (self.final_position_y - fy))

    def tangentPointTop(self,collisionCheck):
        robot1 = collisionCheck[0]
        pos1 = robot1.getPosition()
        x1 = pos1[0]
        y1 = pos1[1]
        r1 = robot1.getRadius()

        robot2 = collisionCheck[1]
        pos2 = robot2.getPosition()
        x2 = pos2[0]
        y2 = pos2[1]
        r2 = robot2.getRadius()

        distance = np.linalg.norm(np.asarray([x1, y1]) - np.asarray([x2,y2]))

        theta1 = np.arctan((y2-y1)/(x2-x1))
        theta2 = np.arcsin(r2/(distance))
        d = distance*np.cos(theta2)
        theta3 = np.arcsin(r1/d)

        L = r1/np.sin(theta3)
        xd = x1 + L*np.cos(theta1+theta2+theta3)
        yd = y1 + L*np.sin(theta1+theta2+theta3)
        return (xd, yd)

    def tangentPointBottom(self,collisionCheck):
        robot2 = collisionCheck[0]
        pos2 = robot2.getPosition()
        x2 = pos2[0]
        y2 = pos2[1]
        r2 = robot2.getRadius()

        robot1 = collisionCheck[1]
        pos1 = robot1.getPosition()
        x1 = pos1[0]
        y1 = pos1[1]
        r1 = robot1.getRadius()

        distance = np.linalg.norm(np.asarray([x1, y1]) - np.asarray([x2,y2]))

        theta1 = np.arctan((y2-y1)/(x2-x1))
        theta2 = np.arcsin(r2/(distance))
        d = distance*np.cos(theta2)
        theta3 = np.arcsin(r1/d)

        L = r1/np.sin(theta3)
        xd = x1 + L*np.cos(theta1+theta2+theta3)
        yd = y1 + L*np.sin(theta1+theta2+theta3)
        return (xd, yd)

    def tangentPoints(self,collisionCheck):
        robot1 = collisionCheck[0]
        pos1 = robot1.getPosition()
        x1 = pos1[0]
        y1 = pos1[1]
        r1 = robot1.getRadius()

        robot2 = collisionCheck[1]
        pos2 = robot2.getPosition()
        x2 = pos2[0]
        y2 = pos2[1]
        r2 = robot2.getRadius()

        d = r1+r2
        distance = np.linalg.norm(np.asarray([x1, y1]) - np.asarray([x2, y2]))
        print "--------_DEBUG------------"
        print d
        print distance
        theta1 = np.arcsin(d/distance)
        L = distance*np.cos(theta1)

        x3_1 = (d**2*(x1 - x2) + L**2*(-x1 + x2) + (x1 + x2)*((x1 - x2)**2 + (y1 - y2)**2) -
                np.sqrt(-((d - L)**2 - (x1 - x2)**2 - (y1 - y2)**2)*((d + L)**2 - (x1 - x2)**2 - (y1 - y2)**2)*(y1 - y2)**2))/(2*((x1 - x2)**2 + (y1 - y2)**2))

        y3_1 = (1/(2*((x1 - x2)**2 + (y1 - y2)**2)*(y1 - y2)))*(d**2*(y1 - y2)**2 + x1*(np.sqrt(-(d**4 + (-L**2 + (x1 - x2)**2 + (y1 - y2)**2)**2 -
            2*d**2*(L**2 + (x1 - x2)**2 + (y1 - y2)**2))*(y1 - y2)**2) - x2 *np.sqrt(-(d**4 + (-L**2 + (x1 - x2)**2 + (y1 - y2)**2)**2 -
         2*d**2*(L**2 + (x1 - x2)**2 + (y1 - y2)**2))*(y1 - y2)**2) + (y1 - y2)*(L**2*(-y1 + y2) + ((x1 - x2)**2 + (y1 - y2)**2)*(y1 + y2))))

        x3_2 = (d ** 2 * (x1 - x2) + L ** 2 * (-x1 + x2) + (x1 + x2) * ((x1 - x2) ** 2 + (y1 - y2) ** 2) +
                np.sqrt(-((d - L) ** 2 - (x1 - x2) ** 2 - (y1 - y2) ** 2) * (
                (d + L) ** 2 - (x1 - x2) ** 2 - (y1 - y2) ** 2) * (y1 - y2) ** 2)) / (
               2 * ((x1 - x2) ** 2 + (y1 - y2) ** 2))

        y3_2 = (1 / (2 * ((x1 - x2) ** 2 + (y1 - y2) ** 2) * (y1 - y2))) * (
        d ** 2 * (y1 - y2) ** 2 - x1 * (np.sqrt(-(d ** 4 + (-L ** 2 + (x1 - x2) ** 2 + (y1 - y2) ** 2) ** 2 -
                                                  2 * d ** 2 * (L ** 2 + (x1 - x2) ** 2 + (y1 - y2) ** 2)) * (
                                                y1 - y2) ** 2) + x2 * np.sqrt(
            -(d ** 4 + (-L ** 2 + (x1 - x2) ** 2 + (y1 - y2) ** 2) ** 2 -
              2 * d ** 2 * (L ** 2 + (x1 - x2) ** 2 + (y1 - y2) ** 2)) * (y1 - y2) ** 2) + (y1 - y2) * (
                                        L ** 2 * (-y1 + y2) + ((x1 - x2) ** 2 + (y1 - y2) ** 2) * (y1 + y2))))

        distance1 = np.linalg.norm(np.asarray([x3_1, y3_1]) - np.asarray([self.final_position_x, self.final_position_y]))
        distance2 = np.linalg.norm(np.asarray([x3_2, y3_2]) - np.asarray([self.final_position_x, self.final_position_y]))
        offset = 20
        if distance1 < distance2:
            if x1 < self.final_position_x:
                if y1 <= self.final_position_y:
                    print "test1"
                    return (x3_1 - offset, y3_1 + offset)
                else:
                    print "test2"
                    return (x3_1 - offset, y3_1 - offset)
            if x1 > self.final_position_x:
                if y1 <= self.final_position_y:
                    print "test3"
                    return (x3_1 + offset, y3_1 + offset)
                else:
                    print "test4"
                    return (x3_1 + offset, y3_1 - offset)

        else:
            if x1 < self.final_position_x:
                if y1 <= self.final_position_y:
                    print "test5"
                    return (x3_2 - offset, y3_2 - offset)
                else:
                    print "test6"
                    return (x3_2 - offset, y3_2 + offset)
            if x1 > self.final_position_x:
                if y1 <= self.final_position_y:
                    print "test7"
                    return (x3_2 + offset, y3_2 - offset)
                else:
                    print "test8"
                    return (x3_2 + offset, y3_2 + offset)


    def step(self, listCollision):
        if self.passive or len(listCollision) == 0:
            self.setVelocity(self.final_position_x, self.final_position_y)
            self.currentPosition_x = self.currentPosition_x + self.velocity_x*self.DT
            self.currentPosition_y = self.currentPosition_y + self.velocity_y*self.DT
        else:
            print "test2"
            collisionCheck = listCollision[0]

            robot1 = collisionCheck[0]
            pos1 = robot1.getPosition()
            vel1 = robot1.getVelocity()
            x1 = pos1[0]
            y1 = pos1[1]
            vx1 = vel1[0]
            vy1 = vel1[1]

            robot2 = collisionCheck[1]
            pos2 = robot2.getPosition()
            vel2 = robot1.getVelocity()
            x2 = pos2[0]
            y2 = pos2[1]
            vx2 = vel2[0]
            vy2 = vel2[1]
            collisionFlag = False
            dtau = self.DT
            for i in range(1,41):
                print "test3"

                (x1, y1) = [x1 + vx1*dtau, y1 + vy1*dtau]
                (x2, y2) = [x2 + vx2 * dtau, y2 + vy2 * dtau]
                distance = np.linalg.norm(np.asarray([x1, y1]) - np.asarray([x2,y2]))
                distance2 = dst.euclidean((x1, y1),(x2,y2))
                print "-------------------Distance------------------"
                print distance
                print distance2
                if distance < robot1.getRadius()+robot2.getRadius():
                    print "test4"
                    collisionFlag = True
                else:
                    (vx1, vy1) = robot1.calculateVelocity(x1, y1)
                    (vx2, vy2) = robot2.calculateVelocity(x2, y2)
            if collisionFlag:
                newPosition = self.tangentPoints(collisionCheck)
                print "NEW POSITIONS"
                print newPosition
                self.setVelocity(newPosition[0], newPosition[1])
                self.currentPosition_x = self.currentPosition_x + self.velocity_x * self.DT
                self.currentPosition_y = self.currentPosition_y + self.velocity_y * self.DT
            else:
                self.setVelocity(self.final_position_x, self.final_position_y)
                self.currentPosition_x = self.currentPosition_x + self.velocity_x * self.DT
                self.currentPosition_y = self.currentPosition_y + self.velocity_y * self.DT


