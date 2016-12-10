import numpy as np

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

    def tangentPoint(self,collisionCheck):
        robot1 = collisionCheck[0]
        pos1 = robot1.getPosition()
        vel1 = robot1.getVelocity()
        x1 = pos1[0]
        y1 = pos1[1]
        r1 = robot1.getRadius()

        robot2 = collisionCheck[1]
        pos2 = robot2.getPosition()
        vel2 = robot1.getVelocity()
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

    def step(self, listCollision):
        if self.passive or len(listCollision) == 0:
            self.setVelocity(self.final_position_x, self.final_position_y)
            self.currentPosition_x = self.currentPosition_x + self.velocity_x*self.DT
            self.currentPosition_y = self.currentPosition_y + self.velocity_y*self.DT
        else:
            print "test2"
            tau = 2
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
            for i in range(1,21):
                print "test3"
                dtau = .1
                (x1, y1) = [x1 + vx1*dtau, y1 + vy1*dtau]
                (x2, y2) = [x2 + vx2 * dtau, y2 + vy2 * dtau]
                distance = np.linalg.norm(np.asarray([x1, y1]) - np.asarray([x2,y2]))
                print "-------------------Distance------------------"
                print distance
                if distance < robot1.getRadius()+robot2.getRadius() + 50:
                    collisionFlag = True
                    updatePosition = self.tangentPoint(collisionCheck)
                    print "------------------Update Position------------------------"
                    print updatePosition
                    self.setVelocity(updatePosition[0], updatePosition[1])
                    self.currentPosition_x = self.currentPosition_x + self.velocity_x * self.DT
                    self.currentPosition_y = self.currentPosition_y + self.velocity_y * self.DT
                else:
                    (vx1, vy1) = robot1.calculateVelocity(x1, y1)
                    (vx2, vy2) = robot2.calculateVelocity(x2, y2)
            if not collisionFlag:
                self.setVelocity(self.final_position_x, self.final_position_y)
                self.currentPosition_x = self.currentPosition_x + self.velocity_x * self.DT
                self.currentPosition_y = self.currentPosition_y + self.velocity_y * self.DT









    # def has_collided(self, x1, y1, x2, y2):
    #     if sqrt((x1 - x1) ** 2 + (x1 - x1) ** 2) <= (self.robot1.radius + self.robot2.radius):
    #         return true
    #     else:
    #         return false

    # def step(self, x1_start, y1_start, x2_start, y2_start, x1_end, y1_end, x2_end, y2_end, v_passive):
    #     t_curr = self.t_0
    #     x1_curr = x1_start
    #     y1_curr = y1_start
    #     x1_curr = x2_start
    #     y2_curr = y2_start
    #     x1_end = x1_end
    #     y1_end = y1_end
    #     x2_end = x2_end
    #     y2_end = y2_end
    #
    #     while t_curr < self.t_f:
    #         # We can find the velocity for the active robot here as follows
    #         curr_vel1 = velocity(self, x1_curr, y1_curr, x1_end, y1_end)
    #         curr_vel2 = velocity(self, x2_curr, y2_curr, x2_end, y2_end)
    #
    #         # Then we can upgrade the position of the active robot
    #         x1_curr = x_curr + curr_vel1(1) * dt
    #         y1_curr = x_curr + curr_vel1(2) * dt
    #
    #         # Where the velocity and positions of the passive robot are
    #         x2_curr = x2_curr + v_passive(1)
    #         y2_curr = y2_curr + v_passive(2)
    #
    #         # now that we have the positions of both robots at this moment in time then we can check for collision
    #         collision = has_collided(self, x1_curr, y1_curr, x2_curr, y2_curr)
    #         if not collision:
    #             # we can cut to the next time step normally
    #             self.t_curr = self.t_curr + dt
    #         else:
    #     # If a collision has happened then the following occurs:





