#!/usr/bin/env python






class robot:
    
    def __init__(self,radius, init_position, final_position, K):
        self.radius = radius
        self.init_position.x = init_position(1)
        self.init_position.y = init_position(2)
        self.final_position.x = final_position(1)
        self.final_position.y = final_position(2)
        self.K = K



class move:

    dt = 0.1

    def __init__(self,robot1,robot2,t_0,t_f):
        self.robot1 = robot1
        self.robot2 = robot2
        self.t_0 = t_0
        self.t_f = t_0


    def velocity(self,x_curr,y_curr,x_d,y_d):
        velocity_x = -robot1.K*(x_curr - x_d)
        velocity_y = -robot1.K*(y_curr - y_d)

        return velocity_x,velocity_y

        
    def has_collided(self,x1,y1,x2,y2):
        if sqrt((x1 - x1)**2 + (x1 - x1)**2) <= (self.robot1.radius + self.robot2.radius):
            return true
        else:
            return false


    def step(self,x1_start,y1_start,x2_start,y2_start,x1_end,y1_end,x2_end,y2_end,v_passive):
        t_curr = self.t_0
        x1_curr = x1_start
        y1_curr = y1_start
        x1_curr = x2_start
        y2_curr = y2_start
        x1_end = x1_end
        y1_end = y1_end
        x2_end = x2_end
        y2_end = y2_end
            
        while t_curr < self.t_f:
            #We can find the velocity for the active robot here as follows
            curr_vel1 = velocity(self,x1_curr,y1_curr,x1_end,y1_end)
            curr_vel2 = velocity(self,x2_curr,y2_curr,x2_end,y2_end)

            #Then we can upgrade the position of the active robot
            x1_curr = x_curr + curr_vel1(1)*dt
            y1_curr = x_curr + curr_vel1(2)*dt

            #Where the velocity and positions of the passive robot are
            x2_curr = x2_curr + v_passive(1)
            y2_curr = y2_curr + v_passive(2)

            #now that we have the positions of both robots at this moment in time then we can check for collision
            collision = has_collided(self,x1_curr,y1_curr,x2_curr,y2_curr)
            if not collision:
                #we can cut to the next time step normally
                self.t_curr = self.t_curr + dt
            else:
                #If a collision has happened then the following occurs: 



        
