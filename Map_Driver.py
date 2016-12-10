import matplotlib.pyplot as plt
import numpy as np
import Robot


class MapDriver:
    def __init__(self, robotList, worldsize,timeStep):
        self.listRobots = robotList
        self.timeSteep = timeStep
        self.mapFigure = plt.figure(1)
        self.worldSize = worldsize
        # axes = plt.gca()
        # plt.ion()
        # axes.set_aspect('equal')
        # axes.set_xlim([self.worldSize[0], self.worldSize[1]])
        # axes.set_ylim([self.worldSize[2], self.worldSize[3]])


    def setFigure(self):
        axes = plt.gca()
        plt.ion()
        axes.set_aspect('equal')
        axes.set_xlim([self.worldSize[0],self.worldSize[1]])
        axes.set_ylim([self.worldSize[2], self.worldSize[3]])

    def update(self, listCollision):
        listRobotPos = []
        for robot in self.listRobots:
            robot.step(listCollision)
            listRobotPos.append(robot.getPosition())
        self.drawMap(listRobotPos)

    def drawMap(self, robotPos):
        plt.figure(1)
        self.setFigure()
        axes = plt.gca()
        x_list = [x for [x, y] in robotPos]
        y_list = [y for [x, y] in robotPos]
        for i in range(len(robotPos)):
            circ = plt.Circle((x_list[i],y_list[i]), radius=10, color='b', fill=False)
            axes.add_patch(circ)
        plt.scatter(x_list, y_list, marker=".")
        plt.show()
        plt.pause(.1)

    def mapClear(self):
        plt.cla()
        self.setFigure()

    def checkCollisions(self):
        collisionCheck = []
        for i in range(len(self.listRobots)-1):
            robot1 = self.listRobots[i]
            pos1 = np.asarray(robot1.getPosition())
            for j in range(i+1, len(self.listRobots)):
                robot2 = self.listRobots[j]
                pos2 = np.asarray(robot2.getPosition())
                distance = np.linalg.norm(np.asarray(pos1)-np.asarray(pos2))
                if distance < (robot1.getRadius() + robot2.getRadius() + 40):
                    collisionCheck.append([robot1, robot2])
        return collisionCheck





def main():
    # Working RUN 1
    # robot1 = Robot.robot(10, [200, -200], [-200, 200], 5, .005, False)
    # robot2 = Robot.robot(10, [200, 200], [-200, -200], 5, .005, True)
    # robot1 = Robot.robot(10, [200, -200], [-200, 200], 5, .005, False)
    # robot2 = Robot.robot(10, [0, 200], [-40, -200], 5, .005, True)
    # robot1 = Robot.robot(10, [200, -200], [-200, 200], 5, .005, False)
    # robot2 = Robot.robot(10, [0, 200], [-40, -200], 5, .005, True)
    # robot3 = Robot.robot(10, [-200, -200], [20, 200], 5, .005, True)
    robot1 = Robot.robot(10, [200, -200], [-200, 200], 5, .005, False)
    robot2 = Robot.robot(10, [0, 200], [-40, -200], 5, .005, True)
    robot3 = Robot.robot(10, [-200, -200], [20, 200], 5, .005, True)
    robot4 = Robot.robot(10, [-200, 50], [200, -70], 5, .003, True)
    map = MapDriver([robot1,robot2, robot3, robot4], [-200,200,-200,200],.01)
    time = 700
    t_count = 0
    while t_count < time:
        print t_count
        listCollision = []
        listCollision = map.checkCollisions()
        print listCollision
        map.update(listCollision)

        map.mapClear()

        t_count = t_count + 1


if __name__ == "__main__":
    main()