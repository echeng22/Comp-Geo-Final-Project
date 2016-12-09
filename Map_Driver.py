import matplotlib.pyplot as plt
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

    def update(self):
        print "Test"

    def drawMap(self, robotPos):
        plt.figure(1)
        axes = plt.gca()
        x_list = [x for [x, y] in robotPos]
        y_list = [y for [x, y] in robotPos]
        plt.scatter(x_list, y_list, marker = ".")

        for i in range(len(robotPos)):
            circ = plt.Circle((x_list[i],y_list[i]), radius=10, color='b', fill=False)
            axes.add_patch(circ)
        plt.show()

    def mapClear(self):
        plt.cla()
        self.setFigure()



def main():
    map = MapDriver([], [-200,200,-200,200],.01)
    robotPos = [[-150, 0], [0, 0], [150, 0], [100, 0]]
    for i in range(100):
        map.mapClear()
        map.drawMap(robotPos)
        robotPos = [[x,y+1] for [x,y] in robotPos]
        print robotPos
        plt.pause(.05)

if __name__ == "__main__":
    main()