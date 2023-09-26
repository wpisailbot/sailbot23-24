"""
This file will be used for high level pathing, we will define all of the subclasses, algorithms and the
	necessary I/O from this program. The main purpose of this is to just use the Astar class, init it
	with the json file + waypoints and it will output the corresponding waypoints the boat should sail to

"""

"""
Imports
"""
import json, pygame, time, sys, platform, math, bisect, copy, itertools
from pygame.locals import KEYDOWN, K_q

_VARS = {'surf': False, 'gridMult': 20,
         'gridOrigin': (10, 10), 'lineWidth': 2}
blue, black, green, red, lavender = ((0, 128, 255), (0, 0, 0), (0, 204, 0), (204, 0, 0), (229, 204, 255))
"""
Subclasses
"""

################
# Coordinate Class
################
class Coordinate():

    def __init__(self, x: int, y: int, lat: int, lng: int, blocked: bool):
        self.x = x
        self.y = y
        self.lat = lat
        self.lng = lng
        self.blocked = blocked
        self.partOfPath = False
        self.weight = 0

    def __lt__(self, other):
        return self.weight < other.weight

    def __gt__(self, other):
        return self.weight > other.weight

    def __str__(self):
        return 'weight - {} x: {}, y: {}, blocked: {}, inPath: {};'.format(self.weight, self.x, self.y, self.blocked, self.partOfPath)

    def __repr__(self):
        return str(self)

################
# GPS Line Class
################
class Line():

    def __init__(self, startLat: int, startLng: int, endLat: int, endLng: int):
        self.startLat = startLat
        self.startLng = startLng
        self.endLat = endLat
        self.endLng = endLng

################
# Path Line Class
################
class Path():
    def __init__(self, startNode):
        self.path = [startNode]
    def __str__(self):
        return str(self.path[0])
    def __len__(self):
        return len(self.path)
    def __lt__(self, other):
        return self.weight() < other.weight()
    def __gt__(self, other):
        return self.weight() > other.weight()
    def append(self, node):
        self.path.append(node)
    def path(self):
        return self.path
    def top(self):
        return self.path[0]
    def weight(self):
        return self.path[0].weight
    def insert(self, node):
        return self.path.insert(0, node)

################
# Grid Class
################

#this will need to be fleshed out as we determine how to create and maintain the grid
#-- will it have the weights stored in it?
#-- will it just be 0s and 1s for navigatable vs not?

class Grid():

    def __init__(self):
        self.gridNodes = []
        self.latLines = []
        self.lngLines = []

        self.squareRad = 0.001; # 10 meters is 4th decimal places I think? (near the equator it is 11)

    def get(self, x: int, y: int):
        return self.gridNodes[x * _VARS['gridCellsY'] + y]

    def set(self, x: int, y: int, val):
        self.gridNodes[x * _VARS['gridCellsY'] + y] = val

    def getGrid(self):
        return self.gridNodes

    def importMapGridFile(self, jsonFilePath):
        with open(jsonFilePath) as file:
            data = json.load(file)

        self.gridNodes = list(itertools.chain.from_iterable(data['gridCenters'])) #gridcenters come in a list of lists so we have to collapse

        self.gridNodes = [Coordinate(node['index']['x'], node['index']['y'], node['gps']['lat'], node['gps']['lng'], node['flags']['blocked']) for node in self.gridNodes]

        self.latLines = data['latitudeLines']
        self.lngLines = data['longitudeLines']
        
        _VARS['gridCellsX'] = self.gridNodes[-1].x + 1
        _VARS['gridCellsY'] = self.gridNodes[-1].y + 1

    def checkEvents(self):
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                sys.exit()
            elif event.type == KEYDOWN and event.key == K_q:
                pygame.quit()
                sys.exit()

    def drawGrid(self):
        CELL_SIZE = 40  # Not to be confused with SCREENSIZE
        cont_x, cont_y = 10, 10  # TOP LEFT OF CONTAINER
        xCells = _VARS['gridCellsX']
        yCells = _VARS['gridCellsY']

        # DRAW Grid Border:
        # TOP lEFT TO RIGHT
        pygame.draw.line(
          _VARS['surf'], (black),
          (cont_x, cont_y),
          (CELL_SIZE * xCells + cont_x, cont_y), 2)
        # # BOTTOM lEFT TO RIGHT
        pygame.draw.line(
          _VARS['surf'], (black),
          (cont_x, CELL_SIZE * yCells + cont_y),
          (CELL_SIZE * xCells + cont_x, CELL_SIZE * yCells + cont_y), 2)
        # # LEFT TOP TO BOTTOM
        pygame.draw.line(
          _VARS['surf'], (black),
          (cont_x, cont_y),
          (cont_x, cont_y + CELL_SIZE * yCells), 2)
        # # RIGHT TOP TO BOTTOM
        pygame.draw.line(
          _VARS['surf'], (black),
          (CELL_SIZE * xCells + cont_x, cont_y),
          (CELL_SIZE * xCells + cont_x, CELL_SIZE * yCells + cont_y), 2)


        # VERTICAL DIVISIONS: (0,1,2) for grid(3) for example
        for i in range(xCells):
            pygame.draw.line(
               _VARS['surf'], (black),
               (cont_x + (CELL_SIZE * i), cont_y),
               (cont_x + (CELL_SIZE * i), CELL_SIZE * yCells + cont_y), 2)
        # # HORIZONTAl DIVISIONS
        for i in range(yCells):
            pygame.draw.line(
              _VARS['surf'], (black),
              (cont_x, cont_y + (CELL_SIZE * i)),
              (cont_x + CELL_SIZE * xCells, cont_y + (CELL_SIZE * i)), 2)

    def drawCells(self):
        CELL_SIZE = 40  # Not to be confused with SCREENSIZE
        cellBorder = 10
        celldimX = celldimY = CELL_SIZE - (cellBorder*2)


        for node in self.gridNodes:
            if any(way[0] == node.x and way[1] == node.y for way in waypoints):
                pygame.draw.rect(
                    _VARS['surf'], green,
                    (int(_VARS['gridOrigin'][0] + (celldimX * node.x) + cellBorder + (2 * node.x * cellBorder) + _VARS['lineWidth']/2),
                    int(_VARS['gridOrigin'][1] + (celldimY * node.y) + cellBorder + (2 * node.y * cellBorder) + _VARS['lineWidth']/2),
                    celldimX, celldimY)
                )
            elif node.partOfPath:
                pygame.draw.rect(
                    _VARS['surf'], lavender,
                    (int(_VARS['gridOrigin'][0] + (celldimX * node.x) + cellBorder + (2 * node.x * cellBorder) + _VARS['lineWidth']/2),
                    int(_VARS['gridOrigin'][1] + (celldimY * node.y) + cellBorder + (2 * node.y * cellBorder) + _VARS['lineWidth']/2),
                    celldimX, celldimY)
                )
            elif node.blocked:
                pygame.draw.rect(
                    _VARS['surf'], red,
                    (int(_VARS['gridOrigin'][0] + (celldimX * node.x) + cellBorder + (2 * node.x * cellBorder) + _VARS['lineWidth']/2),
                    int(_VARS['gridOrigin'][1] + (celldimY * node.y) + cellBorder + (2 * node.y * cellBorder) + _VARS['lineWidth']/2),
                    celldimX, celldimY)
                )


    def visAstar(self):
        if not self.gridNodes:
            print('Yall gotta import something first')
            return 0

        if not platform.system() == 'Linux':
            pygame.init() # draw grid stuff stolen from: https://betterprogramming.pub/making-grids-in-python-7cf62c95f413
            _VARS['surf'] = pygame.display.set_mode((600, 400))

            while True:
                self.checkEvents()
                _VARS['surf'].fill(blue)
                self.drawGrid()
                self.drawCells()
                pygame.display.update()
        else:
            print('You\'re on a linux dist so you\'re probably on the jetson, so im not gonna let this work 凸(｀⌒´メ)凸')


"""
Main Astar class with all the important functions
"""


class Astar():

    def __init__(self, grid: Grid, wind_direction: int, start: Coordinate, end: Coordinate):

        ## required params
        self.grid = grid
        self.wind_direction = wind_direction
        self.start = Path(start)
        self.end = end

        ## globals
        self.lowestPath = []
        self.pathsToCheck = []
        self.lowestPathWeight = 999999999

        ## constants
        self.currentDirectionWeight = 20
        self.nodeWeightMultiplier = 10000


    # Heuristic stuff:
    #   - node weight: will be total distance from current node to the end
    #   - edge weight: will be current direction of travel + current wind direction vs edge direction
    #
    # Algo Stuff:
    #   - F(n) = g(n) + h(n)
    #       - g(n) = edge weight
    #       - h(n) = dest node weight
    #
    #   - Evaluate all pathsToCheck from a node, then start continuing on the lowest cost one
    #   - once you find the final node, keep going through your list until the value you have is lower than
    #       the top of the list
    def runAstar(self):

        self.resetPaths(self.start)
        counter = 0

        while (len(self.pathsToCheck) > 0 and self.pathsToCheck[0].weight() <= self.lowestPathWeight and counter < 1000):
            # [print('c: ', i, ' - ', path) for i, path in enumerate(self.pathsToCheck)]
            self.findCheapestPath()
            counter += 1

        # [print(path.path) for path in self.pathsToCheck]
        return self.lowestPath


    # Reset globals
    def resetPaths(self, start):
        self.pathsToCheck = [start] # hardcoded start
        self.lowestPathWeight = 999999999
        self.lowestPath = []


    def findCheapestPath(self):
        curPath = self.pathsToCheck.pop(0)
        pathNode = curPath.top()
        pathWeight = pathNode.weight
        pathNodeNeighbors = self.findNeightbors(pathNode.x, pathNode.y)

        counter = 0
        for neighbor in pathNodeNeighbors:
            # print('counter: ', counter, ' - path: ', curPath)
            neighborWeight = self.findNodeWeight(neighbor, self.end) # HAVE TO CHANGE THE PATHS FROM THE PATH OBJECT
            edgeWeight = self.findEdgeWeight(curPath, neighbor, self.wind_direction)
            totalWeight = pathWeight + neighborWeight + edgeWeight
            tempPath = copy.deepcopy(curPath)

            insert = copy.deepcopy(neighbor) 
            insert.weight = totalWeight
            tempPath.insert(insert) #insert at the first position
            self.addPath(tempPath)
            counter += 1

    def addPath(self, path):
        pathNode = path.path[0]
        if (pathNode.x == self.end.x and pathNode.y == self.end.y): ## if we get to the destination
            if (pathNode.weight < self.lowestPathWeight): ## keep this new path if its good
                self.lowestPathWeight = pathNode.weight
                self.lowestPath = path
             ## discard if we have a better one

        # index = 0
        # for i in range(len(self.pathsToCheck)):
        #     if (self.pathsToCheck[i][0] > path[0]):
        #         index = i
        #         break
        ## console.log('pre', pathsToCheck)
        # self.pathsToCheck.insert(index, path)  ## just put the path in the global
        # self.pathsToCheck.sort(key = lambda path: path[0].weight)  # sort it so we have the shortest path on top
        ## console.log('post',pathsToCheck)

        bisect.insort_left(self.pathsToCheck, path)


    # calculate the distance from the destination for the current node
    def findNodeWeight(self, node, dest):
        # got the converter to meters here: https:#www.movable-type.co.uk/scripts/latlong.html
        R = 6371e3  # metres
        φ1 = node.lat * math.pi/180  # φ, λ in radians
        φ2 = dest.lat * math.pi/180
        Δφ = (dest.lat-node.lat) * math.pi/180
        Δλ = (dest.lng-node.lng) * math.pi/180

        a = math.sin(Δφ/2) * math.sin(Δφ/2) + math.cos(φ1) * math.cos(φ2) * math.sin(Δλ/2) * math.sin(Δλ/2)
        c = 2 * math.atan2(math.sqrt(a), math.sqrt(1-a))

        meters = R * c  # in metres
        return meters
        ## return nodeWeightMultiplier * Math.pow(node.lat - dest.lat, 2) + Math.pow(node.lng - dest.lng, 2)
        ## return nodeWeightMultiplier * Math.max(Math.abs(node.lat - dest.lat) + Math.abs(node.lng - dest.lng))


    ## calculate the edge weight between the start node and dest node
    ##     So we calculate if it is staying the same direction, and direction with respect to wind
    def findEdgeWeight(self, prevPath, dest, windDirection):
        weight = 0

        if len(prevPath) > 1:
            nodeA = prevPath.path[1]
            nodeB = prevPath.path[0]
            nodeC = dest

            slopeABy = (nodeA.y - nodeB.y)
            slopeABx = (nodeA.x - nodeB.x)
            slopeBCy = (nodeB.y - nodeC.y)
            slopeBCx = (nodeB.x - nodeC.x)

            if (slopeABy == slopeBCy and slopeABx == slopeBCx): # find out if nodes are collinear by seeing if slopes are equatl
                weight += self.currentDirectionWeight

            startNode = prevPath.path[0]
        # TODO ADD DIRECTION BASED ON WIND
        return weight


    # find and returns an array of the neighbors of a given point
    def findNeightbors(self, x, y):
        neighbors = []
        maxX = _VARS['gridCellsX'] - 1
        maxY = _VARS['gridCellsY'] - 1

        if (x != 0): # left
            neighbors.append(self.grid.get(x - 1, y))
        if (x != maxX): # right
            neighbors.append(self.grid.get(x + 1, y))
        if (x != 0 and y != 0): #  top left
            neighbors.append(self.grid.get(x - 1, y - 1))
        if (x != maxX and y != 0): # top right
            neighbors.append(self.grid.get(x + 1, y - 1))
        if (x != 0 and y != maxY): #  bot left
            neighbors.append(self.grid.get(x - 1, y + 1))
        if (x != maxX and y != maxY): # bot right
            neighbors.append(self.grid.get(x + 1, y + 1))
        if (y != 0): # top
            neighbors.append(self.grid.get(x, y - 1))
        if (y != maxY): # bottom
            neighbors.append(self.grid.get(x, y + 1))

        return neighbors


### Temp Test code
g = Grid()
g.importMapGridFile('../../resource/mapgrid.json');

waypoints = [(3, 6), (7, 6), (5, 4), (0, 4)]
FullBoatPath = []
one = Astar(g, 0, g.get(*waypoints[0]), g.get(*waypoints[1])).runAstar().path
two = Astar(g, 0, g.get(*waypoints[1]), g.get(*waypoints[2])).runAstar().path
three = Astar(g, 0, g.get(*waypoints[2]), g.get(*waypoints[3])).runAstar().path
FullBoatPath.append(one)
FullBoatPath.append(two)
FullBoatPath.append(three)

FullBoatPath =  list(itertools.chain.from_iterable(FullBoatPath))

for node in FullBoatPath:
    node.partOfPath = True
    g.set(node.x, node.y, node)


print(FullBoatPath)
# print(aaa.pathsToCheck)
g.visAstar()
