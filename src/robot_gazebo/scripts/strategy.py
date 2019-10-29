import constants 
import pickle
import search
import time

destination = None

class Location:
    def __init__(self, x, y):
        self.x = x
        self.y = y


class Node:
    def __init__(self, name, paths, location, state=constants.NODE_AVAILABLE):
        self.name = name
        self.paths = paths
        # self.toExploreCost = toExploreCost      #cost of path to destination from current position
        self.location = Location(location[0], location[1])
        self.state = [state] * constants.RANGE_SAMPLES
        self.buffer = False

    def setState(self, state, index):
        self.state[index] = state
        self.setBuffer(False)

    def getState(self):
        wall = 0
        for i in self.state:
            if i == constants.NODE_WALL:
                wall += 1

        return wall < constants.RANGE_SAMPLES / 2.0

    def setBuffer(self, buffer):
        self.buffer = buffer

    def isBuffer(self):
        return self.buffer

    def isAvailable(self):
        return self.getState() and not self.isBuffer()

    def getName(self):
        return self.name

    def getNeighbours(self):
        neighbours = []
        for i in self.paths:
            neighbours.append(self.paths[i].getEnd(self))

        return neighbours

    def getPaths(self):
        return self.paths

    def getToExploreCost(self):
        global destination
        endNode = destination
        return ((endNode[0] - self.location.x) ** 2 + (endNode[1] - self.location.y) ** 2) ** 0.5

    def addPath(self, path):
        end = path.getEnd(self).getName()

        if end not in self.paths and end is not None:
            self.paths[end] = path

    def createBuffer(self):
        for i in self.paths:
            path = self.paths[i]
            if path.getState():
                path.setBuffer(True)

            head = path.getEnd(self)
            if head.getState():
                head.setBuffer(True)
            
            for j in head.paths:
                pathN = head.paths[j]
                if pathN.getState():
                    pathN.setBuffer(True)

        return True

class Path:
    def __init__(self, pathId, ends, cost = 1, state=constants.NODE_AVAILABLE):
        self.pathId = pathId
        self.tail = None
        self.parentPath = None
        self.ends = ends
        self.cost = cost  # path cost
        self.exploredCost = 0
        self.state = [state] * constants.RANGE_SAMPLES
        self.buffer = False

    def setState(self, state):
        self.state = state

    def getState(self):
        wall = 0
        for i in self.state:
            if i == constants.NODE_WALL:
                wall += 1

        return wall < constants.RANGE_SAMPLES / 2.0

    def setBuffer(self, buffer):
        self.buffer = buffer

    def isBuffer(self):
        return self.buffer

    def isAvailable(self):
        return self.getState() and not self.isBuffer()

    def getPathId(self):
        return self.pathId

    def getEnd(self, end):
        if end.name in self.ends:
            keys = list(self.ends.keys())
            index = keys.index(end.name) - 1
            return self.ends.get(keys[index])

        return None

    def getHead(self):
        if self.tail is None:
            return None

        return self.getEnd(self.tail)

    def getTail(self):
        return self.tail

    def getParentPath(self):
        return self.parentPath

    def getCost(self):
        head = self.ends[list(self.ends.keys())[0]]
        tail = self.ends[list(self.ends.keys())[-1]]
        return ((head.location.x - tail.location.x) ** 2 + (head.location.y - tail.location.y) ** 2) ** 0.5

    def getTotalCost(self):  # total cost calculated from the tail of the path since the path isn't explored yet
        return self.getHead().getToExploreCost() + self.getExploredCost()

    def getExploredCost(self):
        return self.exploredCost

    def setExploredCost(self):
        if self.parentPath is not None:
            self.exploredCost = self.getParentPath().getExploredCost() + self.getCost()
        else:
            self.exploredCost = self.getCost()

    def setTail(self, tail):
        self.tail = tail

    def setParentPath(self, parentPath):
        self.parentPath = parentPath


class Strategy:
    STRATEGY = {'UCS': 1, 'DFS': 2, 'BFS': 3, 'A*': 4}

    def __init__(self, strategy):
        self.strategy = strategy

    def nodeToExpand(self, states):
        if self.strategy == self.STRATEGY['DFS']:
            return states.pop()

        elif self.strategy == self.STRATEGY['BFS']:
            return states.pop(0)

    def pathToExpand(self, states):
        if self.strategy == self.STRATEGY['DFS']:
            return states.pop()

        elif self.strategy == self.STRATEGY['BFS']:
            return states.pop(0)

        elif self.strategy == self.STRATEGY['UCS']:
            minCost = states[0].getExploredCost()
            index = 0

            for i in range(len(states)):
                if states[i].getExploredCost() < minCost:
                    minCost = states[i].getExploredCost()
                    index = i
            return states.pop(index)

        elif self.strategy == self.STRATEGY['A*']:
            minCost = states[0].getTotalCost()
            index = 0

            for i in range(len(states)):
                if states[i].getTotalCost() < minCost:
                    minCost = states[i].getTotalCost()
                    index = i
            return states.pop(index)


class Problem:
    def __init__(self, problem, initialState, goalState):
        global destination
        self.problem = problem
        self.initialState = initialState
        self.goalState = goalState
        destination = self.goalState.getName()

    def getInitialState(self):
        return self.initialState

    def actions(self, currentState):
        return currentState.getNeighbours()

    def pathActions(self, currentPath):
        actions = []

        h = currentPath.getHead()
        paths = h.getPaths()

        for x in paths:
            if paths[x].getTail() is None and paths[x].getParentPath() is None:
                paths[x].setParentPath(currentPath)
                paths[x].setTail(currentPath.getHead())
                paths[x].setExploredCost()
            actions.append(paths[x])

        return actions

    def goalTest(self, currentState):
        return currentState == self.goalState


class Search:
    def getPath(self, node):
        path = [node.getNodeId()]
        parent = node.getParent()
        while parent is not None:
            path.append(parent.getNodeId())
            parent = parent.getParent()

        path.reverse()

        return path

    def treeSearch(self, problem, strategy):
        frontier = [problem.getInitialState()]

        while True:
            if not frontier:
                return False

            nodeToExpand = strategy.nodeToExpand(frontier)

            if problem.goalTest(nodeToExpand):
                return nodeToExpand

            for i in problem.actions(nodeToExpand):
                frontier.append(i)

    def graphSearch(self, problem, strategy):
        frontier = [x for x in problem.getInitialState().getPaths()]
        # explored = {}
        explored = []

        while True:
            if not frontier:
                return False

            nodeToExpand = strategy.nodeToExpand(frontier)
            if nodeToExpand.getName() not in explored:
                explored[nodeToExpand.getName()] = nodeToExpand
            else:
                continue

            if problem.goalTest(nodeToExpand):
                return nodeToExpand

            for i in problem.actions(nodeToExpand):
                for j in i.getPaths():
                    if i.getName() not in explored and i not in frontier:
                        frontier.append(i)

    def ngraphSearch(self, problem, strategy):
        frontier = []
        paths = problem.getInitialState().getPaths()
        for x in paths:
            if paths[x].isAvailable():
                paths[x].setTail(problem.getInitialState())
                paths[x].setExploredCost()
                frontier.append(paths[x])

        exploredNodes = {}
        explored = {}
        
        while True:
            if not frontier:
                return False

            pathToExpand = strategy.pathToExpand(frontier)

            if pathToExpand.getPathId() not in explored and pathToExpand.getHead().getName() not in exploredNodes:
                explored[pathToExpand.getPathId()] = pathToExpand
            else:
                continue

            if pathToExpand.getHead().getName() not in exploredNodes:
                exploredNodes[pathToExpand.getHead().getName()] = pathToExpand.getHead()

            if problem.goalTest(pathToExpand.getHead()):
                return pathToExpand

            for i in problem.pathActions(pathToExpand):
                if i.getPathId() not in explored and i.isAvailable() and i not in frontier and i.getTail() not in exploredNodes:
                    frontier.append(i)
