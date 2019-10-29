import numpy as np
import pickle
import strategy
import time
import constants as Constant

def readFileToObj(fileName):
    return pickle.load(open(fileName, 'rb'))

def writeObjToFile(fileName, obj):
    f = open(fileName, 'wb')
    pickle.dump(obj, f)

def convertPathToPlotArr(arr):
    newArr = [[],[]]
    if arr is not None:
        for ele in arr:
            newArr[0].append(ele[0])
            newArr[1].append(ele[1])

    return newArr

def convertGraphToPlotArr(graph, nodeState):
    arr = [[],[]]

    if graph is not None:
        for i in graph:
            node = graph[i]
            if node.getState() == nodeState:
                arr[0].append(i[0])
                arr[1].append(i[1])                

    return arr

def createEmptyNodes(row, col, interval):
    nodeNeighbourLst = {}

    for y in range(row):
        for x in range(col):
            neighbours = []

            node = getXAndY(x, y, row, col, interval)

            if x + 1 < col: addNeighbour(neighbours, getXAndY(x + 1, y, row, col, interval)) 
            if x - 1 >= 0: addNeighbour(neighbours, getXAndY(x - 1, y, row, col, interval)) 
            if y + 1 < row: addNeighbour(neighbours, getXAndY(x, y + 1, row, col, interval)) 
            if y - 1 >= 0: addNeighbour(neighbours, getXAndY(x, y - 1, row, col, interval)) 
            
            if x + 1 < col and y + 1 < row: addNeighbour(neighbours, getXAndY(x + 1, y + 1, row, col, interval)) 
            if x - 1 >= 0 and y - 1 >= 0: addNeighbour(neighbours, getXAndY(x - 1, y - 1, row, col, interval)) 
            if y + 1 < row and x - 1 >= 0: addNeighbour(neighbours, getXAndY(x - 1, y + 1, row, col, interval)) 
            if y - 1 >= 0 and x + 1 < col: addNeighbour(neighbours, getXAndY(x + 1, y - 1, row, col, interval))

            nodeNeighbourLst[node] = neighbours

    return nodeNeighbourLst

def addNeighbour(neighboursLst, node):
    if node is not None:
        neighboursLst.append(node)

def getXAndY(x, y, row, col, interval):
    if x < col/2:
        nx = -interval * ((col/2) - x)
    else:
        nx = interval * (x - (col/2))

    if y < row/2:
        ny = -interval * ((row/2) - y)
    else:
        ny = interval * (y - (row/2))    

    return (nx, ny)


def getXAndYFilter(arrGiven, x, y, row, col, interval, selectOnly = 1):
    if arrGiven[y][x] == selectOnly:
        return getXAndY(x, y, row, col, interval)

    return None

def createGraph(nodes):
    graph = nodes
    nodeObjs = {}
    for i in graph:
        nodeObjs[i] = strategy.Node(i, {}, i)

    paths = []
    a = b = 0

    for j in graph:
        for i in range(len(graph[j])):
            k = graph[j].pop()

            path = strategy.Path('P' + str(a) + str(b), {j: nodeObjs[j], k: nodeObjs[k]})
            nodeObjs[j].addPath(path)
            nodeObjs[k].addPath(path)

            graph[k].remove(j)
            paths.append(path)
            b += 1

        a += 1

    return nodeObjs

def callCreateEmptyNodes():
    nodes = createEmptyNodes(Constant.ROW, Constant.COL, Constant.INTERVAL)
    writeObjToFile('emptyNodes', nodes)

def callCreateEmptyGraph():
    nodes = readFileToObj('emptyNodes')
    return createGraph(nodes)

def roundPoint(point, interval):
    if abs(point[0]%interval) > interval/2:
        p1 = round(point[0] + interval - (point[0]%interval), 2)

    else:
        p1 = round(point[0] - (point[0]%interval), 2)

    if abs(point[1]%interval) > interval/2:
        p2 = round(point[1] + interval - (point[1]%interval), 2)

    else:
        p2 = round(point[1] - (point[1]%interval), 2)

    return (p1, p2)

def findPath(graph, initPoint, finalPoint):
    return getPathToFollow(graph, roundPoint(initPoint, Constant.INTERVAL), roundPoint(finalPoint, Constant.INTERVAL))

# problem is the graph
def getPathToFollow(problem, initPoint, finalPoint):
    startT = time.time()

    prob = strategy.Problem(problem, problem[initPoint], problem[finalPoint])
    srh = strategy.Search()
    final = srh.ngraphSearch(prob, strategy.Strategy(strategy.Strategy.STRATEGY['A*']))
    
    endT = time.time()
    pathFollowed = []

    if final is False:
        print("Position unreachable")
        return

    while final is not None:
        pathFollowed.append(final.getTail().name)
        final = final.getParentPath()

    pathFollowed.reverse()
    print(pathFollowed)
    print("Search time", endT - startT)

    return pathFollowed

callCreateEmptyNodes()