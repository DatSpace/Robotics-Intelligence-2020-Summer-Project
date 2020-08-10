import random
import math

MIN_NUM_VERT = 20  # Minimum number of vertex in the graph
MAX_NUM_VERT = 1500  # Maximum number of vertex in the graph
STEP_DISTANCE = 20  # Maximum distance between two vertex


def rapidlyExploringRandomTree(img, start, goal, seed=None):
    hundreds = 100
    random.seed(seed)
    points = []
    graph = []
    points.append(start)
    graph.append((start, []))
    print('Generating and conecting random points')
    occupied = True
    phaseTwo = False

    # Phase two values (points 5 step distances around the goal point)
    minX = max(goal[0] - 5 * STEP_DISTANCE, 0)
    maxX = min(goal[0] + 5 * STEP_DISTANCE, len(img[0]) - 1)
    minY = max(goal[1] - 5 * STEP_DISTANCE, 0)
    maxY = min(goal[1] + 5 * STEP_DISTANCE, len(img) - 1)

    i = 0
    while (goal not in points) and (len(points) < MAX_NUM_VERT):
        if (i % 100) == 0:
            print(i, 'points randomly generated...')

        if (len(points) % hundreds) == 0:
            print(len(points), 'vertices generated...')
            hundreds = hundreds + 100

        while(occupied):
            if phaseTwo and (random.random() > 0.8):
                point = [random.randint(minX, maxX),
                         random.randint(minY, maxY)]
            else:
                point = [random.randint(0, len(img[0]) - 1),
                         random.randint(0, len(img) - 1)]

            if(img[point[1]][point[0]] == 0):
                occupied = False

        occupied = True

        nearest = findNearestPoint(points, point)
        newPoints = connectPoints(point, nearest, img)
        addToGraph(graph, newPoints, point)
        newPoints.pop(0)  # The first element is already in the points list
        points.extend(newPoints)
        i = i + 1

        if len(points) >= MIN_NUM_VERT:
            if not phaseTwo:
                print('Minimum vertices generated...')
            phaseTwo = True

        if phaseTwo:
            nearest = findNearestPoint(points, goal)
            newPoints = connectPoints(goal, nearest, img)
            addToGraph(graph, newPoints, goal)
            newPoints.pop(0)
            points.extend(newPoints)

    if goal in points:
        #print('Goal found, total vertices in graph:', len(points))
        #print('Total random points generated:', i)
        path = searchPath(graph, start, [start])

        #print('Showing resulting map')
        #print('Final path:', path)
        #print('The final path is made from:', len(path), 'connected points')
    else:
        path = None
        print('Reached maximum number of vertices and goal was not found...')
        print('Total vertices in graph:', len(points))
        print('Total random points generated:', i)

    return path


def searchPath(graph, point, path):
    for i in graph:
        if point == i[0]:
            p = i

    if p[0] == graph[-1][0]:
        return path

    for link in p[1]:
        path.append(link)
        finalPath = searchPath(graph, link, path)

        if finalPath != None:
            return finalPath
        else:
            path.pop()


def addToGraph(graph, newPoints, point):
    if len(newPoints) > 1:  # If there is anything to add to the graph
        for p in range(len(newPoints) - 1):
            nearest = [nearest for nearest in graph if (
                nearest[0] == [newPoints[p][0], newPoints[p][1]])]
            nearest[0][1].append(newPoints[p + 1])
            graph.append((newPoints[p + 1], []))


def connectPoints(a, b, img):
    newPoints = []
    newPoints.append([b[0], b[1]])
    step = [(a[0] - b[0]) / float(STEP_DISTANCE),
            (a[1] - b[1]) / float(STEP_DISTANCE)]

    # Set small steps to check for walls
    pointsNeeded = int(math.floor(max(math.fabs(step[0]), math.fabs(step[1]))))

    if math.fabs(step[0]) > math.fabs(step[1]):
        if step[0] >= 0:
            step = [1, step[1] / math.fabs(step[0])]
        else:
            step = [-1, step[1] / math.fabs(step[0])]

    else:
        if step[1] >= 0:
            step = [step[0] / math.fabs(step[1]), 1]
        else:
            step = [step[0]/math.fabs(step[1]), -1]

    blocked = False
    for i in range(pointsNeeded+1):  # Creates points between graph and solitary point
        for j in range(STEP_DISTANCE):  # Check if there are walls between points
            coordX = round(newPoints[i][0] + step[0] * j)
            coordY = round(newPoints[i][1] + step[1] * j)

            if coordX == a[0] and coordY == a[1]:
                break
            if coordY >= len(img) or coordX >= len(img[0]):
                break
            if img[int(coordY)][int(coordX)] > 0:
                blocked = True
            if blocked:
                break

        if blocked:
            break
        if not (coordX == a[0] and coordY == a[1]):
            newPoints.append([newPoints[i][0]+(step[0]*STEP_DISTANCE),
                              newPoints[i][1]+(step[1]*STEP_DISTANCE)])

    if not blocked:
        newPoints.append([a[0], a[1]])
    return newPoints


def findNearestPoint(points, point):
    min_distance = 999999999  # Highest
    best_point = (0, 0)
    for p in points:
        if p == point:
            continue
        dist = math.sqrt((p[0] - point[0]) ** 2 + (p[1] - point[1]) ** 2)
        if dist < min_distance:
            min_distance = dist
            best_point = (p[0], p[1])
    return best_point
