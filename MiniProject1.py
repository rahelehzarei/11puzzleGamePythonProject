import os.path
import copy
import heapq


class Node:

    def __init__(self, matrix, name, action, cost, depth):
        self.matrix = matrix
        self.name = name
        self.action = action
        self.cost = cost
        self.depth = depth

    # list of actions


UP = 1
UP_RIGHT = 2
RIGHT = 3
DOWN_RIGHT = 4
DOWN = 5
DOWN_LEFT = 6
LEFT = 7
UP_LEFT = 8

max_depth = 10
goalState = Node([[1, 2, 3, 4], [5, 6, 7, 8], [9, 10, 11, 0]], None, None, None, None)  # Make a node from goal state
AssignedChar = [['a', 'b', 'c', 'd'], ['e', 'f', 'g', 'h'], ['i', 'j', 'k', 'l']]
initStateTxt = input("Enter 3X4 puzzle vales: ")
initStateMat = [int(x) for x in initStateTxt.split(' ')]
initStateMat_2d = [initStateMat[i:i + 4] for i in range(0, len(initStateMat), 4)]
initialStateWithChar = dict(zip(initStateMat, ['a', 'b', 'c', 'd', 'e', 'f', 'g', 'h', 'i', 'j', 'k', 'l']))
initialState = Node(initStateMat_2d, '0', None, 0, 0)  # transform init to node


def dfs(initialState):
    depth = 0
    openNodes = []
    visitedNodes = []
    visitedNodesMat = []
    solutionPath = []
    parents = dict()

    openNodes.append(initialState)

    while openNodes:
        currentNode = openNodes.pop()
        visitedNodes.append(currentNode)
        visitedNodesMat.append(currentNode.matrix)
        # expandedLevel = currentNode.depth

        if currentNode.matrix == goalState.matrix:
            while (currentNode.name != '0'):
                solutionPath.append(currentNode)
                currentNode = parents[currentNode]
            solutionPath.append(currentNode)
            solutionPath.reverse()
            printSolution(solutionPath, 'DFS')
            return

        if currentNode.depth <= max_depth:
            children = reversed(findchildren(currentNode))
            for child in children:
                if child.matrix not in visitedNodesMat:
                    openNodes.append(child)
                    parents[child] = currentNode
    printSolution([], 'DFS')


def bfs_h1(intitialState):
    openNodes = []
    visitedNodes = []
    visitedNodesMat = []
    solutionPath = []
    parents = dict()
    p = 0  # determine the priority when two node have same heuristic value (same f(n)
    hVal = heuristic_1(initialState)  # heuristic value of initial state
    entry = (hVal, p, intitialState)

    heapq.heappush(openNodes, entry)

    while openNodes:
        currentNode = heapq.heappop(openNodes)
        visitedNodes.append(currentNode[2])
        visitedNodesMat.append(currentNode[2].matrix)

        if currentNode[2].matrix == goalState.matrix:
            while currentNode[2].name != '0':  # heuristic of goal is 0. heuristic_h1(goal)=0
                solutionPath.append(currentNode[2])
                currentNode = parents[currentNode[2]]
            solutionPath.append(currentNode[2])
            solutionPath.reverse()
            printSolution(solutionPath, 'BFS-h1')
            return
        else:
            children = findchildren(currentNode[2])
            for child in children:
                if child.matrix not in visitedNodesMat:
                    hVal = heuristic_1(child)
                    p += 1
                    entry = (hVal, p, child)
                    heapq.heappush(openNodes, entry)
                    parents[child] = currentNode
    printSolution([], 'BFS-h1')


def bfs_h2(intialState):
    openNodes = []
    visitedNodes = []
    visitedNodesMat = []
    solutionPath = []
    parents = dict()
    p = 0  # determine the priority when two node have same heuristic value (same f(n)
    hVal = heuristic_2(initialState)  # heuristic value of initial state
    entry = (hVal, p, initialState)

    heapq.heappush(openNodes, entry)

    while openNodes:
        currentNode = heapq.heappop(openNodes)
        visitedNodes.append(currentNode[2])
        visitedNodesMat.append(currentNode[2].matrix)

        if currentNode[2].matrix == goalState.matrix:
            while currentNode[2].name != '0':  # heuristic of goal is 0. heuristic_h1(goal)=0
                solutionPath.append(currentNode[2])
                currentNode = parents[currentNode[2]]
            solutionPath.append(currentNode[2])
            solutionPath.reverse()
            printSolution(solutionPath, 'BFS-h2')
            return
        else:
            children = findchildren(currentNode[2])
            for child in children:
                if child.matrix not in visitedNodesMat:
                    hVal = heuristic_2(child)
                    p += 1
                    entry = (hVal, p, child)
                    heapq.heappush(openNodes, entry)
                    parents[child] = currentNode
    printSolution([], 'BFS-h2')


def astar_h1(intialState):
    openNodes = []
    visitedNodes = []
    visitedNodesMat = []
    solutionPath = []
    parents = dict()
    p = 0  # determin the priority when two node have same heuristic value (same f(n)
    hVal = heuristic_1(initialState)  # heuristic value of intial state
    entry = (hVal, p, intialState)

    heapq.heappush(openNodes, entry)

    while openNodes:
        currentNode = heapq.heappop(openNodes)
        visitedNodes.append(currentNode[2])
        visitedNodesMat.append(currentNode[2].matrix)

        if currentNode[2].matrix == goalState.matrix:
            while currentNode[2].name != '0':  # heuristic of goal is 0. heuristic_h1(goal)=0
                solutionPath.append(currentNode[2])
                currentNode = parents[currentNode[2]]
            solutionPath.append(currentNode[2])
            solutionPath.reverse()
            printSolution(solutionPath, 'As-h1')
            return
        else:
            children = findchildren(currentNode[2])
            for child in children:
                if child.matrix not in visitedNodesMat:
                    hVal = heuristic_1(child) + child.cost
                    p += 1
                    entry = (hVal, p, child)
                    heapq.heappush(openNodes, entry)
                    parents[child] = currentNode
    printSolution([], 'As-h1')


def astar_h2(intialState):
    openNodes = []
    visitedNodes = []
    visitedNodesMat = []
    solutionPath = []
    parents = dict()
    p = 0  # determine the priority when two node have same heuristic value (same f(n)
    hVal = heuristic_2(initialState)  # heuristic value of initial state
    entry = (hVal, p, initialState)

    heapq.heappush(openNodes, entry)

    while openNodes:
        currentNode = heapq.heappop(openNodes)
        visitedNodes.append(currentNode[2])
        visitedNodesMat.append(currentNode[2].matrix)

        if currentNode[2].matrix == goalState.matrix:
            while currentNode[2].name != '0':  # heuristic of goal is 0. heuristic_h1(goal)=0
                solutionPath.append(currentNode[2])
                currentNode = parents[currentNode[2]]
            solutionPath.append(currentNode[2])
            solutionPath.reverse()
            printSolution(solutionPath, 'As-h2')
            return
        else:
            children = findchildren(currentNode[2])
            for child in children:
                if child.matrix not in visitedNodesMat:
                    hVal = heuristic_2(child) + child.cost
                    p += 1
                    entry = (hVal, p, child)
                    heapq.heappush(openNodes, entry)
                    parents[child] = currentNode
    printSolution([], 'As-h2')


def heuristic_1(state):
    h = 0
    for i in range(len(state.matrix)):
        for j in range(len((state.matrix[i]))):
            if state.matrix[i][j] != goalState.matrix[i][j]:
                h += state.matrix[i][j]

    return h


def heuristic_2(state):
    h = 0
    for i in range(len(state.matrix)):
        for j in range(len(state.matrix[0])):
            if state.matrix[i][j] != goalState.matrix[i][j]:
                k = i
                m = j
                index = index_2d(goalState.matrix, state.matrix[i][j])
                while k != index[0] and m != index[1]:
                    if index[0] < k and index[1] < m:
                        h += 1
                        k -= 1
                        m -= 1
                    elif index[0] > k and index[1] > m:
                        h += 1
                        k += 1
                        m += 1
                    elif index[0] > k and index[1] < m:
                        h += 1
                        k += 1
                        m -= 1
                    elif index[0] < k and index[1] > m:
                        h += 1
                        k -= 1
                        m += 1

                if state.matrix[i][i] == goalState.matrix[k][m]:
                    h
                else:
                    h = h + 1
    return h


def findchildren(node):
    global UP, UP_RIGHT, RIGHT, DOWN_RIGHT, DOWN, DOWN_LEFT, LEFT, UP_LEFT
    # global depth
    # depth += 1
    children = list()
    index = index_2d(node.matrix, 0)
    # i = index[0], j = index[1]

    if index[0] - 1 >= 0:  # check if blank tile is in borders but not corners
        newNode1 = copy.deepcopy(node.matrix)
        newNode1[index[0]][index[1]] = newNode1[index[0] - 1][index[1]]
        newNode1[index[0] - 1][index[1]] = 0
        children.append(
            Node(newNode1, initialStateWithChar[newNode1[index[0]][index[1]]], UP, node.cost + 1, node.depth + 1))

    if index[0] - 1 >= 0 and index[1] + 1 <= 3:
        newNode2 = copy.deepcopy(node.matrix)
        newNode2[index[0]][index[1]] = newNode2[index[0] - 1][index[1] + 1]
        newNode2[index[0] - 1][index[1] + 1] = 0
        children.append(Node(newNode2, initialStateWithChar[newNode2[index[0]][index[1]]], UP_RIGHT, node.cost + 1,
                             node.depth + 1))

    if index[1] + 1 <= 3:
        newNode3 = copy.deepcopy(node.matrix)
        newNode3[index[0]][index[1]] = newNode3[index[0]][index[1] + 1]
        newNode3[index[0]][index[1] + 1] = 0
        children.append(Node(newNode3, initialStateWithChar[newNode3[index[0]][index[1]]], RIGHT, node.cost + 1,
                             node.depth + 1))
    if index[0] + 1 <= 2 and index[1] + 1 <= 3:
        newNode4 = copy.deepcopy(node.matrix)
        newNode4[index[0]][index[1]] = newNode4[index[0] + 1][index[1] + 1]
        newNode4[index[0] + 1][index[1] + 1] = 0
        children.append(Node(newNode4, initialStateWithChar[newNode4[index[0]][index[1]]], DOWN_RIGHT, node.cost + 1,
                             node.depth + 1))

    if index[0] + 1 <= 2:
        newNode5 = copy.deepcopy(node.matrix)
        newNode5[index[0]][index[1]] = newNode5[index[0] + 1][index[1]]
        newNode5[index[0] + 1][index[1]] = 0
        children.append(
            Node(newNode5, initialStateWithChar[newNode5[index[0]][index[1]]], DOWN, node.cost + 1, node.depth + 1))

    if index[0] + 1 <= 2 and index[1] - 1 >= 0:
        newNode6 = copy.deepcopy(node.matrix)
        newNode6[index[0]][index[1]] = newNode6[index[0] + 1][index[1] - 1]
        newNode6[index[0] + 1][index[1] - 1] = 0
        children.append(Node(newNode6, initialStateWithChar[newNode6[index[0]][index[1]]], DOWN_LEFT, node.cost + 1,
                             node.depth + 1))

    if index[1] - 1 >= 0:
        newNode7 = copy.deepcopy(node.matrix)
        newNode7[index[0]][index[1]] = newNode7[index[0]][index[1] - 1]
        newNode7[index[0]][index[1] - 1] = 0
        children.append((Node(newNode7, initialStateWithChar[newNode7[index[0]][index[1]]], LEFT, node.cost + 1,
                              node.depth + 1)))

    if index[0] - 1 >= 0 and index[1] - 1 >= 0:
        newNode8 = copy.deepcopy(node.matrix)
        newNode8[index[0]][index[1]] = newNode8[index[0] - 1][index[1] - 1]
        newNode8[index[0] - 1][index[1] - 1] = 0
        children.append((Node(newNode8, initialStateWithChar[newNode8[index[0]][index[1]]], UP_LEFT, node.cost + 1,
                              node.depth + 1)))

    return children


def index_2d(data, search):
    for i, e in enumerate(data):
        try:
            return i, e.index(search)
        except ValueError:
            pass
    raise ValueError("{} is not in list".format(repr(search)))


def printSolution(list, algorithm):
    completeName = os.path.join("puzzle" + algorithm + ".txt")
    if algorithm == 'DFS':
        f = open(completeName, 'w+')
        f.write("11-puzzle game's solution by Dfs search algorithm. \n")
        if list:
            for node in list:
                result = str(node.name) + " - " + " ".join(
                    map(str, [item for sublist in node.matrix for item in sublist])) + "\n"
                f.write(result)
            f.close()
        else:
            f.write("Solution not found.")
            f.close()
        print("File puzzle" + algorithm + ".txt has been created.")

    elif algorithm == 'BFS-h1':
        f = open(completeName, 'w+')
        f.write("11-puzzle game's solution by Bfs-h1 search algorithm. \n")
        if list:
            for node in list:
                result = str(node.name) + " - " + " ".join(
                    map(str, [item for sublist in node.matrix for item in sublist])) + "\n"
                f.write(result)
            f.close()
        else:
            f.write("Solution not found.")
            f.close()
        print("File puzzle" + algorithm + ".txt has been created.")

    elif algorithm == 'BFS-h2':
        f = open(completeName, 'w+')
        f.write("11-puzzle game's solution by Bfs-h2 search algorithm. \n")
        if list:
            for node in list:
                result = str(node.name) + " - " + " ".join(
                    map(str, [item for sublist in node.matrix for item in sublist])) + "\n"
                f.write(result)
            f.close()
        else:
            f.write("Solution not found.")
            f.close()
        print("File puzzle" + algorithm + ".txt has been created.")

    elif algorithm == 'As-h1':
        f = open(completeName, 'w+')
        f.write("11-puzzle game's solution by As-h1 search algorithm. \n")
        if list:
            for node in list:
                result = str(node.name) + " - " + " ".join(
                    map(str, [item for sublist in node.matrix for item in sublist])) + "\n"
                f.write(result)
            f.close()
        else:
            f.write("Solution not found.")
            f.close()
        print("File puzzle" + algorithm + ".txt has been created ")

    elif algorithm == 'As-h2':
        f = open(completeName, 'w+')
        f.write("11-puzzle game's solution by As-h2 search algorithm. \n")
        if list:
            for node in list:
                result = str(node.name) + " - " + " ".join(
                    map(str, [item for sublist in node.matrix for item in sublist])) + "\n"
                f.write(result)
            f.close()
        else:
            f.write("Solution not found.")
            f.close()
        print("File puzzle" + algorithm + ".txt has been created.")


def main():
    dfs(initialState)
    bfs_h1(initialState)
    bfs_h2(initialState)
    astar_h1(initialState)
    astar_h2(initialState)


if __name__ == '__main__':
    main()
