# search.py
# ---------
# Licensing Information:  You are free to use or extend these projects for
# educational purposes provided that (1) you do not distribute or publish
# solutions, (2) you retain this notice, and (3) you provide clear
# attribution to UC Berkeley, including a link to http://ai.berkeley.edu.
# 
# Attribution Information: The Pacman AI projects were developed at UC Berkeley.
# The core projects and autograders were primarily created by John DeNero
# (denero@cs.berkeley.edu) and Dan Klein (klein@cs.berkeley.edu).
# Student side autograding was added by Brad Miller, Nick Hay, and
# Pieter Abbeel (pabbeel@cs.berkeley.edu).


"""
In search.py, you will implement generic search algorithms which are called by
Pacman agents (in searchAgents.py).
"""

import util

class Node:
    def __init__(self, coordinate, action,  parent, cost):
        self.coordinate = coordinate
        self.action = action
        self.parent = parent
        self.cost = cost

class SearchProblem:
    """
    This class outlines the structure of a search problem, but doesn't implement
    any of the methods (in object-oriented terminology: an abstract class).

    You do not need to change anything in this class, ever.
    """

    def getStartState(self):
        """
        Returns the start state for the search problem.
        """
        util.raiseNotDefined()

    def isGoalState(self, state):
        """
          state: Search state

        Returns True if and only if the state is a valid goal state.
        """
        util.raiseNotDefined()

    def getSuccessors(self, state):
        """
          state: Search state

        For a given state, this should return a list of triples, (successor,
        action, stepCost), where 'successor' is a successor to the current
        state, 'action' is the action required to get there, and 'stepCost' is
        the incremental cost of expanding to that successor.
        """
        util.raiseNotDefined()

    def getCostOfActions(self, actions):
        """
         actions: A list of actions to take

        This method returns the total cost of a particular sequence of actions.
        The sequence must be composed of legal moves.
        """
        util.raiseNotDefined()


def tinyMazeSearch(problem):
    """
    Returns a sequence of moves that solves tinyMaze.  For any other maze, the
    sequence of moves will be incorrect, so only use this for tinyMaze.
    """
    from game import Directions
    s = Directions.SOUTH
    w = Directions.WEST
    return  [s, s, w, s, w, w, s, w]

def depthFirstSearch(problem):
    """
    Search the deepest nodes in the search tree first.

    Your search algorithm needs to return a list of actions that reaches the
    goal. Make sure to implement a graph search algorithm.

    To get started, you might want to try some of these simple commands to
    understand the search problem that is being passed in:

    print "Start:", problem.getStartState()
    print "Is the start a goal?", problem.isGoalState(problem.getStartState())
    print "Start's successors:", problem.getSuccessors(problem.getStartState())
    """
    "*** YOUR CODE HERE ***"

    current = problem.getStartState()
    stack = util.Stack()
    startNode = Node(current, None, None)
    stack.push(startNode)
    visited = {'0'} #list of visited nodes
    actions = []
    visitedCoordinates = {'0'}

    while not stack.isEmpty(): 
        top = stack.list[-1] 
        if problem.isGoalState(top.coordinate):
            #print "found goal"
            while top.parent != None:
                actions.append(top.action)
                top = top.parent
            break
        else:
            if(top.coordinate in visitedCoordinates):
                #print "visited"
                stack.pop()
            else:
                #print visitedCoordinates
                visited.add(top)
                visitedCoordinates.add(top.coordinate)
                #print top.coordinate
                successors = problem.getSuccessors(top.coordinate)
                for i in successors:
                    if(successors[0] not in visitedCoordinates):
                        newNode = Node(i[0], i[1], top)
                        #print "New node coordinate: ", newNode.coordinate
                        stack.push(newNode)
        '''check for top of stack
        check if top is goal
            #true = return dic path element of current node
        check if current is visited
            #if not visited, pop and add to visited, get successor, add to stack
            #if visited, add path and action to dic

        store most recent action and parent in each node, create object of node each time new direction taken

        TA notes: state is beginning node, action will bring you to another state
            put states in stack, move from root state to one state, pop current state and move to visited
            continue moving down from current states, add children states to stack, visit children state and pop new current
            can use dictionary to store visited, or create node object from node class structure - coordinate, store action, cost, and parent
            can backtack through recursion
                def backtrack(node):
                    if node.parent is none:
                        return []
                    return backtrack(node.parent) + list of actions'''
    actions.reverse()
    #print actions
    return actions
    #util.raiseNotDefined()

def breadthFirstSearch(problem):
    #create  queue
    queue = util.Queue()
    #push starting node into it
    queue.push(problem.getStartState())
    #create another queue to keep track of every path ran by the search
    path = util.Queue()
    #create a list of reached nodes
    reached = []
    #create the final path list that will contain the correct direction list to return
    fpath = []
    #set the current node to the starting node
    currnode = queue.pop()
    #loop until the current node is the goal state
    while not problem.isGoalState(currnode):
        #check if the current node has not been reached yet
        if currnode not in reached:
            #if it hasn't, add it to the reached list
            reached.append(currnode)
            #loop through the children of the current node
            children = problem.getSuccessors(currnode)
            for child in children:
                #check if the child node has not been reached yet
                if child[0] not in reached:
                    #add the current path of this loop, taking the current final path and adding the current child node to it
                    path.push(fpath + [child[1]])
                    #if it hasn't, push the child node into the queue
                    queue.push(child[0])
        #update the final path to the new current path
        fpath = path.pop()
        #update the current node to the next node in the queue
        currnode = queue.pop() 
    #return the final path
    return fpath

def breadthFirstSearch1(problem):
    """Search the shallowest nodes in the search tree first."""
    "*** YOUR CODE HERE ***"
    current = problem.getStartState()
    queue = util.Queue()
    startNode = Node(current, None, None)
    queue.push(startNode)
    visited = {'0'} #list of visited nodes
    actions = []
    visitedCoordinates = {'0'}

    while not queue.isEmpty(): 
        first = queue.list[-1] 
        if problem.isGoalState(first.coordinate):
            #print "found goal"
            while first.parent != None:
                actions.append(first.action)
                first = first.parent
            break
        else:
            if(first.coordinate in visitedCoordinates):
                #print "visited"
                queue.pop()
            else:
                #print visitedCoordinates
                visited.add(first)
                visitedCoordinates.add(first.coordinate)
                #print first.coordinate
                successors = problem.getSuccessors(first.coordinate)
                for i in successors:
                    if(successors[0] not in visitedCoordinates):
                        newNode = Node(i[0], i[1], first)
                        #print "New node coordinate: ", newNode.coordinate
                        queue.push(newNode)
    actions.reverse()
    #print actions
    return actions
    #util.raiseNotDefined()

def uniformCostSearch(problem):
    """Search the node of least total cost first."""
    "*** YOUR CODE HERE ***"
    current = problem.getStartState()
    pQueue = util.PriorityQueue()
    startNode = Node(current, None, None, 0)
    pQueue.push(startNode, 0)
    visited = {startNode.coordinate : startNode} #list of visited nodes
    actions = []


    while not pQueue.isEmpty():
        first = pQueue.pop() 
        if problem.isGoalState(first.coordinate):
            #print "found goal"
            while first.parent != None:
                actions.append(first.action)
                first = first.parent
            break
        else:
                #print visitedCoordinates
                visited[first.coordinate] = first
                #print first.coordinate
                successors = problem.getSuccessors(first.coordinate)
                for i in successors:
                    childAction, childCoordinate, childCost = i[1], i[0], i[2]
                    if(childCoordinate not in visited or childCost < visited[childCoordinate].cost):
                        newNode = Node(i[0], i[1], first, childCost)
                        #print "New node coordinate: ", newNode.coordinate
                        visited[childCoordinate] = newNode                   
                        pQueue.push(newNode, childCost)
    actions.reverse()
    #print actions
    return actions
    #util.raiseNotDefined()
    #util.raiseNotDefined()

def nullHeuristic(state, problem=None):
    """
    A heuristic function estimates the cost from the current state to the nearest
    goal in the provided SearchProblem.  This heuristic is trivial.
    """
    return 0

def aStarSearch(problem, heuristic=nullHeuristic):
    """Search the node that has the lowest combined cost and heuristic first."""
    #create priority queue
    priorityq = util.PriorityQueue()
    #push starting node into it with a cost of 0 (starting location)
    priorityq.push(problem.getStartState(), 0)
    #create another priority queue to keep track of every path ran by the search, along with each paths cost
    path = util.PriorityQueue()
    #create a list of reached nodes
    reached = []
    #create the final path list that will contain the correct direction list to return
    fpath = []
    #set the current node to the starting node
    currnode = priorityq.pop()
    #loop until the current node is the goal state
    while not problem.isGoalState(currnode):
        #check if the current node has not been reached yet
        if currnode not in reached:
            #if it hasn't, add it to the reached list
            reached.append(currnode)
            #loop through the children of the current node
            for child in problem.getSuccessors(currnode):
                #calculate the path cost of the current final path, now including the current child node
                cost = problem.getCostOfActions(fpath + [child[1]])

                #HEURISTIC CHECK
                #check if heuristic is not NULL
                if heuristic is not nullHeuristic:
                    #calculate the heuristic using the heuristic(state,problem) function and add it to the path cost
                    cost = cost + heuristic(child[0], problem)

                #check if the child node has not been reached yet
                if child[0] not in reached:
                    #add the current path of this loop, taking the current final path and adding the current child node to it, as well as its cost
                    path.push(fpath + [child[1]], cost)
                    #if it hasn't, push the child node into the priority queue along with the calculated cost
                    priorityq.push(child[0], cost)
        #update the final path to the new current path
        fpath = path.pop()
        #update the current node to the next node in the priority queue
        currnode = priorityq.pop() 
    #return the final path
    return fpath

def aStarSearch1(problem, heuristic=nullHeuristic):
    """Search the node that has the lowest combined cost and heuristic first."""
    "*** YOUR CODE HERE ***"
    current = problem.getStartState()
    pQueue = util.PriorityQueue()
    startNode = Node(current, None, None, 0)
    pQueue.push(startNode, 0)
    visited = {startNode.coordinate : startNode} #list of visited nodes
    actions = []


    while not pQueue.isEmpty():
        first = pQueue.pop() 
        if problem.isGoalState(first.coordinate):
            #print "found goal"
            while first.parent != None:
                actions.append(first.action)
                first = first.parent
            break
        else:
                #print visitedCoordinates
                visited[first.coordinate] = first
                #print first.coordinate
                successors = problem.getSuccessors(first.coordinate)
                for i in successors:
                    childAction, childCoordinate, childCost = i[1], i[0], i[2]
                    newCost = childCost + util.manhattanDistance(childCoordinate, problem.goal)
                    if(childCoordinate not in visited or childCost < visited[childCoordinate].cost):
                        newNode = Node(i[0], i[1], first, childCost)
                        #print "New node coordinate: ", newNode.coordinate
                        visited[childCoordinate] = newNode                   
                        pQueue.push(newNode, childCost)
    actions.reverse()
    #print actions
    return actions
    #util.raiseNotDefined()


# Abbreviations
bfs = breadthFirstSearch
dfs = depthFirstSearch
astar = aStarSearch
ucs = uniformCostSearch
