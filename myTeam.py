# myTeam.py
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


from captureAgents import CaptureAgent
import random, time, util
import game

from game import Directions
from game import Agent
from game import Actions
import util
import time

#################
# Team creation #
#################

class FoodSearchProblem:
    """
    A search problem associated with finding the a path that collects all of the
    food (dots) in a Pacman game.

    A search state in this problem is a tuple ( pacmanPosition, foodGrid ) where
      pacmanPosition: a tuple (x,y) of integers specifying Pacman's position
      foodGrid:       a Grid (see game.py) of either True or False, specifying remaining food
    """
    def __init__(self, startingGameState, red):
      if red:
        self.food = startingGameState.getBlueFood()
      else:
        self.food = startingGameState.getRedFood()

      self.start = (startingGameState.getAgentPosition(), self.food)
      self.walls = startingGameState.getWalls()
      self.startingGameState = startingGameState
      self._expanded = 0 # DO NOT CHANGE
      self.heuristicInfo = {} # A dictionary for the heuristic to store information

    def getStartState(self):
        return self.start

    def isGoalState(self, state):
        return state[1].count() == 0

    def getSuccessors(self, state):
        "Returns successor states, the actions they require, and a cost of 1."
        successors = []
        self._expanded += 1 # DO NOT CHANGE
        for direction in [Directions.NORTH, Directions.SOUTH, Directions.EAST, Directions.WEST]:
            x,y = state[0]
            dx, dy = Actions.directionToVector(direction)
            nextx, nexty = int(x + dx), int(y + dy)
            if not self.walls[nextx][nexty]:

                foodGrid = state[1]
                tmp = foodGrid[nextx][nexty]

                nextFood = state[1].copy()
                nextFood[nextx][nexty] = False
                successors.append( ( ((nextx, nexty), nextFood), direction, 1, tmp) )
        return successors

#Our plan: one offensive agent that precomputes the shortest path to eat all the dots, and follows that route either to completion or death.
#          a defensive agent that always goes to the border and lines up vertically with the enemy agent closest (smallest manhattan distance) to the border
#          if enemy eats capsule, both agents follow offensive route
#          if enemy crosses over into our territory, defender agent tracks the offending agent immediately.

#game creates team, game keeps track of the agents
#when an agent's turn is up, the game calls agent.chooseAction() in order to get the action the agent wants to take

#this function creates the planner and the agents
def createTeam(firstIndex, secondIndex, isRed,
               first = 'StarAgent', second = 'StarAgent'):
  """
  This function should return a list of two agents that will form the
  team, initialized using firstIndex and secondIndex as their agent
  index numbers.  isRed is True if the red team is being created, and
  will be False if the blue team is being created.

  As a potentially helpful development aid, this function can take
  additional string-valued keyword arguments ("first" and "second" are
  such arguments in the case of this function), which will come from
  the --redOpts and --blueOpts command-line arguments to capture.py.
  For the nightly contest, however, your team will be created without
  any extra arguments, so you should make sure that the default
  behavior is what you want for the nightly contest.
  """
  
  planner = StarPlanner()

  # The following line is an example only; feel free to change it.
  return [eval(first)(firstIndex, planner), eval(second)(secondIndex, planner)]

###############
# Agent Tools #
###############

#class 

class StarPlanner():

  def __init__(self):
    #Need probability map, this team coord, other team coord
    self.agents = {}
    self.positions = []
    self.us = None
    self.them = None
    self.legalPositions = None
    self.beliefs = None

    # self.searchFunction = lambda prob: aStarSearch(prob, foodHeuristic)
    # self.searchType = FoodSearchProblem
    self.cdts = None     #list of dots to eat ordered s.t. path through them is minimized
    
    #called the first time a new agent is added to planner
  def initialize(self, gameState, red):
    # Set the teams
    if red:
      self.us = gameState.getRedTeamIndices()
      self.them = gamestate.getBlueTeamIndices()
    else:
      self.us = gameState.getBlueTeamIndices()
      self.them = gamestate.getRedTeamIndices()
    
    # Start with uniform belief.
    if self.legalPositions == None:
      self.legalPositions = [p for p in gameState.getWalls().asList(False) if p[1] > 1]
      for i in self.them:
        b = util.Counter()
        for p in self.legalPositions: b[p] = 1.0
        b.normalize()
        self.beliefs[i] = b

    #define the problem given the start state    
    problem = FoodSearchProblem(gameState, red)
    self.cdts = aStarSearch(problem, foodHeuristic)

    #legalPositions is all the positions pacman or ghost can be

 
    #adds the agent to the planner
  def addAgent(agent, index, gameState):
    self.agents[index] = agent
    self.positions[index] = gameState.getAgentPosition(self.index)
    if self.legalPositions == None:
      self.initialize(gameState, agent.red)


    #quick way to update an agent's position
  def updateAgentPosition(index, position):
    self.positions[index] = position
    
    #For defensive play
  def recordState(self, index, gameState):
    #
    noisyDistances = gamestate.getAgentDistances()
    position = gameState.getAgentPosition(index)

   for i in self.them:
   
     enemyPos = gamestate.getAgentPosition(i)
     
     if enemyPos == None:
        noisyDistance = noisyDistance[i]
        allPossible = util.Counter()

        if not noisyDistance == None:
            for p in self.legalPositions:
                trueDistance = util.manhattanDistance(p, position)
                prob = gamestate.getDistanceProb(trueDistance, noisyDistance)
                if prob > 0:
                    allPossible[p] = self.beliefs[i][p]*prob

        allPossible.normalize()
        self.beliefs[i] = allPossible
        
      else:
        allPossible = util.Counter()
        allPossible[enemyPos] = 1.0
        allPossible.normalize()
        self.beliefs[i] = allPossible

  #tells if a agent is offensive or defensive
  def getOffensive(self, index, gameState):
    if index == self.us[0]:
      return True
    else:
      return False

    # if self.positions[index] == gameState.getInitialAgentPosition(index) && : 


  #TODO: return the move the offensive agent will take
  #CAUTION: this method does not generate the food-eating plan, simply returns the move to get to the next point in the plan
  def getOffensiveMove(self, index, gameState):
    agent = self.agents[index]
    x = agent.getMazeDistance(self.positions[index], self.cdts[0])

    for action in gameState.getLegalActions(index):
      successorState = gameState.generateSuccessor(index, action)
      newpos = successorState.getAgentPosition(index)
      tmp = agent.getMazeDistance(newpos, self.cdts[0])

      if tmp == 0:
        del self.cdts[0]
        return action

      if tmp < x:
        return action

    return Directions.STOP      #should never reach this point


##########
# Agents #
##########

class StarAgent(CaptureAgent):
  """
  A Dummy agent to serve as an example of the necessary agent structure.
  You should look at baselineTeam.py for more details about how to
  create an agent as this is the bare minimum.
  """
  
  def __init__(self, index, planner):
    self.planner = planner
    CaptureAgent.__init__(index)

  def registerInitialState(self, gameState):
    """
    This method handles the initial setup of the
    agent to populate useful fields (such as what team
    we're on).

    A distanceCalculator instance caches the maze distances
    between each pair of positions, so your agents can use:
    self.distancer.getDistance(p1, p2)

    IMPORTANT: This method may run for at most 15 seconds.
    """

    '''
    Make sure you do not delete the following line. If you would like to
    use Manhattan distances instead of maze distances in order to save
    on initialization time, please take a look at
    CaptureAgent.registerInitialState in captureAgents.py.
    '''
    CaptureAgent.registerInitialState(self, gameState)

    '''
    Your initialization code goes here, if you need any.
    '''
    
    self.planner.addAgent(self, self.index, gameState)


    #game calls this function to determine
  def chooseAction(self, gameState):
  
    self.planner.recordState(self.index, gameState)

    if self.planner.getOffensive(self.index):
      #return self.planner.getOffensiveMove(self.index, gameState)    
  
    """
    Picks among actions randomly.
    """
    actions = gameState.getLegalActions(self.index)

    '''
    You should change this in your own agent.
    '''

    return random.choice(actions)


#number of food remaining
def foodHeuristic(state):
  return state[1].count()


def aStarSearch(problem, heuristic=nullHeuristic):
    """Search the node that has the lowest combined cost and heuristic first."""
    "*** YOUR CODE HERE ***"
    visited = set() ### Coordinates that have been visited
    toVisit = util.PriorityQueue() ### Stack of coordinates to visit
    start = problem.getStartState() ### Get start state of problem
    visited.add(start[0]) ### Add start cdts to visited since we will expand it
    for successor in problem.getSuccessors(start): ### Generate the successors of the start state (possible legal actions)
        # direction = directions_dict[successor[1]] ### Get the direction related to moving toward that coordinate
        direction = successor[1]
        cost = successor[2] 
        isFoodCdts = successor[3]

        if isFoodCdts:          #food disappeared
          tmp = successor[0]    #get the (cdts, food) pair
          foodCdts = list(tmp[0])
        else:
          foodCdts = []

        coordinate_and_direction_and_cost_and_cdts = [successor[0], [direction], cost, foodCdts] ### Append the coordinate, list of directions, and f(n), and list of food cdts
        # print coordinate_and_direction
        toVisit.push(coordinate_and_direction_and_cost_and_cdts, cost + heuristic(successor[0])) ### Push coordinate and direction to the top of the stack

    while (not toVisit.isEmpty()):
        next_node = toVisit.pop()       #automatically pop the node with the cheapest cost
        next_node_coordinates_and_grid = next_node[0]
        next_node_directions = next_node[1]
        cost = next_node[2]         
        next_node_foodCdts = next_node[3]     #NEW
        if next_node_coordinates_and_grid[0] in visited:
            continue
        elif problem.isGoalState(next_node_coordinates_and_grid):
            return next_node_foodCdts
        else:
            visited.add(next_node_coordinates_and_grid[0])
            for successor in problem.getSuccessors(next_node_coordinates_and_grid):
                # if successor[0] not in visited and successor[0] not in toVisitCoordinates.list:
                new_direction = successor[1]
                new_cost = cost + successor[2]
                isFoodCdts = successor[3]

                new_foodCdts = []
                for f in next_node_foodCdts:
                  new_foodCdts.append(f)

                if isFoodCdts:          #food disappeared
                  tmp = successor[0]    #get the (cdts, food) pair
                  new_foodCdts.append(tmp[0])

                direction = []
                for d in next_node_directions:    #append the original actions
                    direction.append(d)
                direction.append(new_direction)   #append the new actions

                coordinate_and_direction_and_cost_and_cdts = [successor[0], direction, new_cost, new_foodCdts]
                toVisit.push(coordinate_and_direction_and_cost_and_cdts, new_cost + heuristic(successor[0]))

