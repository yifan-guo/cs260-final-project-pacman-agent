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
from game import Directions
import game

#################
# Team creation #
#################

#game creates team, game keeps track of them
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
    self.positions = {}
    self.us = None
    self.them = None
    self.legalPositions = None
    self.beliefs = {}
    self.halfway = 0

    self.plan = None     #plan to eat all the dots
    
    #called the first time a new agent is added to planner
    #post: get
  def initialize(self, gameState, red):
    # Set the teams
    if red:
      self.us = gameState.getRedTeamIndices()
      self.them = gameState.getBlueTeamIndices()
    else:
      self.us = gameState.getBlueTeamIndices()
      self.them = gameState.getRedTeamIndices()
      
    self.halfway = gameState.data.food.width/2
    
    # Start with uniform belief.
    if self.legalPositions == None:
      self.legalPositions = [p for p in gameState.getWalls().asList(False) if p[1] > 1]
      for i in self.them:
        b = util.Counter()
        for p in self.legalPositions: b[p] = 1.0
        b.normalize()
        self.beliefs[i] = b

    #TODO: call a method that will store the plan of eating all the dots
    #plan = generatePlanToEatAllDots()
    #legalPositions is all the positions pacman or ghost can be

 
    #adds the agent to the planner
  def addAgent(self, agent, index, gameState):
    self.agents[index] = agent
    self.positions[index] = gameState.getAgentPosition(index)
    if self.legalPositions == None:
      self.initialize(gameState, agent.red)
    self.recordState(index, gameState)

  def generatePlanToEatAllDots(self, gameState):
    #.aStarSearch(foodHeuristic)
    print "Do not generate plan to eat dots."

    
    #quick way to update an agent's position
  def updateAgentPosition(self, index, position):
    self.positions[index] = position
    
  # Update beliefs with agent observation.
  def recordState(self, index, gameState):
    #Update agent positions, just in case
    for agent in self.us:
      self.updateAgentPosition(agent, gameState.getAgentPosition(agent))
  
    noisyDistances = gameState.getAgentDistances()
    position = gameState.getAgentPosition(index)

    for i in self.them:
     enemyPos = gameState.getAgentPosition(i)
     
     if enemyPos == None:
       noisyDistance = noisyDistances[i]
       allPossible = util.Counter()

       if not noisyDistance == None:
         for p in self.legalPositions:
           trueDistance = util.manhattanDistance(p, position)
           prob = gameState.getDistanceProb(trueDistance, noisyDistance)
           if prob > 0:
             allPossible[p] = self.beliefs[i][p]*prob

       allPossible.normalize()
       self.beliefs[i] = allPossible
        
     else:
       allPossible = util.Counter()
       allPossible[enemyPos] = 1.0
       #allPossible.normalize() #Not necessary
       self.beliefs[i] = allPossible
    
  # Update beliefs for opponent motion. Index is the moving opponent.
  def stepState(self, index, gameState):
    allPossible = util.Counter()

    for oldP in self.legalPositions:
      oldX, oldY = oldP
      pos = [oldP, (oldX+1, oldY), (oldX-1, oldY), (oldX, oldY+1), (oldX, oldY-1)]
      legalPos = [p for p in pos if p in self.legalPositions]
      prob = 1/len(legalPos)
      for newP in legalPos:
        allPossible[newP] += self.beliefs[index][oldP]*prob

    for i in self.us:
      allPossible[self.positions[i]] = 0
    allPossible.normalize()
    self.beliefs[index] = allPossible
    
  # Gets the best gues for each opponent's position
  def getOpponentPositions(self):
    enemyPos = []
    for enemy in self.them:
      enemyPos.append(self.beliefs[enemy].argMax())
    return enemyPos
    
  # Gets the move for the defensive agent
  def getDefensiveMove(self, index, gameState):
    actions = gameState.getLegalActions(index)
    agent = self.agents[index]
  
    # Determine the closest opponent and the locations of intruders
    enemyPos = self.getOpponentPositions()
    print "enemy:", enemyPos
    intruded = False
    intruders = []
    closestPos = enemyPos[0]
    weRed = gameState.isOnRedTeam(self.us[0])
    if weRed:
      for pos in enemyPos:
        if pos == None:
          print "Red enemy lost."
        elif pos[0] < self.halfway:
          intruded = True
          intruders.append(pos)
        elif pos[0] < closestPos[0]:
          closestPos = pos
    else:
      for pos in enemyPos:
        if pos == None:
          print "Blue enemy lost."
        elif pos[0] >= self.halfway:
          intruded = True
          intruders.append(pos)
        elif pos[0] > closestPos[0]:
          closestPos = pos
          
    # If there are intruders, charge the closest one
    if intruded:
      intruderPos = intruders[0]
      intruderDist = agent.getMazeDistance(self.positions[index], intruderPos)
      for intruder in intruders:
        if agent.getMazeDistance(self.positions[index], intruder) < intruderDist:
          intruderPos = intruder
          intruderDist = agent.getMazeDistance(self.positions[index], intruderPos)
      # Choose the action that creates a successorState with a shorter maze distance
      for act in actions:
        successor = gameState.generateSuccessor(index, act)
        dist = agent.getMazeDistance(successor.getAgentPosition(index), intruderPos)
        if dist < intruderDist:
          return act
          
    # If there is no intruder, move along the midline to the closest opponent
    else:
      borderY = None
      if weRed:
        borderY = self.halfway - 1
      else:
        borderY = self.halfway
      destination = (closestPos[0], borderY)
      
      # If destination is not legal, move it
      while not destination in self.legalPositions:
        if weRed:
          destination = (closestPos[0], destination[1] - 1)
        else:
          destination = (closestPos[0], destination[1] + 1)
          
      oldDist = agent.getMazeDistance(self.positions[index], destination)
      for act in actions:
        successor = gameState.generateSuccessor(index, act)
        newDist = agent.getMazeDistance(successor.getAgentPosition(index), destination)
        if newDist < oldDist:
          return act
    
    return Directions.STOP  # Should not reach here?

  #tells if a agent is offensive or defensive
  def getOffensive(self, index):
    return False

  #TODO: return the move the offensive agent will take
  #CAUTION: this method does not generate the food-eating plan, simply returns the move to get to the next point in the plan
  #def getOffensiveMove(self, index, gameState):


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
    CaptureAgent.__init__(self, index)

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

    action = Directions.STOP
    if self.planner.getOffensive(self.index):
      action = self.planner.getOffensiveMove(self.index, gameState)   
    else:
      action = self.planner.getDefensiveMove(self.index, gameState) 
    
    # Upon choosing an action, update beliefs for possible opponent motion and agent position.
    successor = gameState.generateSuccessor(self.index, action)
    self.planner.updateAgentPosition(self.index, successor.getAgentPosition(self.index))
    nextIndex = (self.index + 1) % gameState.getNumAgents()
    self.planner.stepState(nextIndex, gameState)

    return action

