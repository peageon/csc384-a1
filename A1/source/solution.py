#   Look for #IMPLEMENT tags in this file. These tags indicate what has
#   to be implemented to complete the warehouse domain.

#   You may add only standard python imports---i.e., ones that are automatically
#   available on TEACH.CS
#   You may not remove any imports.
#   You may not import or otherwise source any of your own files

import os
import time
from search import * #for search engines
from snowman import SnowmanState, Direction, snowman_goal_state #for snowball specific classes
from test_problems import PROBLEMS #20 test problems

def heur_manhattan_distance(state):
#IMPLEMENT
  '''admissible sokoban puzzle heuristic: manhattan distance'''
  '''INPUT: a snowman state'''
  '''OUTPUT: a numeric value that serves as an estimate of the distance of the state to the goal.'''
  #We want an admissible heuristic, which is an optimistic heuristic.
  #It must never overestimate the cost to get from the current state to the goal.
  #The sum of the Manhattan distances between each snowball that has yet to be stored and the storage point is such a heuristic.
  #When calculating distances, assume there are no obstacles on the grid.
  #You should implement this heuristic function exactly, even if it is tempting to improve it.
  #Your function should return a numeric value; this is the estimate of the distance to the goal.
  
  #resulting total manhattan distance
  result = 0

  #ignore obstacles
  #snowballs is a dictionary in the form {(x_coordinate, y_coordinate): size, ...}
  for snowball in state.snowballs: 
    #calculate manhattan distance
    man_dist = abs(snowball[0] - state.destination[0]) + abs(snowball[1] - state.destination[1])

    #according to @273 if the snowball is stacked we need to calculate for stacked snowballs
    sb_size = state.snowballs[snowball]
    #{0: 'b', 1: 'm', 2: 's', 3: 'A', 4: 'B', 5: 'C', 6: 'G'}
    if (2 < sb_size and sb_size < 6): #2 stacked snowballs 
      man_dist = man_dist * 2
    if (sb_size == 6): #3 stacked snowballs
      man_dist = man_dist * 3
    
    result += man_dist
    
  return result



#HEURISTICS
def trivial_heuristic(state):
  '''trivial admissible snowball heuristic'''
  '''INPUT: a snowball state'''
  '''OUTPUT: a numeric value that serves as an estimate of the distance of the state (# of moves required to get) to the goal.'''   
  return len(state.snowballs)

def heur_alternate(state):
#IMPLEMENT
  '''a better heuristic'''
  '''INPUT: a sokoban state'''
  '''OUTPUT: a numeric value that serves as an estimate of the distance of the state to the goal.'''
  #heur_manhattan_distance has flaws.
  #Write a heuristic function that improves upon heur_manhattan_distance to estimate distance between the current state and the goal.
  #Your function should return a numeric value for the estimate of the distance to the goal.
  total = 0
  for snowball in state.snowballs:
    x = snowball[0]
    y = snowball[1]
    
    # return 0 for the snowballs that are already in destination
    if(snowball in state.destination):
      return 0
    
    else:
      # checks if a snowball is in the one of the corners and destination not in the corner
      if ((x == 0 and y == 0) or (x == 0 and y == state.height - 1)
          or (x == state.width - 1 and y == 0) or (x == state.width - 1 and y == state.height - 1)):
        if (state.destination != (x, y)):
          return float('inf')       
      
      # checks if snowball is in the beside of a wall and the destination is on that wall
      if((x == 0 or x == state.width - 1) and x != state.destination[0]):
        return float('inf')
      elif((y == 0 or y == state.height - 1) and y != state.destination[1]):
        return float('inf')
      
      # calculate manhattan distance
      distance = abs(x - state.destination[0]) + abs(y - state.destination[1])
      
      # recalculate distance in the case of stacks of snowballs
      size = state.snowballs[snowball]
      if (size == 3 or size == 4 or size == 5):
        distance = distance * 2
      elif (size == 6):
        distance = distance * 3
        
      total = total + distance
  # takes into account obstacles
  total -= len(state.obstacles)
  # manhattan distance for robot
  total += abs(state.robot[0] - state.destination[0]) + abs(state.robot[1] - state.destination[1])
  return total

def heur_zero(state):
  '''Zero Heuristic can be used to make A* search perform uniform cost search'''
  return 0

def fval_function(sN, weight):
#IMPLEMENT
  """
  Provide a custom formula for f-value computation for Anytime Weighted A star.
  Returns the fval of the state contained in the sNode.

  @param sNode sN: A search node (containing a SokobanState)
  @param float weight: Weight given by Anytime Weighted A star
  @rtype: float
  """

  #Many searches will explore nodes (or states) that are ordered by their f-value.
  #For UCS, the fvalue is the same as the gval of the state. For best-first search, the fvalue is the hval of the state.
  #You can use this function to create an alternate f-value for states; this must be a function of the state and the weight.
  #The function must return a numeric f-value.
  #The value will determine your state's position on the Frontier list during a 'custom' search.
  #You must initialize your search engine object as a 'custom' search engine if you supply a custom fval function.
  return sN.gval + weight * sN.hval

def anytime_weighted_astar(initial_state, heur_fn, weight=1., timebound = 5):
#IMPLEMENT
  '''Provides an implementation of anytime weighted a-star, as described in the HW1 handout'''
  '''INPUT: a sokoban state that represents the start state and a timebound (number of seconds)'''
  '''OUTPUT: A goal state (if a goal is found), else False'''
  '''implementation of weighted astar algorithm'''
  if (weight < 1):
    print("weight too small")
    return False
  
  temp_result = False
  wrapped_fval_function = (lambda sN : fval_function(sN,weight))
  cur_time = time.time()
  end_time = cur_time + timebound

  #initialize search engine to custom (not astar) and full cycle check
  w_search = SearchEngine('custom', 'full')
  #set up custom search including fval_function parameter
  w_search.init_search(initial_state, snowman_goal_state, heur_fn, wrapped_fval_function)
  #first search iteration
  result = w_search.search(timebound)
  if (result == False):
    return temp_result
  #define costbound for pruning
  #costbound = (result.gval, heur_fn(result), result.gval + weight * heur_fn(result))
  costbound = (result.gval, result.gval, result.gval)

  while (cur_time < end_time):
    #update current time
    cur_time = time.time()
    #timebound: remaining time
    timebound = end_time - cur_time

    if (result == False):
      return temp_result
    
    if (result.gval < costbound[0]):
      #since the new result's gval is smaller than previous result we change to result to the new result
      temp_result = result
      #costbound = (temp_result.gval, hval, temp_result.gval + weight * hval)
      costbound = (result.gval, result.gval, result.gval)
    
    result = w_search.search(timebound, costbound)
  return result



def anytime_gbfs(initial_state, heur_fn, timebound = 5):
#IMPLEMENT
  '''Provides an implementation of anytime greedy best-first search, as de
  scribed in the HW1 handout'''
  '''INPUT: a sokoban state that represents the start state and a timebound (number of seconds)'''
  '''OUTPUT: A goal state (if a goal is found), else False'''
  '''implementation of weighted astar algorithm'''
  temp_result = False
  cur_time = time.time()
  end_time = cur_time + timebound

  #initialize search engine to best first and full cycle check
  bf_search = SearchEngine('best_first', 'full')
  #set up specific search
  bf_search.init_search(initial_state, snowman_goal_state, heur_fn)
  #first search iteration | result is a statespace not NODE
  
  result = bf_search.search(timebound)
  if (result == False):
    return temp_result
  #define costbound for pruning
  #costbound = (result.gval, heur_fn(result), result.gval + heur_fn(result))
  costbound = (result.gval, result.gval, result.gval)

  while (cur_time < end_time - 0.2):
    #update current time
    cur_time = time.time()
    #timebound: remaining time
    timebound = end_time - cur_time
    if (result == False):
      return temp_result
    
    if (result.gval < costbound[0]):
      #since the new result's gval is smaller than previous result we change to result to the new result
      temp_result = result
      #costbound = (temp_result.gval, heur_fn(temp_result), temp_result.gval + heur_fn(temp_result))
      costbound = (result.gval, result.gval, result.gval)
    result = bf_search.search(timebound, costbound)
  return result

