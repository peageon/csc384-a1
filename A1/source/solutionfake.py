#Look for #IMPLEMENT tags in this file. These tags indicate what has
#to be implemented to complete the Snowman Puzzle domain.

#   You may add only standard python imports---i.e., ones that are automatically
#   available on TEACH.CS
#   You may not remove any imports.
#   You may not import or otherwise source any of your own files

# import os for time functions
#import os
import time
from search import * #for search engines
from snowman import SnowmanState, Direction, snowman_goal_state #for snowball specific classes and problems
from test_problems import PROBLEMS #20 test problems

#snowball HEURISTICS
def heur_simple(state):
  '''trivial admissible snowball heuristic'''
  '''INPUT: a snowball state'''
  '''OUTPUT: a numeric value that serves as an estimate of the distance of the state (# of moves required to get) to the goal.'''   
  return len(state.snowballs)

def heur_zero(state):
  return 0

def heur_manhattan_distance(state):
#IMPLEMENT
    '''admissible snowball puzzle heuristic: manhattan distance'''
    '''INPUT: a snowball state'''
    '''OUTPUT: a numeric value that serves as an estimate of the distance of the state to the goal.'''      
    #We want an admissible heuristic, which is an optimistic heuristic. 
    #It must always underestimate the cost to get from the current state to the goal.
    #The sum of the Manhattan distances between the snowballs and the destination for the Snowman is such a heuristic.  
    #When calculating distances, assume there are no obstacles on the grid.
    #You should implement this heuristic function exactly, even if it is tempting to improve it.
    #Your function should return a numeric value; this is the estimate of the distance to the goal.
    
    total = 0
    for snowball in state.snowballs:
      # calculate manhattan distance between snowball and destination
      distance = abs(snowball[0] - state.destination[0]) + abs(snowball[1] - state.destination[1])
      
      # recalculate distance in the case of stacks of snowballs
      size = state.snowballs[snowball]
      if (size == 3 or size == 4 or size == 5):
        distance = distance * 2
      elif (size == 6):
        distance = distance * 3 
          
      total = total + distance
    return total

def heur_alternate(state): 
#IMPLEMENT
    '''a better heuristic'''
    '''INPUT: a snowball state'''
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

def fval_function(sN, weight):
#IMPLEMENT
    """
    Provide a custom formula for f-value computation for Anytime Weighted A star.
    Returns the fval of the state contained in the sNode.
    @param sNode sN: A search node (containing a SnowballState)
    @param float weight: Weight given by Anytime Weighted A star
    @rtype: float
    """
  
    #Many searches will explore nodes (or states) that are ordered by their f-value.
    #For UCS, the fvalue is the same as the gval of the state. For best-first search, the fvalue is the hval of the state.
    #You can use this function to create an alternate f-value for states; this must be a function of the state and the weight.
    #The function must return a numeric f-value.
    #The value will determine your state's position on the Frontier list during a 'custom' search.
    #You must initialize your search engine object as a 'custom' search engine if you supply a custom fval function.
    return sN.gval + (weight * sN.hval)

def anytime_gbfs(initial_state, heur_fn, timebound = 5):
#IMPLEMENT
    '''Provides an implementation of anytime greedy best-first search, as described in the HW1 handout'''
    '''INPUT: a snowball state that represents the start state and a timebound (number of seconds)'''
    '''OUTPUT: A goal state (if a goal is found), else False''' 
    
    result = False
    
    # records time
    starttime = time.time()
    endtime = starttime + timebound
    
    # search engine is initialized
    se = SearchEngine('best_first', 'full')
    se.init_search(initial_state, snowman_goal_state, heur_fn)
    
    cost_bound = (float("inf"), float("inf"), float("inf"))
    search_result = se.search(timebound)
    
    # searches until the timebound exceeded
    while starttime < endtime:
      if search_result == False:
        return result
      
      # records remaining time
      timebound = timebound - (time.time() - starttime)
      starttime = time.time()
      
      # prune based on g_vlaue of node
      if search_result.gval <= cost_bound[0]:
        cost_bound = (search_result.gval, search_result.gval, search_result.gval)
        result = search_result
      print(time.time())
      print(endtime)
      search_result = se.search(timebound, cost_bound)
    
    return result

def anytime_weighted_astar(initial_state, heur_fn, weight=1., timebound = 5):
#IMPLEMENT
    '''Provides an implementation of anytime weighted a-star, as described in the HW1 handout'''
    '''INPUT: a snowball state that represents the start state and a timebound (number of seconds)'''
    '''OUTPUT: A goal state (if a goal is found), else False''' 
    
    result = False
    
    # records time
    starttime = time.time()
    endtime = starttime + timebound
    
    # search engine is initialized with a custom search strategy
    wrapped_fval_function = (lambda sN: fval_function(sN, weight))
    se = SearchEngine('custom', 'full')
    se.init_search(initial_state, snowman_goal_state, heur_fn, wrapped_fval_function)
    
    cost_bound = (float("inf"), float("inf"), float("inf"))
    search_result = se.search(timebound)
    
    # searches until the timebound exceeded
    while starttime < endtime:
      if search_result == False:
        return result
      
      # records remaining time
      timebound = timebound - (time.time() - starttime)
      starttime = time.time()
      
      # prune based on g_vlaue of node
      if search_result.gval <= cost_bound[0]:
        cost_bound = (search_result.gval, search_result.gval, search_result.gval)
        result = search_result
      print(time.time())
      print(endtime)  
      search_result = se.search(timebound, cost_bound)
    
    return result
