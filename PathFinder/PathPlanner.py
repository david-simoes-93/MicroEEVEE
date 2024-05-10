from abc import ABCMeta, abstractmethod
from heapq import heappush, heappop
from typing import Optional, List

from Utils import *
import MapHandler

Infinite = float('inf')


class AStar:
    __metaclass__ = ABCMeta
    __slots__ = ()

    class SearchNode:
        __slots__ = ('data', 'gscore', 'fscore',
                     'closed', 'came_from', 'out_openset')

        def __init__(self, data, gscore=Infinite, fscore=Infinite):
            self.data = data
            self.gscore = gscore
            self.fscore = fscore
            self.closed = False
            self.out_openset = True
            self.came_from = None

        def __lt__(self, b):
            return self.fscore < b.fscore

    class SearchNodeDict(dict):

        def __missing__(self, k):
            v = AStar.SearchNode(k)
            self.__setitem__(k, v)
            return v

    def heuristic_cost_estimate(self, current: MapHandler.Cell, goal: MapHandler.Cell):
        """Computes the estimated (rough) distance between a node and the goal, this method must be implemented in a 
        subclass. The second parameter is always the goal."""
        return dist_manhattan(current.indices, goal.indices)

    def distance_between(self, n0: MapHandler.Cell, n1: MapHandler.Cell, n2: MapHandler.Cell) -> float:
        """Gives the real distance between two adjacent nodes n1 and n2 (i.e n2 belongs to the list of n1's neighbors).
           n2 is guaranteed to belong to the list returned by the call to neighbors(n1).
           This method must be implemented in a subclass."""
        # turning back
        if n0 == n2:
            return 2

        # going in a straight line
        if n0.indices.x == n1.indices.x == n2.indices.x or n0.indices.y == n1.indices.y == n2.indices.y:
            return 1

        # having to turn
        return 1.5
    
    def distance_between_favoring_turns(self, n0: MapHandler.Cell, n1: MapHandler.Cell, n2: MapHandler.Cell) -> float:
        """Gives the real distance between two adjacent nodes n1 and n2 (i.e n2 belongs to the list of n1's neighbors).
           n2 is guaranteed to belong to the list returned by the call to neighbors(n1).
           This method must be implemented in a subclass."""
        # turning back
        if n0 == n2:
            return 2

        # going in a straight line
        if n0.indices.x == n1.indices.x == n2.indices.x or n0.indices.y == n1.indices.y == n2.indices.y:
            return 1

        # having to turn
        return 1

    def is_goal_reached(self, current: MapHandler.Cell, goal: MapHandler.Cell):
        """ returns true when we can consider that 'current' is the goal"""
        return current.indices == goal.indices

    def reconstruct_path(self, last, reversePath=False) -> List[MapHandler.Cell]:
        def _gen():
            current = last
            while current:
                yield current.data
                current = current.came_from

        if reversePath:
            return _gen()
        else:
            return reversed(list(_gen()))

    def astar(self, start: MapHandler.Cell, goal: MapHandler.Cell, 
              reversePath: bool=False, current: Optional[MapHandler.Cell]=None) -> List[MapHandler.Cell]:
        if self.is_goal_reached(start, goal):
            return [start]
        searchNodes = AStar.SearchNodeDict()
        startNode = searchNodes[start] = AStar.SearchNode(
            start, gscore=.0, fscore=self.heuristic_cost_estimate(start, goal))
        openSet = []
        heappush(openSet, startNode)
        if current is None:
            current = startNode
        while openSet:
            previous = current
            current = heappop(openSet)
            if self.is_goal_reached(current.data, goal):
                return list(self.reconstruct_path(current, reversePath))
            current.out_openset = True
            current.closed = True
            for neighbor in [searchNodes[n] for n in current.data.neighbors]:
                if neighbor.closed:
                    continue
                tentative_gscore = current.gscore + \
                                   self.distance_between(previous.data, current.data, neighbor.data)
                if tentative_gscore >= neighbor.gscore:
                    continue
                neighbor.came_from = current
                neighbor.gscore = tentative_gscore
                neighbor.fscore = tentative_gscore + \
                                  self.heuristic_cost_estimate(neighbor.data, goal)
                if neighbor.out_openset:
                    neighbor.out_openset = False
                    heappush(openSet, neighbor)
        return []
