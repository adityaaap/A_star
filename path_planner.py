"""
    This module is your primary workspace. Add whatever helper functions, classes, data structures, imports... etc here.

    We expect most results will utilize more than just dumping code into the plan_paths()
        function, that just serves as a meaningful entry point.

    In order for the rest of the scoring to work, you need to make sure you have correctly
        populated the DeliverySite.path for each result you produce.
"""
import typing
from heapq import heappop, heappush

import numpy as np

from nest_info import Coordinate, DeliverySite, NestInfo
import copy
from A_star import AStar

class PathPlanner:
    def __init__(self, nest_info: NestInfo, delivery_sites: typing.List["DeliverySite"]):
        self.nest_info: NestInfo = nest_info
        self.delivery_sites: typing.List["DeliverySite"] = delivery_sites
        
    def plan_paths(self):
        """
        This is the main planning function. It is expected to mutate the list of
        delivery_sites by calling each DeliverySite's set_path() with the resulting
        path as an argument.

        The default construction shows this format, and should produce 10 invalid paths.
        """
        count = 0
        for site in self.delivery_sites:
            print(count)
            path_length = 100
            count += 1
            weight = 1
            while(path_length > self.nest_info.maximum_range):
                planner = AStar(copy.deepcopy(self.nest_info.risk_zones),weight)
                cost_map = copy.deepcopy(self.nest_info.risk_zones)
                path_array = planner.run(cost_map,[self.nest_info.nest_coord.e, self.nest_info.nest_coord.n], [site.coord.e, site.coord.n])
                path_coords = [Coordinate(arr[0], arr[1]) for arr in path_array]
                #function copied from score_paths
                path_length = np.sum(np.linalg.norm(np.diff(path_coords, axis=0), axis=1))
                weight+=0.1
                # print(path_length)
            site.set_path(path_coords)


