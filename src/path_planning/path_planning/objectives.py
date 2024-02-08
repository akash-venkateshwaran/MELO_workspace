from ompl import base as ob
from ompl import geometric as og
import math
import plotly.graph_objects as go

from custom_interfaces.msg import State, HelperPosition, HeadingAngle

class EuclideanDistanceObjective(ob.StateCostIntegralObjective):
    def __init__(self, si, goal):
        super().__init__(si)
        self.goal = goal

    def stateCost(self, s):
        distance = math.sqrt((s[0] - self.goal[0]) ** 2 + (s[1] - self.goal[1]) ** 2)
        return distance
    



def get_sailing_objective(
    space_information,
    simple_setup,
    ship_state: State(),
    mammal_state: State()
) -> ob.OptimizationObjective:
    objective = ob.MultiOptimizationObjective(si=space_information)
    objective.addObjective(
        objective=EuclideanDistanceObjective(space_information),
        weight=1.0,
    )
    return objective