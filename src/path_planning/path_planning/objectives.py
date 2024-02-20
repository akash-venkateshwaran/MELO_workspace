from ompl import base as ob
from ompl import geometric as og
import math
import plotly.graph_objects as go

from custom_interfaces.msg import State, HelperPosition, HeadingAngle

class EuclideanDistanceObjective(ob.StateCostIntegralObjective):
    def __init__(self, si, mammal_state: HelperPosition):
        super().__init__(si)
        print(mammal_state)
        self.mammal_state = mammal_state

    def stateCost(self, s):
        distance = math.sqrt((s.getX() - self.mammal_state[0]) ** 2 + (s.getY() - self.mammal_state[1]) ** 2)
        return 1/(distance+0.01)
    



def get_sailing_objective(
    space_information,
    simple_setup,
    ship_state: HelperPosition,
    mammal_state: HelperPosition
) -> ob.OptimizationObjective:
    objective = ob.MultiOptimizationObjective(si=space_information)
    objective.addObjective(
        objective=EuclideanDistanceObjective(space_information, mammal_state = mammal_state),
        weight=1.0,
    )
    return objective