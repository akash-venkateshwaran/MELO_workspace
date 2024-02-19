from ompl import base as ob
from ompl import geometric as og
import math
import plotly.graph_objects as go

from custom_interfaces.msg import State, HelperPosition, HeadingAngle

class EuclideanDistanceObjective(ob.StateCostIntegralObjective):
    def __init__(self, si, mammal_state: State()):
        super().__init__(si)
        self.mammal_state = mammal_state
        self.mammal_state_temp = (0.0, 0.0)

    def stateCost(self, s):
        distance = math.sqrt((s.getX() - self.mammal_state_temp[0]) ** 2 + (s.getY() - self.mammal_state_temp[1]) ** 2)
        return 1/(distance+0.01)
    



def get_sailing_objective(
    space_information,
    simple_setup,
    ship_state: State(),
    mammal_state: State()
) -> ob.OptimizationObjective:
    objective = ob.MultiOptimizationObjective(si=space_information)
    objective.addObjective(
        objective=EuclideanDistanceObjective(space_information, mammal_state = mammal_state),
        weight=1.0,
    )
    return objective