from ompl import base as ob
from ompl import geometric as og
import math
import plotly.graph_objects as go

from custom_interfaces.msg import State, HelperPosition, HeadingAngle


class Objective(ob.StateCostIntegralObjective):
    """All of our optimization objectives inherit from this class.

    Attributes:
        space_information (StateSpacePtr): Contains all the information about
            the space planning is done in.
        max_motion_cost (float): The maximum motion cost between any two states in the state space.
    """

    def __init__(self, space_information, num_samples: int):
        super().__init__(si=space_information, enableMotionCostInterpolation=True)
        self.space_information = space_information

        states = self.sample_states(num_samples)
        # initialize to 1 so that motionCost is not normalized when finding the maximum motion cost
        self.max_state_cost = 1.0
        self.max_state_cost = self.find_maximum_state_cost(states)

    def find_maximum_state_cost(self, states: list[ob.SE2StateSpace]) -> float:
        """Finds the maximum motion cost between any two states in `states`.

        Args:
            states (list[ob.SE2StateSpace]): OMPL states.

        Returns:
            float: Maximum motion cost.
        """
        return max(
            self.stateCost(s).value()
            for s in states
        )

    def sample_states(self, num_samples: int) -> list[ob.SE2StateSpace]:
        """Samples `num_samples` states from the state space.

        Args:
            num_samples (int): Number of states to sample.

        Returns:
            list[ob.SE2StateSpace]: OMPL states.
        """
        sampler = self.space_information.getStateSpace().allocDefaultStateSampler()

        sampled_states = []

        for _ in range(num_samples):
            state = self.space_information.getStateSpace().allocState()
            sampler.sampleUniform(state)
            sampled_states.append(state)

        return sampled_states

    def normalization(self, cost: float) -> float:
        """Normalizes cost using max_motion_cost and caps it at 1.

        Args:
            cost (float): motionCost value from an objective function.
        Returns:
            float: normalized cost between 0 to 1.
        """
        normalized_cost = cost / self.max_state_cost
        return min(normalized_cost, 1.0)


class EuclideanDistanceMammalObjective(Objective):
    def __init__(self, si,simple_setup, mammal_state: State):
        self.mammal_state = mammal_state
        self.simple_setup = simple_setup
        self.goal_state = simple_setup.getGoal().getState()
        super().__init__(si,num_samples=100)

    def stateCost(self, s:ob.SE2StateSpace)-> ob.Cost:
        distance = math.sqrt((s.getX() - self.mammal_state.position.latitude) ** 2 + (s.getY() - self.mammal_state.position.longitude) ** 2)
        cost = 150*math.exp(-(distance**2)/1000)
        return ob.Cost(cost)
    
    # def motionCost(self, s1: ob.SE2StateSpace, s2: ob.SE2StateSpace) -> ob.Cost:
    #     cost_s1 = math.sqrt((self.goal_state.getX() - s1.getX()) ** 2 + (self.goal_state.getY() - s1.getY()) ** 2)
    #     cost_s2 = math.sqrt((self.goal_state.getX() - s2.getX()) ** 2 + (self.goal_state.getY() - s2.getY()) ** 2)
    #     cost_diff = cost_s2 - cost_s1
    #     return ob.Cost(cost_diff)


class EuclideanDistanceGoalObjective(Objective):
    def __init__(self, si,simple_setup):
        self.simple_setup = simple_setup
        self.goal_state = simple_setup.getGoal().getState()
        super().__init__(si,num_samples=100)

    def stateCost(self, s:ob.SE2StateSpace)-> ob.Cost:
        distance = math.sqrt((s.getX() - self.goal_state.getX()) ** 2 + (s.getY() - self.goal_state.getY()) ** 2)
        return ob.Cost((distance))
    
    # def motionCost(self, s1: ob.SE2StateSpace, s2: ob.SE2StateSpace) -> ob.Cost:
    #     cost_s1 = math.sqrt((self.goal_state.getX() - s1.getX()) ** 2 + (self.goal_state.getY() - s1.getY()) ** 2)
    #     cost_s2 = math.sqrt((self.goal_state.getX() - s2.getX()) ** 2 + (self.goal_state.getY() - s2.getY()) ** 2)
    #     cost_diff = cost_s2 - cost_s1
    #     return ob.Cost(cost_diff)



def get_sailing_objective(
    space_information,
    simple_setup,
    ship_state: State,
    mammal_state: State
) -> ob.OptimizationObjective:
    objective = ob.MultiOptimizationObjective(si=space_information)
    objective.addObjective(
        objective=EuclideanDistanceMammalObjective(space_information,simple_setup, mammal_state = mammal_state),
        weight=0.6,
    )
    objective.addObjective(
        objective=EuclideanDistanceGoalObjective(space_information,simple_setup),
        weight=0.4,
    )
    return objective