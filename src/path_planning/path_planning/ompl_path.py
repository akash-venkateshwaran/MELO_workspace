from ompl import base as ob
from ompl import geometric as og
from ompl import util as ou
from rclpy.impl.rcutils_logger import RcutilsLogger
from typing import List
from pyproj import Geod
import math

import numpy as np
import plotly.graph_objs as go
from plotly.subplots import make_subplots

from path_planning.objectives import get_sailing_objective
from custom_interfaces.msg import State, HelperPosition, HeadingAngle

ou.setLogLevel(ou.LOG_WARN)
GEODESIC = Geod(ellps="WGS84")


class OMPLPathState:
    def __init__(self,ship_current_state: State, mammal_current_state: State, ship_start_state: State, ship_end_state: State ):
        # TODO: derive OMPLPathState attributes from navigation
        # Note that when you convert latlon to XY the distance btw two points in XY is in km

        def init_helper(state):
            return HelperPosition(
                    latitude=state.position.latitude,
                    longitude=state.position.longitude,
                    depth=state.position.depth
                )

        # Initialize state objects
        self.mammal_position = init_helper(mammal_current_state)
        self.ship_position = init_helper(ship_current_state)
        self.start_position = self.reference_latlon = init_helper(ship_start_state)
        self.goal_position = init_helper(ship_end_state)

        self.start_position_XY = self.latlon_to_xy(reference=self.reference_latlon,latlon=self.start_position)
        self.goal_position_XY = self.latlon_to_xy(reference=self.reference_latlon,latlon=self.goal_position)
        self.mammal_position_XY = self.latlon_to_xy(reference=self.reference_latlon,latlon=self.mammal_position)
        self.ship_position_XY = self.latlon_to_xy(reference=self.reference_latlon,latlon=self.ship_position)

        # adding 1 km buffer for domain and range of XY
        self.domain_XY = (min(self.start_position_XY[0], self.goal_position_XY[0], self.mammal_position_XY[0], self.ship_position_XY[0]) - 1, max(self.start_position_XY[0], self.goal_position_XY[0], self.mammal_position_XY[0], self.ship_position_XY[0]) + 1)
        self.range_XY = (min(self.start_position_XY[1], self.goal_position_XY[1], self.mammal_position_XY[1], self.ship_position_XY[1]) - 1, max(self.start_position_XY[1], self.goal_position_XY[1], self.mammal_position_XY[1], self.ship_position_XY[1]) + 1)

        print(self.ship_position_XY)




        # latlon_states = [self.start_position, self.goal_position, self.mammal_position, self.ship_position]
        # xy_states = [self.start_position_XY, self.goal_position_XY, self.mammal_position_XY, self.ship_position_XY]
        # self.plot_latlon_xy_side_by_side(latlon_states, xy_states)

    def cartesian_to_true_bearing(self,cartesian: float) -> float:
        """Convert a cartesian angle to the equivalent true bearing.

        Args:
            cartesian (float): Angle where 0 is east and values increase counter-clockwise.

        Returns:
            float: Angle where 0 is north and values increase clockwise.
        """
        return (90 - cartesian + 360) % 360


    def meters_to_km(self,meters: float) -> float:
        return meters / 1000


    def km_to_meters(self,km: float) -> float:
        return km * 1000
    
    def latlon_to_xy(self,reference: HelperPosition, latlon: HelperPosition) -> tuple:
        """Convert a geographical coordinate to a 2D Cartesian coordinate given a reference point.

        Args:
            reference (HelperLatLon): Origin of the Cartesian coordinate system.
            latlon (HelperLatLon): Coordinate to be converted to the Cartesian coordinate system.

        Returns:
            XY: The x and y components in km.
        """
        forward_azimuth_deg, _, distance_m = GEODESIC.inv(
            reference.longitude, reference.latitude, latlon.longitude, latlon.latitude
        )
        true_bearing = math.radians(forward_azimuth_deg)
        distance = self.meters_to_km(distance_m)

        return (distance * math.sin(true_bearing),distance * math.cos(true_bearing))


    def xy_to_latlon(self,reference: HelperPosition, xy: tuple) -> HelperPosition:
        """Convert a 2D Cartesian coordinate to a geographical coordinate given a reference point.

        Args:
            reference (HelperLatLon): Coordinate that is the origin of the Cartesian coordinate system.
            xy (XY): Coordinate to be converted to the geographical coordinate system.

        Returns:
            HelperLatLon: The latitude and longitude in degrees.
        """
        true_bearing = math.degrees(math.atan2(xy.x, xy.y))
        distance = self.km_to_meters(math.hypot(*xy))
        dest_lon, dest_lat, _ = GEODESIC.fwd(
            reference.longitude, reference.latitude, true_bearing, distance
        )

        return HelperPosition(latitude=dest_lat, longitude=dest_lon)
    
    def plot_latlon_xy_side_by_side(self, latlon_states, xy_states):
        # print(latlon_states, "\n\n")
        # print(xy_states)
        # Create subplots: one for geographic data, one for Cartesian data
        fig = make_subplots(
            rows=1, cols=2,
            subplot_titles=("Geographic (Lat/Lon) States", "Cartesian (XY) States"),
            specs=[[{"type": "scattergeo"}, {"type": "xy"}]]  # Specify the types of plots for each subplot
        )

        # Define colors for different states for visualization
        colors = ['red', 'green', 'blue', 'purple']  # Add more colors if there are more than four states

        # Add geographic (lat/lon) plot data
        for idx, state in enumerate(latlon_states):
            fig.add_trace(
                go.Scattergeo(lat=[state.latitude], lon=[state.longitude],
                            mode='markers', name=f'Lat/Lon {idx}',
                            marker=dict(color=colors[idx], size=10)),
                row=1, col=1
            )

        # Add Cartesian (XY) plot data, using the same colors
        for idx, state in enumerate(xy_states):
            fig.add_trace(
                go.Scatter(x=[state[0]], y=[state[1]], mode='markers', name=f'XY {idx}',
                        marker=dict(color=colors[idx], size=10)),  # Use same color for corresponding XY states
                row=1, col=2
            )

        # Update layout for better visualization
        fig.update_geos(
            projection_type="natural earth",
            landcolor="rgb(243, 243, 243)",
            oceancolor="rgb(204, 230, 255)",
            showcountries=True,
            countrycolor="rgb(204, 204, 204)",
            showcoastlines=True,
            coastlinecolor="rgb(102, 102, 102)",
            showland=True,
            showocean=True,
            showlakes=True,
            lakecolor="rgb(204, 230, 255)",
            showrivers=True,
            rivercolor="rgb(204, 230, 255)",
            showsubunits=True,
            subunitcolor="rgb(204, 204, 204)",
            showframe=True,
            fitbounds="locations"
        )

        fig.update_layout(
            title_text='Geographic vs Cartesian States',
            height=600, width=1200
        )

        # Show the figure
        fig.write_html("figure2.html")


class OMPLPath:
    """Represents the general OMPL Path.

    Attributes
        _logger (RcutilsLogger): ROS logger of this class.
        _simple_setup (og.SimpleSetup): OMPL SimpleSetup object.
        solved (bool): True if the path is a solution to the OMPL query, else false.
    """

    def __init__(
        self,
        ompl_state: OMPLPathState,
        # parent_logger: RcutilsLogger,
        max_runtime: float
    ):
        """Initialize the OMPLPath Class. Attempt to solve for a path.

        Args:
            parent_logger (RcutilsLogger): Logger of the parent class.
            max_runtime (float): Maximum amount of time in seconds to look for a solution path.
            local_path_state (LocalPathState): State of Sailbot.
        """
        # self._logger = parent_logger.get_child(name="ompl_path")

        self._simple_setup = self._init_simple_setup(ompl_state)
        self.solved = self._simple_setup.solve(time=max_runtime)  # time is in seconds

    def get_cost(self):
        """Get the cost of the path generated.

        Raises:
            NotImplementedError: Method or function hasn't been implemented yet.
        """
        raise NotImplementedError

    def get_waypoints(self) -> List[HelperPosition]:
        """Get a list of waypoints for the boat to follow.

        Returns:
            list: A list of tuples representing the x and y coordinates of the waypoints.
                  Output an empty list and print a warning message if path not solved.
        """
        if not self.solved:
            self._logger.warning("Trying to get the waypoints of an unsolved OMPLPath")
            return []

        solution_path = self._simple_setup.getSolutionPath()

        waypoints = []

        for state in solution_path.getStates():
            #TODO Convert XY to lat lon before appending
            #  waypoint_XY = cs.XY(state.getX(), state.getY())
            # waypoint_latlon = cs.xy_to_latlon(self.state.reference_latlon, waypoint_XY)
            waypoints.append(
                HelperPosition(
                    latitude=state.getX(), longitude=state.getY(), depth = 0.0
                )
            )

        return waypoints
    
    def update_objectives(self):
        """Update the objectives on the basis of which the path is optimized.
        Raises:
            NotImplementedError: Method or function hasn't been implemented yet.
        """
        raise NotImplementedError

    def _init_simple_setup(self,ompl_state: OMPLPathState) -> og.SimpleSetup:
        """Initialize and configure the OMPL SimpleSetup object.

        Returns:
            og.SimpleSetup: Encapsulates the various objects necessary to solve a geometric or
                control query in OMPL.
        """
        space = ob.SE2StateSpace()
        self.ompl_state = ompl_state

        # set the bounds of the state space
        bounds = ob.RealVectorBounds(dim=2)
        bounds.setLow(index=0, value=self.ompl_state.domain_XY[0])
        bounds.setLow(index=1, value=self.ompl_state.range_XY[0])
        bounds.setHigh(index=0, value=self.ompl_state.domain_XY[1])
        bounds.setHigh(index=1, value=self.ompl_state.range_XY[1])

        # self._logger.debug(
        #     "state space bounds: "
        #     f"x=[{bounds.low[0]}, {bounds.high[0]}]; "
        #     f"y=[{bounds.low[1]}, {bounds.high[1]}]"
        # )
        print(
            "state space bounds: "
            f"x=[{bounds.low[0]}, {bounds.high[0]}]; "
            f"y=[{bounds.low[1]}, {bounds.high[1]}]"
        )
        bounds.check()  # check if bounds are valid
        space.setBounds(bounds)

        # create a simple setup object
        simple_setup = og.SimpleSetup(space)
        # simple_setup.setStateValidityChecker(ob.StateValidityCheckerFn(is_state_valid))

        # set the goal and start states of the simple setup object
        start = ob.State(space)
        goal = ob.State(space)
        start().setXY(self.ompl_state.start_position_XY[0], self.ompl_state.start_position_XY[1])
        goal().setXY(self.ompl_state.goal_position_XY[0], self.ompl_state.goal_position_XY[1])

        # self._logger.debug(
        #     "start and goal state: "
        #     f"start=({start().getX()}, {start().getY()}); "
        #     f"goal=({goal().getX()}, {goal().getY()})"
        # )
        print(
            "start and goal state: "
            f"start=({start().getX()}, {start().getY()}); "
            f"goal=({goal().getX()}, {goal().getY()})"
        )
        
        simple_setup.setStartAndGoalStates(start, goal)

        # Constructs a space information instance for this simple setup
        space_information = simple_setup.getSpaceInformation()

        # set the optimization objective of the simple setup object
        # TODO: implement and add optimization objective here

        objective = get_sailing_objective(
            space_information,
            simple_setup,
            self.ompl_state.ship_position_XY,
            self.ompl_state.mammal_position_XY
        )
        simple_setup.setOptimizationObjective(objective)

        # set the planner of the simple setup object
        # TODO: implement and add planner here
        planner = og.RRTstar(space_information)
        simple_setup.setPlanner(planner)

        return simple_setup
    
    def plot_solution(self):
        x = np.linspace(self.ompl_state.domain_XY[0], self.ompl_state.domain_XY[1], 500)
        y = np.linspace(self.ompl_state.range_XY[0], self.ompl_state.range_XY[1], 500)
        X, Y = np.meshgrid(x, y)
        Z = np.zeros(X.shape)

        space = self._simple_setup.getStateSpace()
        objective = self._simple_setup.getOptimizationObjective()
        waypoints_XY = self.get_waypoints()

        for i in range(X.shape[0]):
            for j in range(X.shape[1]):
                state = ob.State(space)
                state().setXY(X[i][j], Y[i][j])
                raw_state = state.get()
                cost = objective.stateCost(raw_state)
                Z[i][j] = cost.value()
        
        fig = go.Figure(data=go.Contour(x=x, y=y, z=Z))

        if waypoints_XY: 
            wp_X = [wp.latitude for wp in waypoints_XY] 
            wp_Y = [wp.longitude for wp in waypoints_XY]
            fig.add_trace(go.Scatter(x=wp_X, y=wp_Y, mode='markers+lines', name='Waypoints XY'))

        # Scatter plot for start and goal positions
        fig.add_trace(go.Scatter(x=[self.ompl_state.start_position_XY[0]], y=[self.ompl_state.start_position_XY[1]],
                                mode='markers', name='Start Position', marker=dict(color='green', size=12)))
        fig.add_trace(go.Scatter(x=[self.ompl_state.goal_position_XY[0]], y=[self.ompl_state.goal_position_XY[1]],
                                mode='markers', name='Goal Position', marker=dict(color='orange', size=12)))

        # Scatter plot for ship and mammal positions
        fig.add_trace(go.Scatter(x=[self.ompl_state.ship_position_XY[0]], y=[self.ompl_state.ship_position_XY[1]],
                                mode='markers', name='Ship Position', marker=dict(color='blue', size=12)))
        fig.add_trace(go.Scatter(x=[self.ompl_state.mammal_position_XY[0]], y=[self.ompl_state.mammal_position_XY[1]],
                                mode='markers', name='Mammal Position', marker=dict(color='red', size=12)))

        fig.update_layout(title='Solution Contour Plot',
                        xaxis_title='X',
                        yaxis_title='Y')
        
        fig.write_html("figure.html")



def is_state_valid(state: ob.SE2StateSpace) -> bool:
    """Evaluate a state to determine if the configuration collides with an environment obstacle.

    Args:
        state (ob.SE2StateSpace): State to check.

    Returns:
        bool: True if state is valid, else false.
    """
    # TODO: implement obstacle avoidance here
    # note: `state` is of type `SE2StateInternal`, so we don't need to use the `()` operator.
    # eg: return state.getX() < 0.6
    return True



# def main(args=None):
#     ompl_path = OMPLPath(max_runtime=1.0)
#     if ompl_path.solved:
#             # self.get_logger("Solved")
#         print("Solved")
#         ompl_path.plot_solution()

# if __name__ == '__main__':
#     main()
