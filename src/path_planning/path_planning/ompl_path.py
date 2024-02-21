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
import matplotlib.pyplot as plt

from path_planning.objectives import get_sailing_objective
from custom_interfaces.msg import State, HelperPosition, HeadingAngle

ou.setLogLevel(ou.LOG_WARN)
GEODESIC = Geod(ellps="WGS84")


class OMPLPathState:
    def __init__(self,ship_current_state: State, mammal_current_state: State, ship_start_state: State, ship_end_state: State ):
        # TODO: derive OMPLPathState attributes from navigation
        # Note that when you convert latlon to XY the distance btw two points in XY is in km

        def _init_helper(state):
            return HelperPosition(
                    latitude=state.position.latitude,
                    longitude=state.position.longitude,
                    depth=state.position.depth
                )
        def _bounds_helper(positions, buffer=10):
            # Calculate max with added buffer 10 km
            min_value = -max(np.abs(positions)) - buffer
            max_value = max(np.abs(positions)) + buffer
            return (min_value, max_value)

        

        # Creating HelperPosition for all the inputs
        self.mammal_position = _init_helper(mammal_current_state)
        self.ship_position = _init_helper(ship_current_state)
        self.start_position = self.reference_latlon = _init_helper(ship_start_state)
        self.goal_position = _init_helper(ship_end_state)

        # Coverting HelperPosition from lat/lon to XY
        self.start_position_XY = self.latlon_to_xy(reference=self.reference_latlon,latlon=self.start_position)
        self.goal_position_XY = self.latlon_to_xy(reference=self.reference_latlon,latlon=self.goal_position)
        self.mammal_position_XY = self.latlon_to_xy(reference=self.reference_latlon,latlon=self.mammal_position)
        self.ship_position_XY = self.latlon_to_xy(reference=self.reference_latlon,latlon=self.ship_position)

        # Replacing lat lon of states with XY
        self.ship_current_state_XY = State()
        self.ship_current_state_XY.position = HelperPosition(latitude=self.ship_position_XY[0], longitude=self.ship_position_XY[1], depth=ship_current_state.position.depth)
        self.ship_current_state_XY.angle = ship_current_state.angle
        self.ship_current_state_XY.speed = ship_current_state.speed

        self.mammal_current_state_XY = State()
        self.mammal_current_state_XY.position = HelperPosition(latitude=self.mammal_position_XY[0], longitude=self.mammal_position_XY[1], depth=mammal_current_state.position.depth)
        self.mammal_current_state_XY.angle = mammal_current_state.angle
        self.mammal_current_state_XY.speed = mammal_current_state.speed

        # The state bounds for OMPL
        self.domain_XY = _bounds_helper([self.start_position_XY[0], self.goal_position_XY[0], self.mammal_position_XY[0], self.ship_position_XY[0]])
        self.range_XY = _bounds_helper([self.start_position_XY[1], self.goal_position_XY[1], self.mammal_position_XY[1], self.ship_position_XY[1]])


        # latlon_pos = [self.start_position, self.goal_position, self.mammal_position, self.ship_position]
        # xy_pos = [self.start_position_XY, self.goal_position_XY, self.mammal_position_XY, self.ship_position_XY]
        # self.plot_latlon_xy_side_by_side(latlon_pos, xy_pos)


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
        true_bearing = math.degrees(math.atan2(xy[0], xy[1]))
        distance = self.km_to_meters(math.hypot(*xy))
        dest_lon, dest_lat, _ = GEODESIC.fwd(
            reference.longitude, reference.latitude, true_bearing, distance
        )

        return HelperPosition(latitude=dest_lat, longitude=dest_lon, depth=0.0)
    
    def plot_latlon_xy_side_by_side(self, latlon_postions, xy_positions):
        # Create subplots: one for geographic data, one for Cartesian data
        fig = make_subplots(
            rows=1, cols=2,
            subplot_titles=("Geographic (Lat/Lon) States", "Cartesian (XY) States"),
            specs=[[{"type": "scattergeo"}, {"type": "xy"}]]  # Specify the types of plots for each subplot
        )

        # Define colors for different states for visualization
        colors = ['red', 'green', 'blue', 'purple']  # Add more colors if there are more than four states

        for idx, pos in enumerate(latlon_postions):
            fig.add_trace(
                go.Scattergeo(lat=[pos.latitude], lon=[pos.longitude],
                            mode='markers', name=f'Lat/Lon {idx}',
                            marker=dict(color=colors[idx], size=10)),
                row=1, col=1
            )

        for idx, pos in enumerate(xy_positions):
            fig.add_trace(
                go.Scatter(x=[pos[0]], y=[pos[1]], mode='markers', name=f'XY {idx}',
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
        self.ompl_state = ompl_state
        self._simple_setup = self._init_simple_setup(ompl_state)
        self.solved = self._simple_setup.solve(time=max_runtime)  # time is in seconds
        self.fig = plt.figure()

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
            waypoint_XY = (state.getX(), state.getY())
            waypoint_latlon = self.ompl_state.xy_to_latlon(reference=self.ompl_state.reference_latlon,xy=waypoint_XY)
            waypoints.append(waypoint_latlon)

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
        start().setXY(self.ompl_state.ship_position_XY[0], self.ompl_state.ship_position_XY[1])
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
            self.ompl_state.ship_current_state_XY,
            self.ompl_state.mammal_current_state_XY
        )
        simple_setup.setOptimizationObjective(objective)

        # set the planner of the simple setup object
        # TODO: implement and add planner here
        planner = og.RRTstar(space_information)
        simple_setup.setPlanner(planner)

        return simple_setup
    
    
    def plot_solution(self):
        # Clear existing axes, if any, and create two new ones
        self.fig.clf()  # Clear the figure
        ax1 = self.fig.add_subplot(121)  # For the contour plot
        ax2 = self.fig.add_subplot(122)  # For the waypoints

        # Prepare data for contour plot
        x = np.linspace(self.ompl_state.domain_XY[0], self.ompl_state.domain_XY[1], 500)
        y = np.linspace(self.ompl_state.range_XY[0], self.ompl_state.range_XY[1], 500)
        X, Y = np.meshgrid(x, y)
        Z = np.zeros(X.shape)

        # Compute the cost for each grid point (this could be time-consuming)
        space = self._simple_setup.getStateSpace()
        objective = self._simple_setup.getOptimizationObjective()
        for i in range(X.shape[0]):
            for j in range(X.shape[1]):
                state = ob.State(space)
                state().setXY(X[i][j], Y[i][j])
                raw_state = state.get()
                cost = objective.stateCost(raw_state)
                Z[i][j] = cost.value()

        # Contour plot
        cp = ax1.contourf(X, Y, Z, levels=50, cmap='viridis')  # Adjust levels and colormap as needed
        self.fig.colorbar(cp, ax=ax1, orientation='vertical', label='Cost')
        ax1.set_title('Solution Space')
        ax1.set_xlabel('X')
        ax1.set_ylabel('Y')

        # Plot waypoints, start, goal, ship, and mammal positions
        waypoints_XY = self.get_waypoints()  # Make sure this returns a list of HelperPosition objects
        if waypoints_XY:
            wp_X = [wp.latitude for wp in waypoints_XY]  # Access latitude as X
            wp_Y = [wp.longitude for wp in waypoints_XY]  # Access longitude as Y
            ax2.plot(wp_X, wp_Y, 'o-', label='Waypoints XY')  # 'o-' for line with markers

        # Add markers for special points
        ax2.scatter([self.ompl_state.start_position_XY[0]], [self.ompl_state.start_position_XY[1]], c='green', label='Start Position', zorder=5)
        ax2.scatter([self.ompl_state.goal_position_XY[0]], [self.ompl_state.goal_position_XY[1]], c='orange', label='Goal Position', zorder=5)
        ax2.scatter([self.ompl_state.ship_position_XY[0]], [self.ompl_state.ship_position_XY[1]], c='blue', label='Ship Position', zorder=5)
        ax2.scatter([self.ompl_state.mammal_position_XY[0]], [self.ompl_state.mammal_position_XY[1]], c='red', label='Mammal Position', zorder=5)

        ax2.set_title('Waypoints and Positions')
        ax2.set_xlabel('X')
        ax2.set_ylabel('Y')
        ax2.legend()

        # Update the plot
        self.fig.canvas.draw_idle()  # Update the figure
        plt.show(block=False)  # Display the figure without blocking the rest of the script




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
