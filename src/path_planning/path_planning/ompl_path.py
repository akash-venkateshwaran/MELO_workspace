from ompl import base as ob
from ompl import geometric as og
from ompl import util as ou
from rclpy.impl.rcutils_logger import RcutilsLogger
from typing import List
from pyproj import Geod
import math
import alphashape
from shapely.geometry import Polygon, Point
from matplotlib.patches import PathPatch
from matplotlib.path import Path
import time


import numpy as np
import plotly.graph_objs as go
from plotly.subplots import make_subplots
import matplotlib.pyplot as plt

from path_planning.objectives import get_sailing_objective
from custom_interfaces.msg import State, HelperPosition, HeadingAngle

ou.setLogLevel(ou.LOG_WARN)
GEODESIC = Geod(ellps="WGS84")


class OMPLPathState:
    def __init__(self,ship_current_state: State, mammal_current_state: State, ship_start_state: State, ship_end_state: State, bathy_points: List[HelperPosition], count: float, shoreline_polygons: List[Polygon] ):
        # TODO: derive OMPLPathState attributes from navigation
        # Note that when you convert latlon to XY the distance btw two points in XY is in km

        def _init_helper(state):
            return HelperPosition(
                    latitude=state.position.latitude,
                    longitude=state.position.longitude,
                    depth=state.position.depth
                )
        

        

        # Creating HelperPosition for all the inputs
        self.mammal_position = _init_helper(mammal_current_state)
        self.ship_position = _init_helper(ship_current_state)
        self.start_position = self.reference_latlon = _init_helper(ship_start_state)
        self.goal_position = _init_helper(ship_end_state)
        self.count = count
        self.bathy_points = bathy_points
        self.shoreline_polygons = shoreline_polygons


        # Coverting HelperPosition from lat/lon to XY
        self.start_position_XY = self.latlon_to_xy(reference=self.reference_latlon,latlon=self.start_position)
        self.goal_position_XY = self.latlon_to_xy(reference=self.reference_latlon,latlon=self.goal_position)
        self.mammal_position_XY = self.latlon_to_xy(reference=self.reference_latlon,latlon=self.mammal_position)
        self.ship_position_XY = self.latlon_to_xy(reference=self.reference_latlon,latlon=self.ship_position)
        self.shoreline_polygons_XY = []
        for poly in shoreline_polygons:
            xy_polygon = self.convert_polygon_latlon_to_xy(reference=self.reference_latlon, polygon=poly)
            self.shoreline_polygons_XY.append(Polygon(xy_polygon))

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
        latitudes = [point.latitude for point in bathy_points]
        longitudes = [point.longitude for point in bathy_points]
        self.domain_lat = (min(latitudes),  max(latitudes))
        self.range_lon = (min(longitudes), max(longitudes))
        # We'll use the four corners: (min_lat, min_lon), (min_lat, max_lon), (max_lat, min_lon), (max_lat, max_lon)
        # Then find the min and max XY values from these points
        min_lat_xy = self.latlon_to_xy(self.reference_latlon, HelperPosition(latitude=self.domain_lat[0], longitude=self.range_lon[0], depth=0.0))
        max_lat_xy = self.latlon_to_xy(self.reference_latlon, HelperPosition(latitude=self.domain_lat[1], longitude=self.range_lon[0], depth=0.0))
        min_lon_xy = self.latlon_to_xy(self.reference_latlon, HelperPosition(latitude=self.domain_lat[0], longitude=self.range_lon[1], depth=0.0))
        max_lon_xy = self.latlon_to_xy(self.reference_latlon, HelperPosition(latitude=self.domain_lat[1], longitude=self.range_lon[1], depth=0.0))

        # Now extract the minimum and maximum X and Y values to define the domain and range in XY coordinates
        min_x = min(min_lat_xy[0], max_lat_xy[0], min_lon_xy[0], max_lon_xy[0])
        max_x = max(min_lat_xy[0], max_lat_xy[0], min_lon_xy[0], max_lon_xy[0])
        min_y = min(min_lat_xy[1], max_lat_xy[1], min_lon_xy[1], max_lon_xy[1])
        max_y = max(min_lat_xy[1], max_lat_xy[1], min_lon_xy[1], max_lon_xy[1])

        # Assign domain_XY and range_XY using the minimum and maximum XY values
        self.domain_XY = (min_x, max_x)
        self.range_XY = (min_y, max_y)

        self.path_cost = []
        self.path_goal_dist = []




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

        # Add the new shoreline polygons to the geographic plot
        for poly in self.shoreline_polygons:
                x, y = poly.exterior.xy
                # Convert x, y to lists explicitly
                x_list, y_list = list(x), list(y)
                fig.add_trace(
                    go.Scattergeo(lon=x_list, lat=y_list, mode='lines', name='Shoreline Lat/Lon',
                                line=dict(width=2, color='yellow')),
                    row=1, col=1
                )

    # Assuming shoreline_polygons_XY is a list of Shapely Polygons for the XY plot
        if self.shoreline_polygons_XY:  # Here, assuming it's a regular list; no change needed
            for poly in self.shoreline_polygons_XY:
                x, y = poly.exterior.xy
                # Convert x, y to lists explicitly if needed
                x_list, y_list = list(x), list(y)
                fig.add_trace(
                    go.Scatter(x=x_list, y=y_list, mode='lines', name='Shoreline XY',
                            line=dict(width=2, color='yellow')),
                    row=1, col=2
                )

        # Update layout for better visualization
        fig.update_geos(
        projection_type="natural earth",
        lataxis_range=[self.domain_lat[0], self.domain_lat[1]],  # Set latitude limits
        lonaxis_range=[self.range_lon[0], self.range_lon[1]],    # Set longitude limits
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
        showframe=True
    )

        # Update Cartesian subplot layout with XY ranges
        fig.update_xaxes(range=[self.domain_XY[0], self.domain_XY[1]], row=1, col=2)  # Set X-axis limits
        fig.update_yaxes(range=[self.range_XY[0], self.range_XY[1]], row=1, col=2)    # Set Y-axis limits

        # Update overall layout for better visualization
        fig.update_layout(
            title_text='Geographic vs Cartesian States',
            height=600, width=1200
        )

        # Show the figure
        fig.write_html("figure2.html")

    def convert_polygon_latlon_to_xy(self, reference: HelperPosition, polygon: Polygon) -> List[tuple]:
        """Convert a Polygon from geographical coordinates to 2D Cartesian coordinates."""
        xy_points = []
        for point in polygon.exterior.coords:
            latlon = HelperPosition(latitude=point[1], longitude=point[0], depth=0.0)  # Assuming no depth for shoreline
            xy = self.latlon_to_xy(reference, latlon)
            xy_points.append(xy)
        return xy_points



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
        self.solved = False
        # self.solved = self._simple_setup.solve(time=max_runtime)  # time is in seconds

        self.solve_incrementally(max_runtime, increment=10.0)
        self.fig = plt.figure()
        self.plot_solution()

    def get_cost(self):
        """Get the cost of the path generated.

        Raises:
            NotImplementedError: Method or function hasn't been implemented yet.
        """
        raise NotImplementedError

    def get_waypoints(self, format: str = 'latlon') -> List[tuple]:
        """Get a list of waypoints including the start and goal position for the boat to follow.

        Args:
            format (str): The format of the waypoints, 'latlon' for latitude and longitude, 'xy' for XY coordinates.

        Returns:
            list: A list of waypoints. Each waypoint is a tuple representing either the latitude and longitude or the X and Y coordinates,
                  depending on the specified format. Outputs an empty list and prints a warning message if path not solved.
        """
        if not self.solved:
            print("Trying to get the waypoints of an unsolved OMPLPath")
            return []

        solution_path = self._simple_setup.getSolutionPath()
        waypoints = []

        # NOTE if xy the func exports as List[tuple] else export as HelperPOsitions
        for state in solution_path.getStates():
            if format == 'xy':
                waypoint = (state.getX(), state.getY())
            else:
                waypoint = (state.getX(), state.getY())
                waypoint = self.ompl_state.xy_to_latlon(reference=self.ompl_state.reference_latlon, xy=waypoint)
            waypoints.append(waypoint)

        return waypoints
    
    def solve_incrementally(self, max_runtime, increment=10.0):
        start_time = time.time()
        self.ompl_state.path_cost = []
        self.ompl_state.path_goal_dist = []

        while time.time() - start_time < max_runtime:
            self._simple_setup.solve(increment)
            self.solved = self._simple_setup.haveSolutionPath()
            # print(self._simple_setup.getPlanner().numIterations())
            if self.solved:
                solution_path = self._simple_setup.getSolutionPath()
                current_cost = solution_path.cost(self._simple_setup.getOptimizationObjective()).value()
                self.ompl_state.path_cost.append(current_cost)
                
                last_waypoint_distance = self.distance_from_goal()
                self.ompl_state.path_goal_dist.append(last_waypoint_distance)
        self.plot_convergence(increment) # Exporting convergence plot
        # Final checks after the loop ends.
        if self.solved:
            self.final_solution = self._simple_setup.getSolutionPath()
            self.final_cost = self.final_solution.cost(self._simple_setup.getOptimizationObjective()).value()
        else:
            print("No solution found.")

    def distance_from_goal(self):
        if self.solved:
            waypoints = self.get_waypoints('xy')
            if waypoints:
                last_waypoint = waypoints[-1]
                goal = (self.ompl_state.goal_position_XY[0], self.ompl_state.goal_position_XY[1])
                distance = math.sqrt((last_waypoint[0] - goal[0]) ** 2 + (last_waypoint[1] - goal[1]) ** 2)
                return distance  # Distance in the same units as your state space (typically meters or kilometers)
        return float('inf')  # Return an infinite distance if unsolved or no waypoints



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
        simple_setup.setStateValidityChecker(ob.StateValidityCheckerFn(self.is_state_valid))

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

        planner = og.ABITstar(space_information)
        planner.setUseKNearest(True)

        # planner = og.InformedRRTstar(space_information)
        # planner.setTreePruning(True)
        # planner.setPruneThreshold(0.5)
        # planner.setRange(10)
        simple_setup.setPlanner(planner)

        return simple_setup
    
    
    def plot_solution(self):
        self.fig.clf()  # Clear the figure
        ax = self.fig.add_subplot(111)  # For the combined plot

        # Prepare data for contour plot
        x = np.linspace(self.ompl_state.domain_XY[0], self.ompl_state.domain_XY[1], 500)
        y = np.linspace(self.ompl_state.range_XY[0], self.ompl_state.range_XY[1], 500)
        X, Y = np.meshgrid(x, y)
        Z = np.zeros(X.shape)  # This remains for plotting costs

        # Compute the cost
        space = self._simple_setup.getStateSpace()
        objective = self._simple_setup.getOptimizationObjective()
        for i in range(X.shape[0]):
            for j in range(X.shape[1]):
                # Correct way to create and use OMPL state objects
                state = space.allocState()  # Allocate a new state object from the state space
                state.setX(X[i][j])
                state.setY(Y[i][j])
                    
                # Compute cost normally for Z
                cost = objective.stateCost(state)  # Correctly pass the OMPL State object
                Z[i][j] = cost.value()
        

        # Contour plot for costs
        cp = ax.contourf(X, Y, Z, levels=50, cmap='viridis')
        self.fig.colorbar(cp, ax=ax, orientation='vertical', label='Cost')

        # Set titles and labels
        ax.set_title('Solution Space with Waypoints')
        ax.set_xlabel('X')
        ax.set_ylabel('Y')
        # ax.set_aspect('equal', adjustable='box')

        for poly in self.ompl_state.shoreline_polygons_XY:
            x, y = poly.exterior.xy
            vertices = list(zip(x, y)) + [(x[0], y[0])]
            codes = [Path.MOVETO] + [Path.LINETO] * (len(vertices) - 2) + [Path.CLOSEPOLY]
            path = Path(vertices, codes)
            patch = PathPatch(path, facecolor='black', lw=2, edgecolor='black', alpha=0.5, zorder=10)
            ax.add_patch(patch)

        ax.set_xlim(self.ompl_state.domain_XY)
        ax.set_ylim(self.ompl_state.range_XY)

        waypoints = self.get_waypoints('xy')
        waypoints_x = [wp[0] for wp in waypoints]
        waypoints_y = [wp[1] for wp in waypoints]
        ax.plot(waypoints_x, waypoints_y, '-o', label='Waypoints XY', color='white')

        # Add markers for special points
        ax.scatter([self.ompl_state.start_position_XY[0]], [self.ompl_state.start_position_XY[1]], c='green', label='Start Position', zorder=5, edgecolors='black')
        ax.scatter([self.ompl_state.goal_position_XY[0]], [self.ompl_state.goal_position_XY[1]], c='orange', label='Goal Position', zorder=5, edgecolors='black')
        ax.scatter([self.ompl_state.ship_position_XY[0]], [self.ompl_state.ship_position_XY[1]], c='blue', label='Ship Position', zorder=5, edgecolors='black')
        ax.scatter([self.ompl_state.mammal_position_XY[0]], [self.ompl_state.mammal_position_XY[1]], c='red', label='Mammal Position', zorder=5, edgecolors='black')

        ax.legend()

        # Save the figure
        filename = f"Fig_{self.ompl_state.count}.png"  # Assuming self.ompl_state.count is correctly updated
        self.fig.savefig("imgs/"+filename)

    def plot_convergence(self, increment: float):
        if self.ompl_state.path_cost and self.ompl_state.path_goal_dist:
            fig, ax1 = plt.subplots()

            time_axis = [increment * (i+1) for i in range(len(self.ompl_state.path_cost))]

            color = 'tab:blue'
            ax1.set_xlabel('Time (s)')
            ax1.set_ylabel('Path Cost', color=color)
            ax1.plot(time_axis, self.ompl_state.path_cost, color=color)
            ax1.tick_params(axis='y', labelcolor=color)

            ax2 = ax1.twinx() 
            color = 'tab:red'
            ax2.set_ylabel('Distance from Goal (km)', color=color) 
            ax2.plot(time_axis, self.ompl_state.path_goal_dist, color=color)
            ax2.tick_params(axis='y', labelcolor=color)

            fig.tight_layout()
            plt.title('Convergence over Time')

            # Save the figure
            filename = f"convergence_{self.ompl_state.count}.png"
            plt.savefig(f"imgs/{filename}")
            plt.close()
        else:
            print("No data available to plot convergence.")


    def is_state_valid(self, state: ob.SE2StateSpace) -> bool:
        point = Point(state.getX(), state.getY())  # Convert the OMPL state to a Shapely Point
        for island_polygon in self.ompl_state.shoreline_polygons_XY:  
            if island_polygon.contains(point):
                return False
        return True
        
    # def is_state_valid(self, state: ob.SE2StateSpace) -> bool:
    #     point = Point(state.getX(), state.getY())  # Convert the OMPL state to a Shapely Point
        
    #     # Define the center and radius of the forbidden circle
    #     circle_center = Point(100, -100)
    #     circle_radius = 1
        
    #     # Create a circular region around the center with the specified radius
    #     forbidden_circle = circle_center.buffer(circle_radius)
    #     if forbidden_circle.contains(point):
    #         return False

        return True




# def main(args=None):
#     ompl_path = OMPLPath(max_runtime=1.0)
#     if ompl_path.solved:
#             # self.get_logger("Solved")
#         print("Solved")
#         ompl_path.plot_solution()

# if __name__ == '__main__':
#     main()
