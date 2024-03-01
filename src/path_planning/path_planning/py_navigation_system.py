import rclpy
from rclpy.node import Node
from custom_interfaces.msg import State, HelperPosition, HeadingAngle, Path
from custom_interfaces.srv import BathyService, ShorelineService
import os
import csv
import math
from pyproj import Geod
import geopandas as gpd
import matplotlib.pyplot as plt
from shapely.geometry import Polygon


from path_planning.ompl_path import OMPLPath, OMPLPathState
GEODESIC = Geod(ellps="WGS84")

class NavigateSystem(Node):
    def __init__(self):
        super().__init__('navigate_system')
        self.setup_logger()
        self.verbose_logging = False
        self.ompl_path_state = None
        self.ship_current_state = State()
        self.mammal_current_state = State()
        self.ship_optimized_waypoints = Path()
        self.ship_optimized_state = State()
        self.temp = 0

        # Attributes for bathymetry data
        # TODO Come up with a logic for domain space
        self.bathy_points = []
        self.shoreline_polygons = []  # List of GeoSeries geometric objects in lat/lon
        self.bathy_max_lat = 52.0 
        self.bathy_min_lat = 48.0  
        self.bathy_max_lon = -121.0  
        self.bathy_min_lon = -130.0 


        self.load_voyage_definition(os.path.join(os.getcwd(), 'src', 'path_planning','Voyage_definition.csv'))
        self.ship_start_state_publisher = self.create_publisher(
            State,
            'ship_start_state',
            10
        )

        self.ship_end_state_publisher = self.create_publisher(
            State,
            'ship_end_state',
            10
        )

        self.ship_optimized_state_publisher = self.create_publisher(
            State,
            'ship_optimized_state',
            10
        )

        self.ship_optimized_waypoints_publisher = self.create_publisher(
            Path,
            'ship_optimized_waypoints',
            10
        )

        # Subscribe to the 'ship_state' topic
        self.ship_state_subscription = self.create_subscription(
            State,
            'ship_state',
            self.ship_state_callback,
            10
        )
        self.ship_state_subscription  # prevent unused variable warning

        # Subscribe to the 'ship_state' topic
        self.mammal_state_subscription = self.create_subscription(
            State,
            'mammal_state',
            self.mammal_state_callback,
            10
        )
        self.ship_state_subscription  # prevent unused variable warning

        # Timer to periodically publish ship_start_state, ship_end_state, and ship_optimized_state
        self.publish_timer = self.create_timer(1, self.publish_states)
        
        self.bathy_client = self.create_client(BathyService, 'bathy_service')
        while not self.bathy_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('BathyService not available, waiting again...')
        self.get_bathy_service_response()

        self.shoreline_client = self.create_client(ShorelineService, 'shoreline_service')
        while not self.shoreline_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('ShorelineService not available, waiting again...')
        self.get_shoreline_service_response()
    
    def setup_logger(self):
        self.get_logger().info("Navigation Node is!")

    def generate_state(self, latitude, longitude, depth, heading, speed):
        return State(
            position=HelperPosition(latitude=latitude, longitude=longitude, depth=depth),
            angle=HeadingAngle(heading=heading),
            speed=speed
        )

    def ship_state_callback(self, msg):
        # Update ship_current_state attribute when a new message is received
        self.ship_current_state = msg
        if self.verbose_logging:
            self.get_logger().info(
                f'I heard Ship State: [{msg.position.latitude}, {msg.position.longitude}, {msg.position.depth}], '
                f'Angle: {msg.angle.heading}, Speed: {msg.speed} m/s')
        self.solve_ompl()
            
    def mammal_state_callback(self, msg):
        # Update ship_current_state attribute when a new message is received
        self.mammal_current_state = msg
        if self.verbose_logging:
            self.get_logger().info(
                f'I heard Mammal State: [{msg.position.latitude}, {msg.position.longitude}, {msg.position.depth}], '
                f'Angle: {msg.angle.heading}, Speed: {msg.speed} m/s')
        self.solve_ompl()

    def publish_states(self):
        # Publish ship_start_state, ship_end_state, and ship_optimized_state periodically
        self.ship_start_state_publisher.publish(self.ship_start_state)
        self.ship_end_state_publisher.publish(self.ship_end_state)
        if self.ship_optimized_state.position.latitude != 0 and self.ship_optimized_state.position.longitude != 0:
            self.ship_optimized_state_publisher.publish(self.ship_optimized_state)
        if len(self.ship_optimized_waypoints.waypoints) > 0:
            self.ship_optimized_waypoints_publisher.publish(self.ship_optimized_waypoints)
        self.get_logger().info("Published states")
            
    def load_voyage_definition(self, file_path):
        try:
            with open(file_path, 'r') as csvfile:
                csvreader = csv.reader(csvfile)
                headers = next(csvreader)
                
                if len(headers) == 5 and headers == ["latitude", "longitude", "depth", "heading", "speed"]:
                    start_state_values = [float(value) for value in next(csvreader)]
                    end_state_values = [float(value) for value in next(csvreader)]
                    voyage_time = float(next(csvreader)[0])

                    self.ship_start_state = self.generate_state(*start_state_values)
                    self.ship_end_state = self.generate_state(*end_state_values)
                    self.voyage_time = voyage_time

                    self.get_logger().info("Voyage definition loaded from CSV.")
                else:
                    self.get_logger().error("Invalid headers in CSV file.")
        except FileNotFoundError:
            self.get_logger().error(f"Voyage Definition CSV file not found at {file_path}")
        except (ValueError, IndexError):
            self.get_logger().error("Error parsing Voyage Definition CSV data.")


    def solve_ompl(self):
        is_ship_state_changed = (self.ship_current_state.position.latitude != 0.0 or
                                self.ship_current_state.position.longitude != 0.0 or
                                self.ship_current_state.position.depth != 0.0 or
                                self.ship_current_state.angle.heading != 0.0 or 
                                self.ship_current_state.speed != 0.0)

        is_mammal_state_changed = (self.mammal_current_state.position.latitude != 0.0 or
                                self.mammal_current_state.position.longitude != 0.0 or
                                self.mammal_current_state.position.depth != 0.0 or
                                self.mammal_current_state.angle.heading != 0.0 or  
                                self.mammal_current_state.speed != 0.0)  

        
        if is_ship_state_changed and is_mammal_state_changed:
            self.temp+=1
            self.ompl_path_state = OMPLPathState(
                ship_current_state=self.ship_current_state,
                mammal_current_state=self.mammal_current_state,
                ship_start_state=self.ship_start_state,
                ship_end_state=self.ship_end_state, bathy_points = self.bathy_points,count = self.temp
            )
            ompl_path = OMPLPath(max_runtime=1.0, ompl_state=self.ompl_path_state)
            if ompl_path.solved:
                self.get_logger().info("Solved")
                self.ship_optimized_waypoints._waypoints = ompl_path.get_waypoints()
                self.estimate_optimized_state()
            else:
                self.get_logger().info("Not solved within the given runtime")
        else:
            self.get_logger().info("Ship and mammal states have not changed from initialization; skipping OMPLPath execution.")

    
    def estimate_optimized_state(self):
        if len(self.ship_optimized_waypoints.waypoints) > 0 and self.ship_current_state is not None:
            # Assuming the first waypoint is the next target
            next_waypoint = self.ship_optimized_waypoints.waypoints[1]
            
            # Calculate the heading to the next waypoint
            new_heading, _, distance_m = GEODESIC.inv(
            self.ship_current_state.position.longitude, self.ship_current_state.position.latitude, next_waypoint.longitude, next_waypoint.latitude)
            new_speed = self.ship_current_state.speed  

            # Update the ship_optimized_state
            self.ship_optimized_state = self.generate_state(
                self.ship_current_state.position.latitude,
                self.ship_current_state.position.longitude,
                self.ship_current_state.position.depth,
                new_heading,
                new_speed
            )

            self.get_logger().info(f"Optimized State: Heading {new_heading}, Speed {new_speed}")
             
        else:
            self.get_logger().warning("No waypoints or current state unavailable to estimate optimized state.")


    def get_bathy_service_response(self):
        """Method to obtain bathymetry data from the bathy service."""
        request = BathyService.Request()
        # Set the bathymetry box edges
        request.bathy_box_edge1.latitude = self.bathy_min_lat
        request.bathy_box_edge1.longitude = self.bathy_min_lon

        request.bathy_box_edge2.latitude = self.bathy_max_lat
        request.bathy_box_edge2.longitude = self.bathy_max_lon

        # Call the service asynchronously
        future = self.bathy_client.call_async(request)

        while rclpy.ok():
            rclpy.spin_once(self)
            if future.done():
                if future.result() is not None:
                    self.bathy_points = future.result().bathy_points  # Assuming the service responds with a list of HelperPosition
                    self.get_logger().info(f"Received Bathy Service Response. {len(self.bathy_points)} points.")
                else:
                    self.get_logger().error('BathyService call failed %r' % (future.exception(),))
                break

    def get_shoreline_service_response(self):
        """Method to obtain shoreline data from the shoreline service and convert to polygons."""
        request = ShorelineService.Request()
        request.min_box_edge.latitude = self.bathy_min_lat
        request.min_box_edge.longitude = self.bathy_min_lon
        request.max_box_edge.latitude = self.bathy_max_lat 
        request.max_box_edge.longitude = self.bathy_max_lon  

        future = self.shoreline_client.call_async(request)

        while rclpy.ok():
            rclpy.spin_once(self)
            if future.done():
                if future.result() is not None:
                    result = future.result()
                    polygons = []
                    for start_index in result.polygon_starts: # Polygon_starts are the starting indices of each polygon in the lat/lon list
                        end_index = result.polygon_starts.index(start_index) + 1
                        end_index = result.polygon_starts[end_index] if end_index < len(result.polygon_starts) else len(result.latitudes)

                        polygon_points = [(result.longitudes[i], result.latitudes[i]) for i in range(start_index, end_index)]

                        if len(polygon_points) > 2:
                            polygons.append(Polygon(polygon_points))

                    # Convert the list of polygons into a GeoSeries
                    self.shoreline_polygons = gpd.GeoSeries(polygons)

                    self.get_logger().info(f"Received Shoreline Service Response. Converted to {len(self.shoreline_polygons)} polygons.")
                else:
                    self.get_logger().error('ShorelineService call failed %r' % (future.exception(),))
                break

def main(args=None):
    rclpy.init(args=args)

    navig_obj = NavigateSystem()

    rclpy.spin(navig_obj)

    navig_obj.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
