import rclpy
from rclpy.node import Node
from custom_interfaces.msg import State, HelperPosition, HeadingAngle
import os
import csv

from path_planning.ompl_path import OMPLPath


class NavigateSystem(Node):
    def __init__(self):
        super().__init__('navigate_system')
        self.setup_logger()
        self.ship_current_state = State()
        # TODO here the path is hard coded
        self.load_voyage_definition(os.path.join(os.getcwd(), 'src', 'path_planning','Voyage_definition.csv'))
        self.ship_optimized_state = self.generate_state(0.0, 0.0, 0.0, 90.0, 150.0) # Note that we won't be using the first three elemets. We are exporting only optimized speed and heading

        # Create publishers for 'ship_start_state', 'ship_end_state', and 'ship_optimized_state'
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

        # Subscribe to the 'ship_state' topic
        self.ship_state_subscription = self.create_subscription(
            State,
            'ship_state',
            self.ship_state_callback,
            10
        )
        self.ship_state_subscription  # prevent unused variable warning

        # Timer to periodically publish ship_start_state, ship_end_state, and ship_optimized_state
        self.publish_timer = self.create_timer(1, self.publish_states)
    
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
        self.get_logger().info(
            f'I heard Ship State: [{msg.position.latitude}, {msg.position.longitude}, {msg.position.depth}], '
            f'Angle: {msg.angle.heading}, Speed: {msg.speed} m/s'
        )

    def publish_states(self):
        # Publish ship_start_state, ship_end_state, and ship_optimized_state periodically
        self.ship_start_state_publisher.publish(self.ship_start_state)
        self.ship_end_state_publisher.publish(self.ship_end_state)
        self.ship_optimized_state_publisher.publish(self.ship_optimized_state)
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


    def get_ship_optimized_state(self):
        ompl_path = OMPLPath(parent_logger= self.get_logger, max_runtime=1.0)
        if ompl_path.solved:
            self.get_logger("Solved")

        return None
    
        


def main(args=None):
    rclpy.init(args=args)

    navig_obj = NavigateSystem()

    rclpy.spin(navig_obj)

    navig_obj.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
