import rclpy
from rclpy.node import Node
from custom_interfaces.msg import State, HelperPosition, HeadingAngle
from pyproj import Geod

GEODESIC = Geod(ellps="WGS84")


class Ship(Node):
    def __init__(self):
        super().__init__('ship_node')
        self.setup_logger()
        self.subscription_ship_start_state = self.create_subscription(
            State,
            'ship_start_state',
            self.ship_start_state_callback,
            10)

        self.subscription_ship_optimized_state = self.create_subscription(
            State,
            'ship_optimized_state',
            self.ship_optimized_state_callback,
            10)

        self.publisher = self.create_publisher(
            State,
            'ship_state',
            10)
            
        self.start_state = None
        self.current_state = None
        self.timer_period = 1.0  # seconds
        self.timer = self.create_timer(self.timer_period, self.timer_callback)

    def setup_logger(self):
        self.get_logger().info("Ship Node is started!")

    def ship_start_state_callback(self, msg):
        if self.start_state is None:
            self.get_logger().info(
                f'Received Ship Start State: [{msg.position.latitude}, {msg.position.longitude}, {msg.position.depth}], '
                f'Angle: {msg.angle.heading}, Speed: {msg.speed} m/s'
            )
        self.start_state = msg

    def ship_optimized_state_callback(self, msg):
        self.get_logger().info(
            f'Received Ship Optimized State: Speed: {msg.speed}, Heading: {msg.angle.heading}'
        )
        if self.current_state is not None:
            # Update current state with optimized speed and heading
            self.current_state.speed = msg.speed
            self.current_state.angle.heading = msg.angle.heading

    def update_state(self):
        if self.current_state is not None:
            distance = self.current_state.speed * self.timer_period
            endlon, endlat, _ = GEODESIC.fwd(
                self.current_state.position.longitude,
                self.current_state.position.latitude,
                self.current_state.angle.heading,
                distance
            )

            # Update current state position based on current speed and heading
            self.current_state.position.latitude = endlat
            self.current_state.position.longitude = endlon

            return self.current_state
        else:
            return None

    def timer_callback(self):
        if self.start_state is None:
            # Waiting for the start state
            self.get_logger().info("Waiting for the start state...")
        elif self.current_state is None:
            # Start state received, update current state and begin continuous updates
            self.current_state = self.start_state
            self.get_logger().info("Start state received. Beginning continuous updates.")
        else:
            # Continuous updates
            new_state = self.update_state()

            if new_state is not None:
                self.publisher.publish(new_state)

                state_msg = (
                    f"Publishing Ship State: [{new_state.position.latitude}, "
                    f"{new_state.position.longitude}, {new_state.position.depth}], "
                    f"Angle: {new_state.angle.heading}, Speed: {new_state.speed} m/s"
                )
                self.get_logger().info(state_msg)

def main(args=None):
    rclpy.init(args=args)
    ship_obj = Ship()
    rclpy.spin(ship_obj)
    ship_obj.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
