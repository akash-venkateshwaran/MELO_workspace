import rclpy
from rclpy.node import Node
from custom_interfaces.msg import State, HelperPosition, HeadingAngle
import random
from pyproj import Geod

GEODESIC = Geod(ellps="WGS84")

class Mammal(Node):
    def __init__(self):
        super().__init__('mammal_node')
        self.setup_logger()
        self.publisher_ = self.create_publisher(State, 'mammal_state', 10)
        self.initial_state = self.generate_random_state()
        self.current_state = self.initial_state
        self.timer_period = 1  # seconds
        self.timer = self.create_timer(self.timer_period, self.timer_callback)

    def setup_logger(self):
        self.get_logger().info("Mammal Node is started!")

    def generate_random_state(self):
        # latitude = random.uniform(46.81041717529297, 49.98125076293945)
        # longitude = random.uniform(-125.19791412353516,-121.56041717529297)
        latitude = 49.00
        longitude = -124.00
        depth = random.uniform(10, 2000)
        heading =120.0
        speed = 2000.0  #must always be in m/s for distance calc

        return State(
            position=HelperPosition(latitude=latitude, longitude=longitude, depth=depth),
            angle=HeadingAngle(heading=heading),
            speed=speed
        )

    def update_state(self):

        distance = self.current_state.speed * self.timer_period

        g = Geod(ellps='clrk66')
        endlon, endlat, _ = GEODESIC.fwd(
            self.current_state.position.longitude,
            self.current_state.position.latitude,
            self.current_state.angle.heading,
            distance
        )

        return State(
            position=HelperPosition(latitude=endlat, longitude=endlon, depth=self.current_state.position.depth),
            angle=HeadingAngle(heading=self.current_state.angle.heading),
            speed=self.current_state.speed
        )

    def timer_callback(self):

        self.current_state = self.update_state()

        self.publisher_.publish(self.current_state)
        
       
        state_msg = (
            f"Publishing Mammal State: [{self.current_state.position.latitude}, "
            f"{self.current_state.position.longitude}, {self.current_state.position.depth}], "
            f"Angle: {self.current_state.angle.heading}, Speed: {self.current_state.speed} m/s"
        )
        self.get_logger().info(state_msg)

def main(args=None):
    rclpy.init(args=args)
    mammal_obj = Mammal()
    rclpy.spin(mammal_obj)
    mammal_obj.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
