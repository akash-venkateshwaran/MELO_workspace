import rclpy
from rclpy.node import Node
from custom_interfaces.srv import BathyService
from custom_interfaces.msg import HelperPosition
from environment.py_bathy import Bathymetry
import os

class Envi(Node):
    def __init__(self):
        super().__init__('envi_node')
        self.setup_logger()
        self.bathy_data_loc = os.path.join(os.getcwd(), 'src', 'environment', 'bathy_data')
        self.bathy_obj = Bathymetry(self.bathy_data_loc)

        self.service = self.create_service(BathyService, 'bathy_service', self.handle_bathy_request)

    def setup_logger(self):
        self.get_logger().info("Envi Node is started!")

    def handle_bathy_request(self, request, response):
        response.bathy_points = self.bathy_obj.get_bathy(request)
        self.get_logger().info(f"Responded to Bathy Service Request. {len(response.bathy_points)} points. \n\n")
        return response

def main(args=None):
    rclpy.init(args=args)
    envi_obj = Envi()
    rclpy.spin(envi_obj)
    envi_obj.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
