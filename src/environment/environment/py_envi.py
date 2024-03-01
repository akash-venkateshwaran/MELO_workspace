import rclpy
from rclpy.node import Node
from custom_interfaces.srv import BathyService, ShorelineService
from custom_interfaces.msg import HelperPosition
from environment.py_bathy import Bathymetry
from environment.py_shoreline import Shoreline  # Assuming the Shoreline class is in py_shoreline.py
import os

class Envi(Node):
    def __init__(self):
        super().__init__('envi_node')
        self.setup_logger()
        
        self.bathy_data_loc = os.path.join(os.getcwd(), 'src', 'environment', 'bathy_data')
        self.bathy_obj = Bathymetry(self.bathy_data_loc)
        self.service = self.create_service(BathyService, 'bathy_service', self.handle_bathy_request)

        self.shoreline_data_loc = os.path.join(os.getcwd(), 'src', 'environment', 'shoreline_data')
        self.shoreline_obj = Shoreline(self.shoreline_data_loc)
        self.shoreline_service = self.create_service(ShorelineService, 'shoreline_service', self.handle_shoreline_request)


    def setup_logger(self):
        self.get_logger().info("Envi Node is started!")

    def handle_bathy_request(self, request, response):
        response.bathy_points = self.bathy_obj.get_bathy(request)
        self.get_logger().info(f"Responded to Bathy Service Request. {len(response.bathy_points)} points. \n\n")
        return response

    def handle_shoreline_request(self, request, response):
        # Convert HelperPosition to bounding box edges
        min_lat, min_lon = request.min_box_edge.latitude, request.min_box_edge.longitude
        max_lat, max_lon = request.max_box_edge.latitude, request.max_box_edge.longitude

        polygon_list = self.shoreline_obj.get_polygons(min_lat, max_lat, min_lon, max_lon)

        # Format the polygons for the response
        formatted_data = self.shoreline_obj.format_polygons_for_service(polygon_list)
        response.latitudes, response.longitudes, response.polygon_starts = formatted_data

        self.get_logger().info(f"Responded to Shoreline Service Request with {len(response.polygon_starts)} polygons.")
        return response

def main(args=None):
    rclpy.init(args=args)
    envi_obj = Envi()
    rclpy.spin(envi_obj)
    envi_obj.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
