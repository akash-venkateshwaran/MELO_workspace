import os
import geopandas as gpd
import shapely
from shapely.geometry import box
import matplotlib.pyplot as plt
from custom_interfaces.msg import HelperPosition


class Shoreline:
    """A class used to represent and manipulate shoreline data.

    Attributes:
        data_loc (str): The file location of the shoreline data.
        data_shp (GeoDataFrame): A GeoDataFrame containing the loaded shoreline shapefile data.
    """

    def __init__(self, shoreline_data_loc: str):
        """Initializes the Shoreline class with a specific location for shoreline data.

        Args:
            shoreline_data_loc (str): The file location of the shoreline data.

        Notes:
            The constructor attempts to load the shoreline data from the specified location.
        """
        self.data_loc = shoreline_data_loc
        self.data_shp = self.load_shoreline_data()

    def load_shoreline_data(self):
        """Loads shoreline data from a shapefile.

        Returns:
            GeoDataFrame or None: The loaded shoreline data as a GeoDataFrame, or None if the data could not be loaded.

        Notes:
            The function searches for a '.shp' file in the specified directory.
        """
        if os.path.exists(self.data_loc):
            for file_name in os.listdir(self.data_loc):
                if file_name.endswith('.shp'):
                    file_path = os.path.join(self.data_loc, file_name)
                    shoreline_data = gpd.read_file(file_path)
                    return shoreline_data
            print(f"No shoreline shapefile data found in the directory: {self.data_loc}")
        else:
            print(f"Directory not found: {self.data_loc}")
            return None

    def get_polygons(self, lat_min: float, lat_max: float, lon_min: float, lon_max: float):
        """Retrieves polygons from the shoreline data within specified latitude and longitude bounds.

        Args:
            lat_min (float): The minimum latitude.
            lat_max (float): The maximum latitude.
            lon_min (float): The minimum longitude.
            lon_max (float): The maximum longitude.

        Returns:
            GeoSeries: A GeoSeries of polygons that lie within the specified bounds.

        Notes:
            The function intersects the shoreline data with a bounding box created from the provided bounds using Geopandas.
        """
        if self.data_shp is not None:
            bounding_box = box(lon_min, lat_min, lon_max, lat_max)
            bounding_box_series = gpd.GeoSeries([bounding_box])
            intersected_geometries = self.data_shp.geometry.intersection(bounding_box_series.iloc[0])
            polygon_list = intersected_geometries[intersected_geometries.is_empty == False]

            if polygon_list.empty:
                print('No polygons found within the specified range')
            else:
                return polygon_list
        else:
            print('Shoreline data is empty')

    def plot_polygons(self, polygon_list):
        """Plots the given list of polygons.

        Args:
            polygon_list (GeoSeries): A GeoSeries of polygons to be plotted.

        Notes:
            The function creates a plot of the provided polygons, each with a different color and fills them.
        """
        if polygon_list is not None:
            gdf = gpd.GeoDataFrame(geometry=polygon_list)
            fig, ax = plt.subplots(1, 1, figsize=(10, 10))
            gdf.plot(ax=ax, edgecolor='black', cmap='tab20', linewidth=1, alpha=0.5)
            ax.set_title('Shoreline Polygons')
            ax.set_xlabel('Longitude')
            ax.set_ylabel('Latitude')
            minx, miny, maxx, maxy = gdf.total_bounds
            ax.set_xlim(minx, maxx)
            ax.set_ylim(miny, maxy)
            plt.savefig('polygons.png', bbox_inches='tight')
        else:
            print('No polygons to plot or polygon_list is not defined.')


    def format_polygons_for_service(self, polygon_list):
        """Formats a list of polygons for output via a ROS service: ShorelineService

        Args:
            polygon_list (GeoSeries): A GeoSeries of polygons to be formatted.

        Returns:
            tuple: Three lists representing the latitudes, longitudes, and starting indices of polygons.

        Notes:
            The function formats the provided polygons into separate latitude and longitude lists, along with a list of starting indices for each polygon.
        """
        latitudes = []
        longitudes = []
        polygon_starts = []  # Marks start index of every polygon

        for polygon in polygon_list:
            # Check if the geometry is a MultiPolygon
            if isinstance(polygon, shapely.geometry.MultiPolygon):
                # Iterate through each Polygon in the MultiPolygon
                for single_polygon in polygon.geoms:
                    # Extract the exterior coordinates of the Polygon
                    coords = list(single_polygon.exterior.coords)
                    latitudes.extend([lat for lon, lat in coords]) 
                    longitudes.extend([lon for lon, lat in coords])
                    polygon_starts.append(len(latitudes))
            elif isinstance(polygon, shapely.geometry.Polygon):
                # If the geometry is a single Polygon, handle it directly
                coords = list(polygon.exterior.coords)
                latitudes.extend([lat for lon, lat in coords])
                longitudes.extend([lon for lon, lat in coords]) 
                polygon_starts.append(len(latitudes))

        polygon_starts = [0] + polygon_starts[:-1]


        return latitudes, longitudes, polygon_starts
