import geopandas as gpd
import matplotlib.pyplot as plt

def plot_shoreline(file_path):
    # Read the shapefile
    gdf = gpd.read_file(file_path)
    
    # Set up the plot
    fig, ax = plt.subplots(figsize=(10, 10))
    ax.set_title('North America Shorelines')
    ax.set_xlabel('Longitude')
    ax.set_ylabel('Latitude')
    
    # Iterate through each polygon in the GeoDataFrame
    for _, row in gdf.iterrows():
        # Each row is a GeoSeries with geometry and other attributes
        # The geometry is typically a Polygon or MultiPolygon
        if row.geometry:
            # Extracting the x and y coordinates from the geometry
            if row.geometry.type == 'Polygon':
                x, y = row.geometry.exterior.xy
                ax.plot(x, y)
            elif row.geometry.type == 'MultiPolygon':
                for poly in row.geometry:
                    x, y = poly.exterior.xy
                    ax.plot(x, y)
    
    # Save the figure
    plt.savefig('shoreline.png', bbox_inches='tight')
    plt.show()

def main():
    # Path to the binary file containing shoreline data
    file_path = '/workspaces/MELO_workspace/src/environment/shoreline_data/north_america_shorlines.shp'
    # Plot shoreline data
    plot_shoreline(file_path)

if __name__ == "__main__":
    main()
