import rclpy
from pyproj import Geod
from custom_interfaces import HeadingAngle

def main(args=None):
    g = Geod(ellps='clrk66') # Use Clarke 1866 ellipsoid.
    # specify the lat/lons of some cities.
    angle = HeadingAngle()
    angle = 0
    boston_lat = 42.+(15./60.); boston_lon = -71.-(7./60.)
    portland_lat = 45.+(31./60.); portland_lon = -123.-(41./60.)
    newyork_lat = 40.+(47./60.); newyork_lon = -73.-(58./60.)
    london_lat = 51.+(32./60.); london_lon = -(5./60.)

    vac_lat = 49.28093924949435
    vac_lon = -123.13250227152369
    temp_lat = 49.282431454965334
    temp_lon = -122.78537973086199


    # compute forward and back azimuths, plus distance
    # between Boston and Portland.
    az12,az21,dist = g.inv(vac_lon,vac_lat,temp_lon,temp_lat)
    print(f"{az12:.3f} {az21:.3f} {dist:.3f}")

    endlon, endlat, backaz = g.fwd(vac_lon, vac_lat, az12, dist)
    print(f"{endlat:.3f} {endlon:.3f} {backaz:.3f}")

if __name__ == '__main__':
    main()
