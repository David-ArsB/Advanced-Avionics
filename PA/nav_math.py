from numpy import pi,sin,cos,arccos,arcsin,sqrt
import math

def radius (lat):
    lat = math.radians(lat) #converting into radians
    a = 6378.137  #Radius at sea level at equator
    b = 6356.752  #Radius at poles
    c = (a**2*math.cos(lat))**2
    d = (b**2*math.sin(lat))**2
    e = (a*math.cos(lat))**2
    f = (b*math.sin(lat))**2
    R = math.sqrt((c+d)/(e+f))
    return R # @sea level

def distCoords(cd1,cd2):
    '''
    Find the distance (in meters) between two sets of coordinates on the surface of the Earth.
    :param cd1: coords1, degrees [lat,long]
    :param cd2: coords1, degrees [lat,long]
    :return:
    '''

    lat1 = cd1[0] / 180 * pi
    long1 = cd1[1] / 180 * pi

    lat2 = cd2[0] / 180 * pi
    long2 = cd2[1] / 180 * pi

    # Haversine formula
    dlon = long2 - long1
    dlat = lat2 - lat1
    a = sin(dlat / 2) ** 2 + cos(lat1) * cos(lat2) * sin(dlon / 2) ** 2

    c = 2 * arcsin(sqrt(a))
    earthRadii = radius((lat1+lat2)/2)

    # calculate the result
    return c * earthRadii*1000

def distCoords2(cd1,cd2):
    '''
    Find the distance (in meters) between two sets of coordinates on the surface of the Earth.
    Returns a tuple containing the northbound component and eastbound component of the distance.

    :param cd1: coords1, degrees [lat,long]
    :param cd2: coords1, degrees [lat,long]
    :return:
    '''

    lat1 = cd1[0]
    long1 = cd1[1]

    lat2 = cd2[0]
    long2 = cd2[1]

    pt0 = [lat1, long1]
    pt1 = [lat2, long1]
    pt2 = [lat1, long2]

    north_dist = distCoords(pt0, pt1)
    if (lat2-lat1) < 0:
        north_dist *= -1

    east_dist = distCoords(pt0, pt2)
    if (long2-long1) < 0:
        east_dist *= -1


    # calculate the result
    return [north_dist, east_dist]

def orientation(cd1, cd2 ):
    '''

    :param cd1:
    :param cd2:
    :param altitude:
    :return:
    '''

    lat1 = cd1[0]
    long1 = cd1[1]
    lat2 = cd2[0]
    long2 = cd2[1]

    dlon = long2 - long1
    dlat = lat2 - lat1

    deg = math.atan2(dlon,dlat)*180/pi
    if deg < 0:
        deg+=360
    print(deg)
    return deg


    # North is +, East is +
    # degrees_temp = math.atan2(dlon, dlat) / math.pi * 180 # Degrees measured from equator (East) to North
    if dlat >= 0:
        message = '[ N '+str(abs(dlat))+' ; '
    elif dlat < 0:
        message = '[ S '+str(abs(dlat))+' ; '

    if dlon >= 0:
        message += 'E '+str(abs(dlon))+' ] '
    elif dlon < 0:
        message += 'W '+str(abs(dlon))+' ] '

    print(message)
    # N dlat ; E dlon -> ex: (N 10deg ; E 20deg)

if __name__ == "__main__":
    cd1 = [45.51741984466869, -73.78359957335857]
    cd2 = [45.5177399722043, -73.78382801187195]
    earthRadii = radius((cd1[0]+cd2[0])/2) #Km #EARTH RADII VARIES BASED ON LATITUDE
    altitude = 0.233
    print(distCoords(cd1,cd2))
    print(distCoords2(cd1,cd2))
    orientation(cd1,cd2)

