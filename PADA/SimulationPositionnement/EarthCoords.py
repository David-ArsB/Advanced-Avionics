from numpy import pi,sin,cos,arccos,arcsin,sqrt
import math

def radius (lat):
    lat=math.radians(lat) #converting into radians
    a = 6378.137  #Radius at sea level at equator
    b = 6356.752  #Radius at poles
    c = (a**2*math.cos(lat))**2
    d = (b**2*math.sin(lat))**2
    e = (a*math.cos(lat))**2
    f = (b*math.sin(lat))**2
    R = math.sqrt((c+d)/(e+f))
    return R # @sea level

def distCoords(cd1, cd2, altitude):
    '''

    :param cd1: coords1, degrees [lat,long]
    :param cd2: coords2, degrees [lat,long]
    :return:
    '''
    earthRadii = radius((cd1[0] + cd2[0]) / 2)  # Km EARTH RADII VARIES BASED ON LATITUDE - Average of two coords

    # Convert to rads
    lat1 = cd1[0] / 180 * pi
    long1 = cd1[1] / 180 * pi

    lat2 = cd2[0] / 180 * pi
    long2 = cd2[1] / 180 * pi

    # Formula 1
    d1 = (earthRadii+altitude) * arccos((sin(lat1) * sin(lat2)) + cos(lat1) * cos(lat2) * cos(long2 - long1))

    # Haversine Formula
    dlon = long2 - long1
    dlat = lat2 - lat1
    a = (sin(dlat / 2)) ** 2 + cos(lat1) * cos(lat2) * (sin(dlon / 2)) ** 2

    c = 2 * arcsin(sqrt(a))

    # calculate the result
    dist = c * earthRadii * 1000


def orientation(cd1, cd2, altitude):
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

    # North is +, East is +
    # degrees_temp = math.atan2(dlon, dlat) / math.pi * 180 # Degrees measured from equator (East) to North
    if dlat >= 0:
        message = '[ N ' + str(abs(dlat)) + ' ; '
    elif dlat < 0:
        message = '[ S ' + str(abs(dlat)) + ' ; '

    if dlon >= 0:
        message += 'E ' + str(abs(dlon)) + ' ] '
    elif dlon < 0:
        message += 'W ' + str(abs(dlon)) + ' ] '

    print(message)

    # N dlat ; E dlon -> ex: (N 10deg ; E 20deg)