# -*- coding: utf-8 -*-
"""
Created on Wed Jan 18 10:53:30 2023

@author: Catherine
"""
import numpy as np
from numpy import pi,sin,cos,arccos,arcsin,sqrt
import math
from matplotlib import pyplot as plt

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

def distCoords(cd1,cd2,altitude):
    '''
    :param cd1: coords1, degrees [lat,long]
    :param cd2:
    :return:
    '''
    
    earthRadii = radius((cd1[0]+cd2[0])/2) #Km #EARTH RADII VARIES BASED ON LATITUDE
    #altitude = 0.233
    
    
    lat1 = cd1[0] / 180 * pi
    long1 = cd1[1] / 180 * pi

    lat2 = cd2[0] / 180 * pi
    long2 = cd2[1] / 180 * pi

    d1 = earthRadii * arccos((sin(lat1) * sin(lat2)) + cos(lat1) * cos(lat2) * cos(long2-long1))
    #print(d1*1000)
    d2 = (earthRadii+altitude) * arccos((sin(lat1) * sin(lat2)) + cos(lat1) * cos(lat2) * cos(long2 - long1))
    #print(d2*1000)

    # Haversine formula
    dlon = long2 - long1
    dlat = lat2 - lat1
    a = sin(dlat / 2) ** 2 + cos(lat1) * cos(lat2) * sin(dlon / 2) ** 2

    c = 2 * arcsin(sqrt(a))


    # calculate the result
    return c * earthRadii*1000


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

    #print(message)
    # N dlat ; E dlon -> ex: (N 10deg ; E 20deg)
    
    
    
    
    


def get_bearing(lat1, long1, lat2, long2):
    #fonction qui retourne le bearing entre 2 ping gps
    dLon = (long2 - long1)
    x = math.cos(math.radians(lat2)) * math.sin(math.radians(dLon))
    y = math.cos(math.radians(lat1)) * math.sin(math.radians(lat2)) - math.sin(math.radians(lat1)) * math.cos(math.radians(lat2)) * math.cos(math.radians(dLon))
    brng = np.arctan2(x,y)
    brng = np.degrees(brng)

    return brng



def get_xy(x,alt):
    #Fonction qui prend en entrée un np.array contenant des données gps tel que montré ci-dessous
    # x=np.array([[lat1,lon1],[lat2,lon2]....,[latn,lonn]])
    #La fontion retourne deux array x,y représentant dans un plan cartésien les distances en m entre chaque ping gps
    coords=x
 
    #coords=np.array([[0,0], [1.9740288191360256e-05,0.0001322460891825204]])
    dists=np.empty(0)
    ori=np.empty(0)
    for i in range(0,len(coords)-1):
        if i!=len(coords):
            
            cord1=coords[-2]
            #print(cord1)
            cord2=coords[-1]
            #print(cord2)
            dist=distCoords(cord1,cord2,alt)
            dists=np.append(dists,dist)
            brng=get_bearing(cord1[0],cord1[1],cord2[0],cord2[1])
            ori=np.append(ori,brng)

    cmb=len(ori)
    angle=np.full(cmb,90)
    angle=np.subtract(angle,ori)
    x=np.array([[0]])
    y=np.array([[0]])
    varx=0
    vary=0
    for i in range(len(dists)):
        varx+=dists[i]*math.cos(math.radians(angle[i]))
        vary+=dists[i]*math.sin(math.radians(angle[i]))
        x=np.append(x,varx)
        y=np.append(y,vary)
    return x,y
    
    


#d#istance=distCoords(cd1,cd2)
#orient=orientation(cd1,cd2)


coords=np.array([[],[],[]])
x,y=get_xy(coords,0.3)
print(x,y)

plt.plot(x,y)

plt.ticklabel_format(style='plain')

#brng=get_bearing(cd1[0],cd1[1],cd2[0],cd2[1])
#angle=90-brng
#angler=math.radians(angle)
#x=distance*np.cos(angler)
#y=distance*np.sin(angler)
#plt.ticklabel_format(style='plain')
#plt.plot(x,y,marker='o')



