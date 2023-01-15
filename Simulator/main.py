from random import randrange
from PIL import Image, ImageDraw
from Simulator.vehicle import vehicle
def InitMap(size):


    im = Image.new('RGB', (size[0], size[1]), (50, 170, 50))
    draw = ImageDraw.Draw(im)

    runway_center = (round(size[0]/2), 900)
    runway_length = 600
    runway_width = 40
    runway_xy = (runway_center[0]-runway_length/2,
                 runway_center[1]-runway_width/2,
                 runway_center[0]+runway_length/2,
                 runway_center[1]+runway_width/2)

    draw.rectangle(runway_xy, fill=(180, 180, 180), outline=(0, 0, 0))

    target_center = (randrange(runway_xy[0],runway_xy[2]), randrange(100,500))
    target_radius = 5
    target_xy = (target_center[0] - target_radius,
                 target_center[1] - target_radius,
                 target_center[0] + target_radius,
                 target_center[1] + target_radius)

    draw.ellipse(target_xy, fill=(255, 0, 0), outline=(255, 0, 0))
    im.show()

N_flights = 100
N_flybys = 5
mapSize = (1920, 1080)
map = InitMap(mapSize)
# for i in range(N_flights):
#     vehicle = vehicle()
#     mapSize = (1920, 1080)
#     map = InitMap(mapSize)
#     for i in range(N_flybys):
#         pass