from random import randrange
from PIL import Image, ImageDraw
from Simulator.vehicle import vehicle
def InitMap(size):

    scaling = 2
    im = Image.new('RGB', (size[0], size[1]), (50, 170, 50))
    draw = ImageDraw.Draw(im)

    spawn_center = (round(size[0] / 2), 350)
    spawn_length = 480 # ft
    spawn_width = 300 # ft
    spawn_xy = (spawn_center[0] - (spawn_length / 2)*scaling,
                spawn_center[1] - (spawn_width / 2)*scaling,
                spawn_center[0] + (spawn_length / 2)*scaling,
                spawn_center[1] + (spawn_width / 2)*scaling)

    draw.rectangle(spawn_xy, fill=(25, 125, 25), outline=(0, 0, 0))

    runway_center = (round(size[0]/2), spawn_center[1]+300*scaling)
    runway_length = 480 # ft
    runway_width = 40
    runway_xy = (runway_center[0]-(runway_length/2)*scaling,
                 runway_center[1]-(runway_width/2)*scaling,
                 runway_center[0]+(runway_length/2)*scaling,
                 runway_center[1]+(runway_width/2)*scaling)

    draw.rectangle(runway_xy, fill=(180, 180, 180), outline=(0, 0, 0))

    target_radius = 15
    target_center = (randrange(runway_xy[0], runway_xy[2]), randrange(spawn_center[1]-spawn_width/2-round(target_radius/2),
                                                                     spawn_center[1]+spawn_width/2-round(target_radius/2)))
    target_xy = (target_center[0] - (target_radius)*scaling,
                 target_center[1] - (target_radius)*scaling,
                 target_center[0] + (target_radius)*scaling,
                 target_center[1] + (target_radius)*scaling)

    draw.ellipse(target_xy, fill=(255, 0, 0), outline=(255, 0, 0))
    im.show()

N_flights = 1
N_flybys = 5
for i in range(N_flights):
     plane = vehicle()
     mapSize = (1920, 1080)
     map = InitMap(mapSize)
     for i in range(N_flybys):
         pass