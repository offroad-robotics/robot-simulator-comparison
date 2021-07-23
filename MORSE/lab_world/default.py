from morse.builder import *
from lab_world.builder.robots import Husky

husky = Husky()
husky.translate(z=1.0)

env = Environment('/home/andrew/Documents/MORSE/lab_world/lab_world.blend')
env.set_camera_location([5, -5, 6])
env.set_camera_rotation([1.0470, 0, 0.7854])
