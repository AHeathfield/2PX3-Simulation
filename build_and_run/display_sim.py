# from base_intersection import *
# from intersection import *
# from example_intersection import *
# from roundabout import *
# from my_intersection import *
from test2 import *

#defining the intersection
intersection = Intersection()
#assigning the simulation
sim = intersection.get_sim()
#defining the window that displays the simulation
win = Window(sim)

#starts running the simulation then displays it
win.run()
win.show()