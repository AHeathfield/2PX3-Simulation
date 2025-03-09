# from base_intersection import *
# from intersection import *
# from example_intersection import *
# from roundabout import *
# from my_intersection import *
from team_simulation import *
# from initial_mods import *

# Do 2 simulations one with self driving one without

#defining the intersection
intersection = Intersection()
#assigning the simulation
sim = intersection.get_sim()
#defining the window that displays the simulation
win = Window(sim)

#starts running the simulation then displays it
win.run()
win.show()