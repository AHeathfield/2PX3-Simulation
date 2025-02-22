from trafficSimulator import *
import numpy as np

# Unfortunately everytime you add something the numbers shift by 1
class Intersection:
    def __init__(self):
        self.sim = Simulation()
        lane_space = 3.5 # This is the size of gap between lanes
        # lane_space = 8
        intersection_size = 49
        island_width = 2
        length = 43.75 #I have shortened the length of the entrance roads because vehicles base speeds are much lower because they are driving in a round about, however they would be able to drive faster in the entrance.
        radius = 18


        # Hardcoded Im not sure where to get actual value
        lane_width = 3.6

        self.v = 8.5


        #ENTRANCES 0-3 (ALWAYS RIGHT LANE) =========================
        # BOTTOM MIDDLE
        self.sim.create_segment((lane_space/2 + island_width/2, length + intersection_size/2), (lane_space/2 + island_width/2, intersection_size/2)) 
        # BOTTOM MIDDLE 2
        self.sim.create_segment((lane_space/2 + island_width/2 + lane_width, length + intersection_size/2), (lane_space/2 + island_width/2 + lane_width, intersection_size/2 + lane_width))  

        # RIGHT MIDDLE
        self.sim.create_segment((length + intersection_size/2, -lane_space/2 - island_width/2), (intersection_size/2, -lane_space/2 - island_width/2)) 

        # TOP MIDDLE
        self.sim.create_segment((-lane_space/2 - island_width/2, -length - intersection_size/2), (-lane_space/2 - island_width/2, - intersection_size/2)) 

        # LEFT MIDDLE
        self.sim.create_segment((-length - intersection_size/2, lane_space/2 + island_width/2), (-intersection_size/2, lane_space/2 + island_width/2)) 



        #EXITS 4-7 (LEFT LANE) =========================
        # BOTTOM MIDDLE
        self.sim.create_segment((-lane_space/2 - island_width/2, intersection_size/2), (-lane_space/2 - island_width/2, length + intersection_size/2))

        # RIGHT MIDDLE
        self.sim.create_segment((intersection_size/2, lane_space/2 + island_width/2), (length+intersection_size/2, lane_space/2 + island_width/2))

        # TOP MIDDLE
        self.sim.create_segment((lane_space/2 + island_width/2, -intersection_size/2), (lane_space/2 + island_width/2, -length - intersection_size/2))

        # LEFT MIDDLE
        self.sim.create_segment((-intersection_size/2, -lane_space/2 - island_width/2), (-length-intersection_size/2, -lane_space/2 - island_width/2))


        #CORNERS 8-11 (these are the corner of the roundabout 4 sections) ========================= 
        # BOTTOM RIGHT
        self.sim.create_quadratic_bezier_curve((lane_space + island_width/2, radius),(radius,radius),(radius,lane_space + island_width/2))
        self.sim.create_quadratic_bezier_curve((lane_space + island_width/2 + lane_width, radius + lane_width),(radius + 2, radius + 2),(radius + lane_width,lane_space + island_width/2 + lane_width))
        
        # TOP RIGHT
        self.sim.create_quadratic_bezier_curve((radius,-lane_space - island_width/2),(radius,-radius),(lane_space + island_width/2,-radius))
        
        # TOP LEFT
        self.sim.create_quadratic_bezier_curve((-lane_space - island_width/2,-radius),(-radius,-radius),(-radius,-lane_space - island_width/2))
        
        #BOTTOM LEFT
        self.sim.create_quadratic_bezier_curve((-radius,lane_space + island_width/2),(-radius,radius),(-lane_space - island_width/2, radius))


        #CONNECTORS 12-15 (the corners are basically a yield right, the connector makes it into a roundabout (circle))=========================
        #RIGHT MIDDLE
        self.sim.create_segment((radius,lane_space + island_width/2),(radius,-lane_space - island_width/2))
        
        #TOP MIDDLE
        self.sim.create_segment((lane_space + island_width/2,-radius),(-lane_space - island_width/2,-radius))
        
        #LEFT MIDDLE
        self.sim.create_segment((-radius,-lane_space - island_width/2),(-radius,lane_space + island_width/2))
        
        #BOTTOM MIDDLE
        self.sim.create_segment((-lane_space - island_width/2, radius),(lane_space + island_width/2, radius))



        #TURN INTO CORNERS 16-19 (these are for the when the right lanes are near the roundabout, it curves into it, giving the path the car will take) =========================
        self.sim.create_quadratic_bezier_curve((lane_space/2 + island_width/2, intersection_size/2),(lane_space/2 + island_width/2, radius),(lane_space + island_width/2, radius))
        self.sim.create_quadratic_bezier_curve((lane_space/2 + island_width/2 + lane_width, intersection_size/2 + lane_width),(lane_space/2 + island_width/2 + lane_width, radius + lane_width),(lane_space + island_width/2 + lane_width, radius + lane_width)) #

        self.sim.create_quadratic_bezier_curve((intersection_size/2, -lane_space/2 - island_width/2),(radius, -lane_space/2 - island_width/2),(radius,-lane_space - island_width/2))

        self.sim.create_quadratic_bezier_curve((-lane_space/2 - island_width/2, - intersection_size/2),(-lane_space/2 - island_width/2, -radius),(-lane_space - island_width/2,-radius))
        # self.sim.create_quadratic_bezier_curve((-lane_space/2 - island_width/2 - lane_width, - intersection_size/2),(-lane_space/2 - island_width/2 - lane_width, -radius),(-lane_space - island_width/2,-radius))


        self.sim.create_quadratic_bezier_curve((-intersection_size/2, lane_space/2 + island_width/2),(-radius,lane_space/2 + island_width/2),(-radius,lane_space + island_width/2))



        #TURN INTO EXITS 20-23 (same as turn into corners but for exit) =========================
        self.sim.create_quadratic_bezier_curve((radius,lane_space + island_width/2),(radius,lane_space/2 + island_width/2),(intersection_size/2, lane_space/2 + island_width/2))
        self.sim.create_quadratic_bezier_curve((lane_space + island_width/2,-radius),(lane_space/2 + island_width/2,-radius),(lane_space/2 + island_width/2, -intersection_size/2))
        self.sim.create_quadratic_bezier_curve((-radius,-lane_space - island_width/2),(-radius,-lane_space/2 - island_width/2),(-intersection_size/2, -lane_space/2 - island_width/2))
        self.sim.create_quadratic_bezier_curve((-lane_space - island_width/2, radius),(-lane_space/2 - island_width/2,radius),(-lane_space/2 - island_width/2, intersection_size/2))
    
        # Generates the vehicles
        self.vg = VehicleGenerator({


            'vehicles': [
                (1, {'path': [0, 16, 8,20,5],'v_max':self.v, 'colour': (0, 225, 0)}),
                (1, {'path': [0, 16, 8,12,9,21,6],'v_max':self.v, 'colour': (0, 225, 0)}),
                (1, {'path': [1, 17, 8,12,9,13,10,22,7],'v_max':self.v, 'colour': (0, 225, 0)}), #
                (1, {'path': [1, 17, 8,12,9,13,10,14,11,23,4],'v_max':self.v, 'colour': (0, 225, 0)}), #

                (1,{'path': [1, 17, 9, 21, 6],'v_max':self.v}),
                (1, {'path': [1,17,9,13,10,22,7],'v_max':self.v}),
                (1, {'path': [1, 17, 9,13,10,14,11,23,4],'v_max':self.v}),
                (1, {'path': [1, 17, 9,13,10,14,11,15,8,20,5],'v_max':self.v}),

                (1, {'path': [2, 18, 10, 22, 7],'v_max':self.v}),
                (1, {'path': [2,18,10,14,11,23,4],'v_max':self.v}),
                (1, {'path': [2,18,10,14,11,15,8,20,5],'v_max':self.v}),
                (1, {'path': [2, 18, 10,14,11,15,8,12,9,21,6],'v_max':self.v}),
                
                (1, {'path': [3, 19, 11, 23, 4],'v_max':self.v}),
                (1, {'path': [3,19,11,15,8,20,5],'v_max':self.v}),
                (1, {'path': [3,19,11,15,8,12,9,21,6],'v_max':self.v}),
                (1, {'path': [3, 19, 11,15,8,12,9,13,10,22,7],'v_max':self.v}),
            ], 'vehicle_rate': 30
        }
        
        )
        # Entrance -> turning into corners, connector -> roundabout corner
        self.sim.define_interfearing_paths([0,16],[15,8],turn=True)
        self.sim.define_interfearing_paths([1,17],[12,9],turn=True)
        self.sim.define_interfearing_paths([2,18],[13,10],turn=True)
        self.sim.define_interfearing_paths([3,19],[14,11],turn=True)
        self.sim.add_vehicle_generator(self.vg)
    
    def get_sim(self):
        return self.sim