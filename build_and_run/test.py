from trafficSimulator import *
import numpy as np


class Intersection:
    def __init__(self):
        self.sim = Simulation()
        lane_space = 3.5
        num_lanes = 2  # Adding two lanes
        intersection_size = 49
        island_width = 4  # Increased to accommodate two lanes
        length = 43.75
        radius = 22  # Increased radius for two lanes

        self.v = 8.5

        # Creating two lanes for each entrance and exit
        for i in range(num_lanes):
            offset = i * lane_space
            # Entrance lanes
            self.sim.create_segment((offset + lane_space/2 + island_width/2, length + intersection_size/2), (offset + lane_space/2 + island_width/2, intersection_size/2))
            self.sim.create_segment((length + intersection_size/2, -offset - lane_space/2 - island_width/2), (intersection_size/2, -offset - lane_space/2 - island_width/2))
            self.sim.create_segment((-offset - lane_space/2 - island_width/2, -length - intersection_size/2), (-offset - lane_space/2 - island_width/2, - intersection_size/2))
            self.sim.create_segment((-length - intersection_size/2, offset + lane_space/2 + island_width/2), (-intersection_size/2, offset + lane_space/2 + island_width/2))
            # Exit lanes
            self.sim.create_segment((-offset - lane_space/2 - island_width/2, intersection_size/2), (-offset - lane_space/2 - island_width/2, length + intersection_size/2))
            self.sim.create_segment((intersection_size/2, offset + lane_space/2 + island_width/2), (length + intersection_size/2, offset + lane_space/2 + island_width/2))
            self.sim.create_segment((offset + lane_space/2 + island_width/2, -intersection_size/2), (offset + lane_space/2 + island_width/2, -length - intersection_size/2))
            self.sim.create_segment((-intersection_size/2, -offset - lane_space/2 - island_width/2), (-length - intersection_size/2, -offset - lane_space/2 - island_width/2))
        
         #corners 8-11
        self.sim.create_quadratic_bezier_curve((lane_space + island_width/2, radius),(radius,radius),(radius,lane_space + island_width/2))
        self.sim.create_quadratic_bezier_curve((radius,-lane_space - island_width/2),(radius,-radius),(lane_space + island_width/2,-radius))
        self.sim.create_quadratic_bezier_curve((-lane_space - island_width/2,-radius),(-radius,-radius),(-radius,-lane_space - island_width/2))
        self.sim.create_quadratic_bezier_curve((-radius,lane_space + island_width/2),(-radius,radius),(-lane_space - island_width/2, radius))
        #connectors 12-15
        self.sim.create_segment((radius,lane_space + island_width/2),(radius,-lane_space - island_width/2))
        self.sim.create_segment((lane_space + island_width/2,-radius),(-lane_space - island_width/2,-radius))
        self.sim.create_segment((-radius,-lane_space - island_width/2),(-radius,lane_space + island_width/2))
        self.sim.create_segment((-lane_space - island_width/2, radius),(lane_space + island_width/2, radius))
        #turn into corners 16-19
        self.sim.create_quadratic_bezier_curve((lane_space/2 + island_width/2, intersection_size/2),(lane_space/2 + island_width/2, radius),(lane_space + island_width/2, radius))
        self.sim.create_quadratic_bezier_curve((intersection_size/2, -lane_space/2 - island_width/2),(radius, -lane_space/2 - island_width/2),(radius,-lane_space - island_width/2))
        self.sim.create_quadratic_bezier_curve((-lane_space/2 - island_width/2, - intersection_size/2),(-lane_space/2 - island_width/2, -radius),(-lane_space - island_width/2,-radius))
        self.sim.create_quadratic_bezier_curve((-intersection_size/2, lane_space/2 + island_width/2),(-radius,lane_space/2 + island_width/2),(-radius,lane_space + island_width/2))
        #turn to exit 20-23
        self.sim.create_quadratic_bezier_curve((radius,lane_space + island_width/2),(radius,lane_space/2 + island_width/2),(intersection_size/2, lane_space/2 + island_width/2))
        self.sim.create_quadratic_bezier_curve((lane_space + island_width/2,-radius),(lane_space/2 + island_width/2,-radius),(lane_space/2 + island_width/2, -intersection_size/2))
        self.sim.create_quadratic_bezier_curve((-radius,-lane_space - island_width/2),(-radius,-lane_space/2 - island_width/2),(-intersection_size/2, -lane_space/2 - island_width/2))
        self.sim.create_quadratic_bezier_curve((-lane_space - island_width/2, radius),(-lane_space/2 - island_width/2,radius),(-lane_space/2 - island_width/2, intersection_size/2))
    

        # Roundabout lanes
        # for i in range(num_lanes):
        #     offset = i * lane_space
        #     self.sim.create_quadratic_bezier_curve((lane_space + island_width/2 + offset, radius), (radius, radius), (radius, lane_space + island_width/2 + offset))
        #     self.sim.create_quadratic_bezier_curve((radius, -lane_space - island_width/2 - offset), (radius, -radius), (lane_space + island_width/2 + offset, -radius))
        #     self.sim.create_quadratic_bezier_curve((-lane_space - island_width/2 - offset, -radius), (-radius, -radius), (-radius, -lane_space - island_width/2 - offset))
        #     self.sim.create_quadratic_bezier_curve((-radius, lane_space + island_width/2 + offset), (-radius, radius), (-lane_space - island_width/2 - offset, radius))
            
        # Adjust vehicle paths to accommodate two lanes (might require further refinement)
        self.vg = VehicleGenerator({
            'vehicles': [
                (1, {'path': [0, 16, 8, 20, 5], 'v_max': self.v}),
                (1, {'path': [1, 17, 9, 21, 6], 'v_max': self.v}),
                (1, {'path': [2, 18, 10, 22, 7], 'v_max': self.v}),
                (1, {'path': [3, 19, 11, 23, 4], 'v_max': self.v}),
            ], 'vehicle_rate': 30
        })

        self.sim.define_interfearing_paths([0, 16], [15, 8], turn=True)
        self.sim.define_interfearing_paths([1, 17], [12, 9], turn=True)
        self.sim.define_interfearing_paths([2, 18], [13, 10], turn=True)
        self.sim.define_interfearing_paths([3, 19], [14, 11], turn=True)
        self.sim.add_vehicle_generator(self.vg)

    def get_sim(self):
        return self.sim
