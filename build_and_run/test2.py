from trafficSimulator import *
import numpy as np

class Intersection:
    def __init__(self):
        self.sim = Simulation()
        lane_space = 3.5
        num_lanes = 2
        intersection_size = 49
        island_width = 4
        length = 43.75
        radius = 18 
        self.v = 8.5

        for i in range(num_lanes):
            offset = i * lane_space
            # Should be exits
            self.sim.create_segment((offset + lane_space/2 + island_width/2, length + intersection_size/2), (offset + lane_space/2 + island_width/2, intersection_size/2))
            self.sim.create_segment((length + intersection_size/2, -offset - lane_space/2 - island_width/2), (intersection_size/2, -offset - lane_space/2 - island_width/2))
            self.sim.create_segment((-offset - lane_space/2 - island_width/2, -length - intersection_size/2), (-offset - lane_space/2 - island_width/2, - intersection_size/2))
            self.sim.create_segment((-length - intersection_size/2, offset + lane_space/2 + island_width/2), (-intersection_size/2, offset + lane_space/2 + island_width/2))

            # Should be exits
            self.sim.create_segment((-offset - lane_space/2 - island_width/2, intersection_size/2), (-offset - lane_space/2 - island_width/2, length + intersection_size/2))
            self.sim.create_segment((intersection_size/2, offset + lane_space/2 + island_width/2), (length + intersection_size/2, offset + lane_space/2 + island_width/2))
            self.sim.create_segment((offset + lane_space/2 + island_width/2, -intersection_size/2), (offset + lane_space/2 + island_width/2, -length - intersection_size/2))
            self.sim.create_segment((-intersection_size/2, -offset - lane_space/2 - island_width/2), (-length - intersection_size/2, -offset - lane_space/2 - island_width/2))

            # Corners of Roundabout
            self.sim.create_quadratic_bezier_curve((lane_space + island_width/2, radius - offset), (radius - 0.93*offset, radius - 0.93*offset), (radius - 1*offset, lane_space + island_width/2))
            self.sim.create_quadratic_bezier_curve((radius - offset, -lane_space - island_width/2 + 0*offset), (radius - 0.93*offset, -radius + 0.93*offset), (lane_space + island_width/2 + 0*offset, -radius + offset))
            self.sim.create_quadratic_bezier_curve((-lane_space - island_width/2 - offset, -radius), (-radius, -radius), (-radius, -lane_space - island_width/2 - offset))
            self.sim.create_quadratic_bezier_curve((-radius, lane_space + island_width/2 + offset), (-radius, radius), (-lane_space - island_width/2 - offset, radius))

            # Connectors for roundabout
            self.sim.create_segment((radius - offset, lane_space + island_width/2 + 1*offset), (radius - offset, -lane_space - island_width/2 - 1*offset))
            self.sim.create_segment((lane_space + island_width/2 + offset, -radius + offset), (-lane_space - island_width/2 - offset, -radius + offset))
            self.sim.create_segment((-radius, -lane_space - island_width/2 - offset), (-radius, lane_space + island_width/2 + offset))
            self.sim.create_segment((-lane_space - island_width/2 - offset, radius), (lane_space + island_width/2 + offset, radius))
            
            # Turn into corners
            if (i == 0):
                self.sim.create_quadratic_bezier_curve((lane_space/2 + island_width/2 + offset, intersection_size/2), (lane_space/2 + island_width/2 + offset, radius), (lane_space + island_width/2 + offset, radius - lane_space))
                self.sim.create_quadratic_bezier_curve((intersection_size/2, -lane_space/2 - island_width/2 - offset), (radius, -lane_space/2 - island_width/2 - offset), (radius, -lane_space - island_width/2 - offset))
                self.sim.create_quadratic_bezier_curve((-lane_space/2 - island_width/2 - offset, - intersection_size/2), (-lane_space/2 - island_width/2 - offset, -radius), (-lane_space - island_width/2 - offset, -radius))
                self.sim.create_quadratic_bezier_curve((-intersection_size/2, lane_space/2 + island_width/2 + offset), (-radius, lane_space/2 + island_width/2 + offset), (-radius, lane_space + island_width/2 + offset))    
            else:
                self.sim.create_quadratic_bezier_curve((lane_space/2 + island_width/2 + offset, intersection_size/2), (lane_space/2 + island_width/2 + offset, radius), (lane_space + island_width/2 + offset, radius))
                self.sim.create_quadratic_bezier_curve((intersection_size/2, -lane_space/2 - island_width/2 - offset), (radius, -lane_space/2 - island_width/2 - offset), (radius, -lane_space - island_width/2 - offset))
                self.sim.create_quadratic_bezier_curve((-lane_space/2 - island_width/2 - offset, - intersection_size/2), (-lane_space/2 - island_width/2 - offset, -radius), (-lane_space - island_width/2 - offset, -radius))
                self.sim.create_quadratic_bezier_curve((-intersection_size/2, lane_space/2 + island_width/2 + offset), (-radius, lane_space/2 + island_width/2 + offset), (-radius, lane_space + island_width/2 + offset))
            
            # Turn to exits
            self.sim.create_quadratic_bezier_curve((radius, lane_space + island_width/2 + offset), (radius, lane_space/2 + island_width/2 + offset), (intersection_size/2, lane_space/2 + island_width/2 + offset))
            self.sim.create_quadratic_bezier_curve((lane_space + island_width/2 + offset, -radius), (lane_space/2 + island_width/2 + offset, -radius), (lane_space/2 + island_width/2 + offset, -intersection_size/2))
            self.sim.create_quadratic_bezier_curve((-radius, -lane_space - island_width/2 - offset), (-radius, -lane_space/2 - island_width/2 - offset), (-intersection_size/2, -lane_space/2 - island_width/2 - offset))
            self.sim.create_quadratic_bezier_curve((-lane_space - island_width/2 - offset, radius), (-lane_space/2 - island_width/2 - offset, radius), (-lane_space/2 - island_width/2 - offset, intersection_size/2))

        # There's 24 total but count starts at 0, to get the other lane, just add 24
        self.vg = VehicleGenerator({
            'vehicles': [
                (1, {'path': [0, 16, 8, 20, 5], 'v_max': self.v}),
                # (1, {'path': [1, 17, 9, 21, 6], 'v_max': self.v}),
                # (1, {'path': [2, 18, 10, 22, 7], 'v_max': self.v}),
                # (1, {'path': [3, 19, 11, 23, 4], 'v_max': self.v}),
                # My test path
                (1, {'path': [0, 16, 32, 20, 5], 'v_max': self.v}),
            ], 'vehicle_rate': 30
        })

        self.sim.define_interfearing_paths([0, 16], [15, 8], turn=True)
        self.sim.define_interfearing_paths([1, 17], [12, 9], turn=True)
        self.sim.define_interfearing_paths([2, 18], [13, 10], turn=True)
        self.sim.define_interfearing_paths([3, 19], [14, 11], turn=True)
        self.sim.add_vehicle_generator(self.vg)

    def get_sim(self):
        return self.sim