
from trafficSimulator import *
import numpy as np


class Intersection:
    def __init__(self):
        self.sim = Simulation()
        lane_space = 3.5
        intersection_size = 24
        island_width = 2
        length = 100


#---------------------------------------------------------------Variables----------------------------------------------------------------------------#
        self.vehicle_rate = 10
        self.v = 19
        self.speed_variance = 0
        self.self_driving_vehicle_proportion = 0 #number between 0 and 1, 0 means no self driving vehicles, 1 means entirely self driving vehicles
        if self.self_driving_vehicle_proportion == 1:
            self.v = self.v * 1.5

#--------------------------------------
        # 0,0 is very middle, 224 by 224, lane space by default is 3.5 which is why we do 1.75 (half)
        self.sim.create_segment((1.75, (length + intersection_size / 2)), (1.75, (intersection_size / 2))) # South entrance
        #intersection_size / 2 is 12

        self.sim.create_quadratic_bezier_curve((1.75, 12), (1.75, 1.75), (12, 1.75))# Creates a turn

        # Creates VehicleGenerator with params
        self.vg = VehicleGenerator({
            'vehicles': [
                #(1, {'path': [0,1], 'v_max': self.v * self.speed_variance*np.random.random() -self.speed_variance}),
                (1, {'path': [0, 1], 'v_max': self.v+ 2*self.speed_variance*np.random.random() -self.speed_variance})
            ],
            'vehicle_rate': self.vehicle_rate
        })

        # Self driving
        # self.sdvg = VehicleGenerator({
        #     'vehicles': [
        #         (1, {'path': [0, 16, 12], 'v_max': self.v, 'T' : 0.1,'s0' : 4})
        #     ],
        #     'vehicle_rate': self.vehicle_rate
        # })

        #Signal lights
        # self.sig = TrafficSignal([
        #     [self.sim.segments[0], self.sim.segments[8]], [self.sim.segments[7], self.sim.segments[9]],
        #     {'cycle': [(False,True,30), (True,False,60)]}
        # ])

        #Adds VehicleGenerator to the simulation
        self.sim.add_vehicle_generator(self.vg)
        # self.sim.add_vehicle_generator(self.sdvg)

    def get_sim(self):
        return self.sim