from trafficSimulator import *
from collections import deque
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

        offset = lane_space
        adj = 0.45   # This is a number to adjust the offset
        
        
        # My new "cleaner" code
        # ==================== Right Lanes =====================
        # Entrances 0-3, 24-27
        entranceStartX = offset + lane_space/2 + island_width/2
        entranceStartY = length + intersection_size/2
        entranceEndX = offset + lane_space/2 + island_width/2
        entranceEndY = intersection_size/2
        self.sim.create_segment((entranceStartX, entranceStartY), (entranceEndX, entranceEndY))
        self.sim.create_segment((entranceStartY, -entranceStartX), (entranceEndY, -entranceEndX))
        self.sim.create_segment((-entranceStartX, -entranceStartY), (-entranceEndX, -entranceEndY))
        self.sim.create_segment((-entranceStartY, entranceStartX), (-entranceEndY, entranceEndX))

        # Exits 4-7, 28-31
        self.sim.create_segment((-entranceEndX, entranceEndY), (-entranceStartX, entranceStartY))
        self.sim.create_segment((entranceEndY, entranceEndX), (entranceStartY, entranceStartX))
        self.sim.create_segment((entranceEndX, -entranceEndY), (entranceStartX, -entranceStartY))
        self.sim.create_segment((-entranceEndY, -entranceEndX), (-entranceStartY, -entranceStartX))

        # Outer Ring Corners 8-11
        cornerStartX = lane_space + island_width/2 + 1.25*offset
        cornerStartY = radius
        cornerEndX = cornerStartY
        cornerEndY = cornerStartX
        self.sim.create_quadratic_bezier_curve((cornerStartX, cornerStartY), (radius, radius), (cornerEndX, cornerEndY))
        self.sim.create_quadratic_bezier_curve((cornerStartY, -cornerStartX), (radius, -radius), (cornerEndY, -cornerEndX))
        self.sim.create_quadratic_bezier_curve((-cornerStartX, -cornerStartY), (-radius, -radius), (-cornerEndX, -cornerEndY))
        self.sim.create_quadratic_bezier_curve((-cornerStartY, cornerStartX), (-radius, radius), (-cornerEndY, cornerEndX))

        # Outer Ring Connectors 12-15
        connectorStartX = radius
        connectorStartY = lane_space + island_width/2 + 1.2*offset
        connectorEndX = connectorStartX
        connectorEndY = connectorStartY
        self.sim.create_segment((connectorStartX, connectorStartY), (connectorEndX, -connectorEndY))
        self.sim.create_segment((connectorStartY, -connectorStartX), (-connectorEndY, -connectorEndX))
        self.sim.create_segment((-connectorStartX, -connectorStartY), (-connectorEndX, connectorEndY))
        self.sim.create_segment((-connectorStartY, connectorStartX), (connectorEndY, connectorEndX))

        # Turn into corners 16-19
        turnIntoRadius = 7.3
        turnStartX = lane_space/2 + island_width/2 + offset
        turnStartY = intersection_size/2
        turnEndX = lane_space + island_width/2 + offset + 1
        turnEndY = radius
        self.sim.create_quadratic_bezier_curve((turnStartX, turnStartY), (turnIntoRadius, radius), (turnEndX, turnEndY))
        self.sim.create_quadratic_bezier_curve((turnStartY, -turnStartX), (radius, -turnIntoRadius), (turnEndY, -turnEndX))
        self.sim.create_quadratic_bezier_curve((-turnStartX, -turnStartY), (-turnIntoRadius, -radius), (-turnEndX, -turnEndY))
        self.sim.create_quadratic_bezier_curve((-turnStartY, turnStartX), (-radius, turnIntoRadius), (-turnEndY, turnEndX))

        # Turn into exits 20-23  
        self.sim.create_quadratic_bezier_curve((turnEndY, turnEndX), (radius, turnIntoRadius), (turnStartY, turnStartX))
        self.sim.create_quadratic_bezier_curve((turnEndX, -turnEndY), (turnIntoRadius, -radius), (turnStartX, -turnStartY))
        self.sim.create_quadratic_bezier_curve((-turnEndY, -turnEndX), (-radius, -turnIntoRadius), (-turnStartY, -turnStartX))
        self.sim.create_quadratic_bezier_curve((-turnEndX, turnEndY), (-turnIntoRadius, radius), (-turnStartX, turnStartY))

        # ================================ LEFT LANES =======================================
        # Entrances 24-27
        entranceStartX = lane_space/2 + island_width/2
        entranceStartY = length + intersection_size/2
        entranceEndX = lane_space/2 + island_width/2
        entranceEndY = intersection_size/2
        self.sim.create_segment((entranceStartX, entranceStartY), (entranceEndX, entranceEndY))
        self.sim.create_segment((entranceStartY, -entranceStartX), (entranceEndY, -entranceEndX))
        self.sim.create_segment((-entranceStartX, -entranceStartY), (-entranceEndX, -entranceEndY))
        self.sim.create_segment((-entranceStartY, entranceStartX), (-entranceEndY, entranceEndX))

        # Exits 28-31
        self.sim.create_segment((-entranceEndX, entranceEndY), (-entranceStartX, entranceStartY))
        self.sim.create_segment((entranceEndY, entranceEndX), (entranceStartY, entranceStartX))
        self.sim.create_segment((entranceEndX, -entranceEndY), (entranceStartX, -entranceStartY))
        self.sim.create_segment((-entranceEndY, -entranceEndX), (-entranceStartY, -entranceStartX))
        
        # Inner Ring Corners 32-35
        cornerStartX = lane_space + island_width/2 + 1.25*offset
        cornerStartY = radius - 1.0*offset
        innerRadius = radius - 3.3 
        cornerEndX = cornerStartY
        cornerEndY = cornerStartX
        self.sim.create_quadratic_bezier_curve((cornerStartX, cornerStartY), (innerRadius, innerRadius), (cornerEndX, cornerEndY))
        self.sim.create_quadratic_bezier_curve((cornerStartY, -cornerStartX), (innerRadius, -innerRadius), (cornerEndY, -cornerEndX))
        self.sim.create_quadratic_bezier_curve((-cornerStartX, -cornerStartY), (-innerRadius, -innerRadius), (-cornerEndX, -cornerEndY))
        self.sim.create_quadratic_bezier_curve((-cornerStartY, cornerStartX), (-innerRadius, innerRadius), (-cornerEndY, cornerEndX))
        
        # Inner Ring Connectors 36-39
        connectorStartX = radius - offset
        connectorStartY = lane_space + island_width/2 + 1.2*offset
        connectorEndX = connectorStartX
        connectorEndY = connectorStartY
        self.sim.create_segment((connectorStartX, connectorStartY), (connectorEndX, -connectorEndY))
        self.sim.create_segment((connectorStartY, -connectorStartX), (-connectorEndY, -connectorEndX))
        self.sim.create_segment((-connectorStartX, -connectorStartY), (-connectorEndX, connectorEndY))
        self.sim.create_segment((-connectorStartY, connectorStartX), (connectorEndY, connectorEndX))

        # Turn into corners 40-43
        turnIntoRadius = 7.3 - offset
        newRadius = radius - offset
        turnStartX = lane_space/2 + island_width/2
        turnStartY = intersection_size/2
        turnEndX = lane_space + island_width/2 + offset + 1
        turnEndY = radius - offset
        self.sim.create_quadratic_bezier_curve((turnStartX, turnStartY), (turnIntoRadius, newRadius), (turnEndX, turnEndY))
        self.sim.create_quadratic_bezier_curve((turnStartY, -turnStartX), (newRadius, -turnIntoRadius), (turnEndY, -turnEndX))
        self.sim.create_quadratic_bezier_curve((-turnStartX, -turnStartY), (-turnIntoRadius, -newRadius), (-turnEndX, -turnEndY))
        self.sim.create_quadratic_bezier_curve((-turnStartY, turnStartX), (-newRadius, turnIntoRadius), (-turnEndY, turnEndX))

        # Turn into exits 44-47
        self.sim.create_quadratic_bezier_curve((turnEndY, turnEndX), (newRadius, turnIntoRadius), (turnStartY, turnStartX))
        self.sim.create_quadratic_bezier_curve((turnEndX, -turnEndY), (turnIntoRadius, -newRadius), (turnStartX, -turnStartY))
        self.sim.create_quadratic_bezier_curve((-turnEndY, -turnEndX), (-newRadius, -turnIntoRadius), (-turnStartY, -turnStartX))
        self.sim.create_quadratic_bezier_curve((-turnEndX, turnEndY), (-turnIntoRadius, newRadius), (-turnStartX, turnStartY))

        # ========================= Designated Right Turn Into Hospital ===========================
        emergencyConfig = {} #{"color": (99,99,99)}

        # Entrance Lane 48
        entranceStartX = 2*offset + lane_space/2 + island_width/2
        entranceEndY = intersection_size/2 + 15
        entranceEndX = 2*offset + lane_space/2 + island_width/2
        entranceStartY = intersection_size/2
        self.sim.create_segment((entranceStartX, -entranceStartY), (entranceEndX, -entranceEndY))
        
        # Turn Into corner 49
        turnIntoRadius = 7.3 + 3.5
        turnStartX = lane_space/2 + island_width/2 + 2*offset
        turnStartY = entranceEndY - 1
        turnEndX = lane_space + island_width/2 + 2*offset + 1
        turnEndY = radius + (intersection_size/1.5 - intersection_size/2) + 15
        newRadiusY = radius + (intersection_size/1.5 - intersection_size/2) + 15
        self.sim.create_quadratic_bezier_curve((turnStartX, -turnStartY), (turnIntoRadius, -newRadiusY), (turnEndX, -turnEndY))

        # Exit 50
        exitStartX = turnEndX - 0.5
        exitStartY = turnEndY
        exitEndX = 2*length
        exitEndY = exitStartY
        self.sim.create_segment((exitStartX, -exitStartY), (exitEndX, -exitEndY))


        # Normal Emergency Entrances =======================
        # Entrance 51
        entranceStartY += length
        entranceEndY -= offset
        # self.sim.create_segment((entranceStartX, entranceStartY), (entranceEndX, entranceEndY))
        
        # ========================== Pedestrian =======================
        # Overpass 51-54
        crossStartX = lane_space + island_width/2 + 5*offset
        crossStartY =  2*offset + lane_space/2 + island_width/2
        crossEndX = crossStartX
        crossEndY = -crossStartY
        self.sim.create_segment((crossStartX, crossStartY), (crossEndX, crossEndY))
        self.sim.create_segment((-crossStartY, -crossStartX), (-crossEndY, -crossEndX))
        self.sim.create_segment((-crossStartX, -crossStartY), (-crossEndX, -crossEndY))
        self.sim.create_segment((crossStartY, crossStartX), (crossEndY, crossEndX))

        # ========================== Bus Lanes =========================
        entranceStartX = 2*offset + lane_space/2 + island_width/2
        entranceStartY = length + intersection_size/2
        entranceEndX = 2*offset + lane_space/2 + island_width/2
        entranceEndY = intersection_size/2 + 4*offset
        self.sim.create_segment((-entranceEndX, entranceEndY), (-entranceStartX, entranceStartY))
        self.sim.create_segment((entranceEndY, entranceEndX), (entranceStartY, entranceStartX))
        self.sim.create_segment((entranceEndX, -entranceEndY), (entranceStartX, -entranceStartY))
        self.sim.create_segment((-entranceEndY, -entranceEndX), (-entranceStartY, -entranceStartX))

        # My new simulations
        # Entrances 0-3, 24-27
        # Exits 4-7, 28-31
        # Outer Ring Corners 8-11, 32-35
        # Outer Ring Connectors 12-15, 36-39
        # Turn into corners 16-19, 40-43
        # Turn into exits 20-23, 44-47
        # =============================== PATHS =============================
        bottomToRight = [0, 16, 8, 20, 5]
        bottomToTop = [0, 16, 8, 12, 9, 21, 6]
        rightToTop = [1, 17, 9, 21, 6]
        rightToLeft = [1, 17, 9, 13, 10, 22, 7]
        topToLeft = [2, 18, 10, 22, 7]
        topToBottom = [2, 18, 10, 14, 11, 23, 4]
        leftToBottom = [3, 19, 11, 23, 4]
        leftToRight = [3, 19, 11, 15, 8, 20, 5]
        # Regular vehicles BLUE
        self.outerRingVehicles = VehicleGenerator({
            'vehicles': [
                # Bottom Right Lane -> Right Right Lane
                (1, {'path': bottomToRight, 'v_max': self.v}),
                # (1, {'path': [0, 16], 'v_max': self.v}),
                # Bottom Right Lane -> Top Right Lane
                (1, {'path': bottomToTop, 'v_max': self.v}),
                # Right Right Lane -> Top Right Lane
                (1, {'path': rightToTop, 'v_max': self.v}),
                # Right Right Lane -> Left Right Lane
                (1, {'path': rightToLeft, 'v_max': self.v}),
                # Top Right Lane -> Left Right Lane
                (1, {'path': topToLeft, 'v_max': self.v}),
                # Top Right Lane -> Bottom Right Lane
                (1, {'path': topToBottom, 'v_max': self.v}),
                # Left Right Lane -> Bottom Right Lane
                (1, {'path': leftToBottom, 'v_max': self.v}),
                # Left Right Lane -> Right Right Lane
                (1, {'path': leftToRight, 'v_max': self.v}),
            ],
            'vehicle_rate': 30
        })
        self.innerRingVehicles = VehicleGenerator({
            'vehicles': [
                (1, {'path': self.nextLanePath(bottomToRight), 'v_max': self.v}),
                (1, {'path': self.nextLanePath(bottomToTop), 'v_max': self.v}),
                (1, {'path': self.nextLanePath(rightToTop), 'v_max': self.v}),
                (1, {'path': self.nextLanePath(rightToLeft), 'v_max': self.v}),
                (1, {'path': self.nextLanePath(topToLeft), 'v_max': self.v}),
                (1, {'path': self.nextLanePath(topToBottom), 'v_max': self.v}),
                (1, {'path': self.nextLanePath(leftToBottom), 'v_max': self.v}),
                (1, {'path': self.nextLanePath(leftToRight), 'v_max': self.v}),
            ],
            'vehicle_rate': 30
        })
        # Emergency Vehicle RED
        emergencyIntoHospital = [48, 49, 50]
        self.emergencyVehicles = VehicleGenerator({
            'vehicles': [
                (1, {'path': emergencyIntoHospital, 'v_max': self.v + 30}),
            ],
            'vehicle_rate': 10
        })

        # Do not show
        # Pedestrians Green
        overpassRight = [51]
        overpassTop = [52]
        overpassLeft = [53]
        overpassBottom = [54]
        self.pedestrians = VehicleGenerator({
            'vehicles': [
                (1, {'path': overpassRight, 'v_max': self.v - 5, 'colour': (0, 255, 0), 'l': 1, 'w': 1}),
                (1, {'path': overpassTop, 'v_max': self.v - 5, 'colour': (0, 255, 0), 'l': 1, 'w': 1}),
                (1, {'path': overpassLeft, 'v_max': self.v - 5, 'colour': (0, 255, 0), 'l': 1, 'w': 1}),
                (1, {'path': overpassRight, 'v_max': self.v - 5, 'colour': (0, 255, 0), 'l': 1, 'w': 1}),
            ],
            'vehicle_rate': 10
        })
        # Entrances 0-3, 24-27
        # Exits 4-7, 28-31
        # Ring Corners (Outer)8-11, (Inner)32-35
        # Ring Connectors (Outer)12-15, (Inner)36-39
        # Turn into corners 16-19, 40-43
        # Turn into exits 20-23, 44-47
        # Right Lane Entrances and Connectors to Corners
        self.sim.define_interfearing_paths([0, 16], [15, 8], turn=True)
        self.sim.define_interfearing_paths([1, 17], [12, 9], turn=True)
        self.sim.define_interfearing_paths([2, 18], [13, 10], turn=True)
        self.sim.define_interfearing_paths([3, 19], [14, 11], turn=True)
        # Left Lane Entrances and Connectors to Corners
        self.sim.define_interfearing_paths([24, 40], [39, 32], turn=True)
        self.sim.define_interfearing_paths([25, 41], [36, 33], turn=True)
        self.sim.define_interfearing_paths([26, 42], [37, 34], turn=True)
        self.sim.define_interfearing_paths([27, 43], [38, 35], turn=True)
        # Right Lane Turn intos interfearing with connectors, turn into interfearing with corner
        # self.sim.define_interfearing_paths([17, 9], [16, 8], turn=True)
        # self.sim.define_interfearing_paths([16, 15], [15, 16], turn=True)
        # Left Turn intos interfearing with right lane connectors
        self.sim.define_interfearing_paths([40, 15], [41, 12], turn=True)
        self.sim.define_interfearing_paths([42, 13], [43, 14], turn=True)

        # Adding vehicle generators to the simulate
        self.sim.add_vehicle_generator(self.outerRingVehicles)
        self.sim.add_vehicle_generator(self.innerRingVehicles)
        self.sim.add_vehicle_generator(self.emergencyVehicles)
        # self.sim.add_vehicle_generator(self.pedestrians)

    def get_sim(self):
        return self.sim
    
    # The way I defined the segments is I created on lane for whole roundabout, then I added another lane, each lane consists of 24 segments, to get to the next lane you simply add 24 to the segment index
    def nextLanePath(self, path):
        newPath = path.copy() # Just in case we don't want to modify the given list
        for i in range(len(path)):
            newPath[i] += 24
        return newPath
