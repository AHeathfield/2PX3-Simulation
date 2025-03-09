from trafficSimulator import *
from collections import deque
import numpy as np

# NOTE WHEN self.self_driving_vehicle_proportion = 1, the program doesn't work i'm not sure why, to counter this I made self_driving_team_simulation.py
class Intersection:
    def __init__(self):
        # ============================= Class Variables ==============================
        self.speed_variance = 2
        self.v = 8.5
        self.self_driving_vehicle_proportion = 0 # IMPORTANT number between 0 and 1, 0 means no self driving vehicles, 1 means entirely self driving vehicles
        if self.self_driving_vehicle_proportion == 1:
            self.v = self.v * 1.5
        self.vehicle_rate = 30*(1-self.self_driving_vehicle_proportion)
        self.self_driving_vehicle_rate = 30*self.self_driving_vehicle_proportion
        self.pedestrian_rate = 50
        self.sim = Simulation()
        # ============================================================================

        lane_space = 3.5
        intersection_size = 49 # roundabout
        # intersection_size = 24 # base
        island_width = 4
        length = 43.75 #roundabout
        # length = 100 #base
        radius = 18 
        offset = lane_space
        num_lanes = 2
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

        # ========================= Designated Turn Into Hospital ===========================
        emergencyConfig = {"color": (99,99,99)}

        # Entrance Lane 48
        entranceStartX = radius + 5*offset - 0.5
        entranceEndY = intersection_size/2 + 15
        entranceEndX = entranceStartX
        entranceStartY = lane_space + island_width/2 + 1.25*offset + offset
        # Top right (special case) ================================
        self.sim.create_segment((entranceStartX, -entranceStartY), (entranceEndX, -entranceEndY - 0.5), emergencyConfig)

        # Turn Into corner 49
        turnIntoRadius = 7.3 + 8*3.5
        turnStartX = entranceStartX
        turnStartY = entranceEndY
        # turnStartY = intersection_size/1.5 + 15
        turnEndX = entranceStartX + offset + 1
        turnEndY = radius + (intersection_size/1.5 - intersection_size/2) + 15 + 3
        newRadiusY = radius + (intersection_size/1.5 - intersection_size/2) + 15 + 3
        self.sim.create_quadratic_bezier_curve((turnStartX, -turnStartY), (turnIntoRadius, -newRadiusY), (turnEndX, -turnEndY), emergencyConfig)

        # Exit 50
        exitStartX = turnEndX
        exitStartY = turnEndY
        exitEndX = 2*length
        exitEndY = exitStartY
        self.sim.create_segment((exitStartX, -exitStartY), (exitEndX, -exitEndY), emergencyConfig)


        # Normal Emergency Entrances =======================
        # Emergency entrances 51-54
        entranceStartX = 2*offset + lane_space/2 + island_width/2
        entranceStartY = length + intersection_size/2
        entranceEndX = 2*offset + lane_space/2 + island_width/2
        entranceEndY = intersection_size/2 + 4*offset
        self.sim.create_segment((entranceStartX, entranceStartY), (entranceEndX, entranceEndY), emergencyConfig)
        # The entrance on the right will be a direct entrance
        self.sim.create_segment((entranceStartY, -entranceStartX), (entranceEndY + 5*offset, -entranceEndX), emergencyConfig)
        self.sim.create_segment((-entranceStartX, -entranceStartY), (-entranceEndX, -entranceEndY), emergencyConfig)
        self.sim.create_segment((-entranceStartY, entranceStartX), (-entranceEndY, entranceEndX), emergencyConfig)

        # Emergency outer ring corners 55-57
        cornerStartX = lane_space + island_width/2 + 1.25*offset + offset
        cornerStartY = radius + 5*offset - 0.5
        newRadius = radius + 5.3*3.3
        cornerEndX = cornerStartY
        cornerEndY = cornerStartX
        self.sim.create_quadratic_bezier_curve((cornerStartX, cornerStartY), (newRadius, newRadius), (cornerEndX, cornerEndY), emergencyConfig)
        # self.sim.create_quadratic_bezier_curve((cornerStartY, -cornerStartX), (newRadius, -newRadius), (cornerEndY, -cornerEndX), emergencyConfig)
        self.sim.create_quadratic_bezier_curve((-cornerStartX, -cornerStartY), (-newRadius, -newRadius), (-cornerEndX, -cornerEndY), emergencyConfig)
        self.sim.create_quadratic_bezier_curve((-cornerStartY, cornerStartX), (-newRadius, newRadius), (-cornerEndY, cornerEndX), emergencyConfig)

        # Emergency Outer Ring Connectors 58-60
        connectorStartX = cornerEndX
        # connectorStartY = lane_space + island_width/2 + 1.2*offset
        connectorStartY = cornerStartX
        connectorEndX = connectorStartX
        connectorEndY = connectorStartY
        self.sim.create_segment((connectorStartX, connectorStartY), (connectorEndX, -connectorEndY), emergencyConfig)
        # self.sim.create_segment((connectorStartY, -connectorStartX), (-connectorEndY, -connectorEndX), emergencyConfig)
        self.sim.create_segment((-connectorStartX, -connectorStartY), (-connectorEndX, connectorEndY), emergencyConfig)
        self.sim.create_segment((-connectorStartY, connectorStartX), (connectorEndY, connectorEndX), emergencyConfig)

        # Turn into emergency corners 61-64
        # emergencyConfig = {'color': (255, 0, 0)}
        turnIntoRadius = 7.3 + offset
        newRadius = radius + 4.8*offset
        turnStartX = lane_space/2 + island_width/2 + 2*offset
        turnStartY = intersection_size/2 + 4*offset
        turnEndX = lane_space + island_width/2 + 2*offset + 1
        turnEndY = turnStartY - offset
        self.sim.create_quadratic_bezier_curve((turnStartX, turnStartY), (turnIntoRadius, newRadius), (turnEndX, turnEndY), emergencyConfig)
        # The turn into for direct right entrance
        self.sim.create_quadratic_bezier_curve((turnStartY + 5*offset, -turnStartX), (newRadius + 5*offset, -turnIntoRadius), (turnEndY + 5*offset, -turnEndX), emergencyConfig)
        self.sim.create_quadratic_bezier_curve((-turnStartX, -turnStartY), (-turnIntoRadius, -newRadius), (-turnEndX, -turnEndY), emergencyConfig)
        self.sim.create_quadratic_bezier_curve((-turnStartY, turnStartX), (-newRadius, turnIntoRadius), (-turnEndY, turnEndX), emergencyConfig)

        # For the right direct entrance and into corner and exit 65-67
        rightEntranceStartX = radius + 5*offset - 0.5 + 5*offset
        rightEntranceEndY = intersection_size/2 + 15
        rightEntranceEndX = rightEntranceStartX
        rightEntranceStartY = lane_space + island_width/2 + 1.25*offset + offset
        self.sim.create_segment((rightEntranceStartX, -rightEntranceStartY), (rightEntranceEndX, -rightEntranceEndY - 0.5), emergencyConfig)
        turnIntoRadius = 7.3 + 8*3.5 + 5*offset
        turnStartX = rightEntranceEndX
        turnStartY = rightEntranceEndY
        # turnStartY = intersection_size/1.5 + 15
        turnEndX = rightEntranceStartX + offset + 1
        turnEndY = radius + (intersection_size/1.5 - intersection_size/2) + 15 + 3
        newRadiusY = radius + (intersection_size/1.5 - intersection_size/2) + 15 + 3
        self.sim.create_quadratic_bezier_curve((turnStartX, -turnStartY), (turnIntoRadius, -newRadiusY), (turnEndX, -turnEndY), emergencyConfig)
        rightExitStartX = turnEndX
        rightExitStartY = turnEndY
        rightExitEndX = 2*length
        rightExitEndY = rightExitStartY
        self.sim.create_segment((rightExitStartX, -rightExitStartY), (rightExitEndX, -rightExitEndY), emergencyConfig)


        # ========================== Pedestrian =======================
        # Overpass 68-71
        crossStartX = lane_space + island_width/2 + 5*offset
        crossStartY =  2*offset + lane_space/2 + island_width/2
        crossEndX = crossStartX
        crossEndY = -crossStartY
        overpassConfigs = {"color": (205,133,63)}
        self.sim.create_segment((crossStartX, crossStartY), (crossEndX, crossEndY), overpassConfigs)
        self.sim.create_segment((-crossStartY, -crossStartX), (-crossEndY, -crossEndX), overpassConfigs)
        self.sim.create_segment((-crossStartX, -crossStartY), (-crossEndX, -crossEndY), overpassConfigs)
        self.sim.create_segment((crossStartY, crossStartX), (crossEndY, crossEndX), overpassConfigs)

        # ========================== Bus Lanes ===================
        # 72-75
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
                (1, {'path': bottomToRight, 'v_max': self.v+ 2*self.speed_variance*np.random.random() -self.speed_variance}),
                (1, {'path': bottomToTop, 'v_max': self.v+ 2*self.speed_variance*np.random.random() -self.speed_variance}),
                (1, {'path': rightToTop, 'v_max': self.v+ 2*self.speed_variance*np.random.random() -self.speed_variance}),
                (1, {'path': rightToLeft, 'v_max': self.v+ 2*self.speed_variance*np.random.random() -self.speed_variance}),
                (1, {'path': topToLeft, 'v_max': self.v+ 2*self.speed_variance*np.random.random() -self.speed_variance}),
                (1, {'path': topToBottom, 'v_max': self.v+ 2*self.speed_variance*np.random.random() -self.speed_variance}),
                (1, {'path': leftToBottom, 'v_max': self.v+ 2*self.speed_variance*np.random.random() -self.speed_variance}),
                (1, {'path': leftToRight, 'v_max': self.v+ 2*self.speed_variance*np.random.random() -self.speed_variance}),
            ],
            'vehicle_rate': self.vehicle_rate
        })
        self.innerRingVehicles = VehicleGenerator({
            'vehicles': [
                (1, {'path': self.nextLanePath(bottomToRight), 'v_max': self.v+ 2*self.speed_variance*np.random.random() -self.speed_variance}),
                (1, {'path': self.nextLanePath(bottomToTop), 'v_max': self.v+ 2*self.speed_variance*np.random.random() -self.speed_variance}),
                (1, {'path': self.nextLanePath(rightToTop), 'v_max': self.v+ 2*self.speed_variance*np.random.random() -self.speed_variance}),
                (1, {'path': self.nextLanePath(rightToLeft), 'v_max': self.v+ 2*self.speed_variance*np.random.random() -self.speed_variance}),
                (1, {'path': self.nextLanePath(topToLeft), 'v_max': self.v+ 2*self.speed_variance*np.random.random() -self.speed_variance}),
                (1, {'path': self.nextLanePath(topToBottom), 'v_max': self.v+ 2*self.speed_variance*np.random.random() -self.speed_variance}),
                (1, {'path': self.nextLanePath(leftToBottom), 'v_max': self.v+ 2*self.speed_variance*np.random.random() -self.speed_variance}),
                (1, {'path': self.nextLanePath(leftToRight), 'v_max': self.v+ 2*self.speed_variance*np.random.random() -self.speed_variance}),
            ],
            'vehicle_rate': self.vehicle_rate
        })

        # Self Driving vehicles
        self.selfDrivingOuterRingVehicles = VehicleGenerator({
            'vehicles': [
                (1, {'path': bottomToRight, 'v_max': self.v, 'T':0.1,'s0' : 4}),
                (1, {'path': bottomToTop, 'v_max': self.v, 'T':0.1,'s0' : 4}),
                (1, {'path': rightToTop, 'v_max': self.v, 'T':0.1,'s0' : 4}),
                (1, {'path': rightToLeft, 'v_max': self.v, 'T':0.1,'s0' : 4}),
                (1, {'path': topToLeft, 'v_max': self.v, 'T':0.1,'s0' : 4}),
                (1, {'path': topToBottom, 'v_max': self.v, 'T':0.1,'s0' : 4}),
                (1, {'path': leftToBottom, 'v_max': self.v, 'T':0.1,'s0' : 4}),
                (1, {'path': leftToRight, 'v_max': self.v, 'T':0.1,'s0' : 4}),
            ],
            'vehicle_rate': self.self_driving_vehicle_rate
        })
        self.selfDrivingInnerRingVehicles = VehicleGenerator({
            'vehicles': [
                (1, {'path': self.nextLanePath(bottomToRight), 'v_max': self.v, 'T':0.1, 's0' : 4, 'colour': (114, 4, 204)}),
                (1, {'path': self.nextLanePath(bottomToTop), 'v_max': self.v, 'T':0.1, 's0' : 4, 'colour': (114, 4, 204)}),
                (1, {'path': self.nextLanePath(rightToTop), 'v_max': self.v, 'T':0.1, 's0' : 4, 'colour': (114, 4, 204)}),
                (1, {'path': self.nextLanePath(rightToLeft), 'v_max': self.v, 'T':0.1, 's0' : 4, 'colour': (114, 4, 204)}),
                (1, {'path': self.nextLanePath(topToLeft), 'v_max': self.v, 'T':0.1, 's0' : 4, 'colour': (114, 4, 204)}),
                (1, {'path': self.nextLanePath(topToBottom), 'v_max': self.v, 'T':0.1, 's0' : 4, 'colour': (114, 4, 204)}),
                (1, {'path': self.nextLanePath(leftToBottom), 'v_max': self.v, 'T':0.1, 's0' : 4, 'colour': (114, 4, 204)}),
                (1, {'path': self.nextLanePath(leftToRight), 'v_max': self.v, 'T':0.1, 's0' : 4, 'colour': (114, 4, 204)}),
            ],
            'vehicle_rate': self.self_driving_vehicle_rate
        })

        # Emergency Vehicle RED

        emergencyTopToHospital = [53, 63, 56, 59, 57, 60, 55, 58, 48, 49, 50]
        emergencyLeftToHospital = [54, 64, 57, 60, 55, 58, 48, 49, 50]
        emergencyBottomToHospital = [51, 61, 55, 58, 48, 49, 50]
        emergencyRightToHospital = [52, 62, 65, 66, 67]
        emergencyVehicleRate = self.vehicle_rate/2
        self.emergencyVehicles = VehicleGenerator({
            'vehicles': [
                (1, {'path': emergencyTopToHospital, 'v_max': self.v+ 2*self.speed_variance*np.random.random() -self.speed_variance + 30}),
                (1, {'path': emergencyLeftToHospital, 'v_max': self.v+ 2*self.speed_variance*np.random.random() -self.speed_variance + 30}),
                (1, {'path': emergencyBottomToHospital, 'v_max': self.v+ 2*self.speed_variance*np.random.random() -self.speed_variance + 30}),
                (1, {'path': emergencyRightToHospital, 'v_max': self.v+ 2*self.speed_variance*np.random.random() -self.speed_variance + 30}),
            ],
            'vehicle_rate': emergencyVehicleRate
        })

        selfDrivingEmergencyVehicleRate = self.self_driving_vehicle_rate / 2
        self.selfDrivingEmergencyVehicles = VehicleGenerator({
            'vehicles': [
                (1, {'path': emergencyTopToHospital, 'v_max': self.v + 30, 'T':0.1,'s0' : 4}),
                (1, {'path': emergencyLeftToHospital, 'v_max': self.v + 30, 'T':0.1,'s0' : 4}),
                (1, {'path': emergencyBottomToHospital, 'v_max': self.v + 30, 'T':0.1,'s0' : 4}),
                (1, {'path': emergencyRightToHospital, 'v_max': self.v + 30, 'T':0.1,'s0' : 4}),
            ],
            'vehicle_rate': selfDrivingEmergencyVehicleRate
        })
        # Pedestrians Green
        overpassRight = [68]
        overpassTop = [69]
        overpassLeft = [70]
        overpassBottom = [71]
        self.pedestrians = VehicleGenerator({
            'vehicles': [
                (1, {'path': overpassRight, 'v_max': self.v - 5, 'colour': (0, 255, 0), 'l': 1, 'w': 1, 's0': 0}),
                (1, {'path': overpassTop, 'v_max': self.v - 5, 'colour': (0, 255, 0), 'l': 1, 'w': 1, 's0': 0}),
                (1, {'path': overpassLeft, 'v_max': self.v - 5, 'colour': (0, 255, 0), 'l': 1, 'w': 1, 's0': 0}),
                (1, {'path': overpassBottom, 'v_max': self.v - 5, 'colour': (0, 255, 0), 'l': 1, 'w': 1, 's0': 0}),
            ],
            'vehicle_rate': self.pedestrian_rate
        })
        # Entrances 0-3, 24-27
        # Exits 4-7, 28-31
        # Ring Corners (Outer)8-11, (Inner)32-35
        # Ring Connectors (Outer)12-15, (Inner)36-39 (STARTS AT RIGHT)
        # Turn into corners 16-19, 40-43
        # Turn into exits 20-23, 44-47
        # Right Lane Entrances to into corners, into corners to connectors
        self.sim.define_interfearing_paths([0, 16], [16, 15], turn=True)
        self.sim.define_interfearing_paths([1, 17], [17, 12], turn=True)
        self.sim.define_interfearing_paths([2, 18], [18, 13], turn=True)
        self.sim.define_interfearing_paths([3, 19], [19, 14], turn=True)

        # Left Lane Entrances to into corners, into corners to inner connectors
        self.sim.define_interfearing_paths([24, 40], [40, 39], turn=True)
        self.sim.define_interfearing_paths([25, 41], [41, 36], turn=True)
        self.sim.define_interfearing_paths([26, 42], [42, 37], turn=True)
        self.sim.define_interfearing_paths([27, 43], [43, 38], turn=True)
        # This is for the into corners to outer connectors (this is a very confusing method)
        self.sim.define_interfearing_paths([24, 40], [40, 15], turn=True)
        self.sim.define_interfearing_paths([25, 41], [41, 12], turn=True)
        self.sim.define_interfearing_paths([26, 42], [42, 13], turn=True)
        self.sim.define_interfearing_paths([27, 43], [43, 14], turn=True)
        
        # Left Turn intos interfearing with right lane connectors
        self.sim.define_interfearing_paths([40, 15], [41, 12], turn=True)
        self.sim.define_interfearing_paths([42, 13], [43, 14], turn=True)

        # Adding vehicle generators to the simulate
        self.sim.add_vehicle_generator(self.outerRingVehicles)
        self.sim.add_vehicle_generator(self.innerRingVehicles)
        self.sim.add_vehicle_generator(self.emergencyVehicles)
        self.sim.add_vehicle_generator(self.selfDrivingOuterRingVehicles)
        self.sim.add_vehicle_generator(self.selfDrivingInnerRingVehicles)
        self.sim.add_vehicle_generator(self.selfDrivingEmergencyVehicles)
        self.sim.add_vehicle_generator(self.pedestrians)

    def get_sim(self):
        return self.sim
    
    # The way I defined the segments is I created on lane for whole roundabout, then I added another lane, each lane consists of 24 segments, to get to the next lane you simply add 24 to the segment index
    def nextLanePath(self, path):
        newPath = path.copy() # Just in case we don't want to modify the given list
        for i in range(len(path)):
            newPath[i] += 24
        return newPath

