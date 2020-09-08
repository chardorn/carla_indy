import carla
import random
import math
import numpy as np
import time
import transforms3d
import threading
import matplotlib.pyplot as plt
 
world = None
spectator = None
vehicle_list = []
physics_control = None
lock1 = threading.Lock()
lock = threading.Lock()



def main():
    global world
    global vehicle_list
    global lock

    client = carla.Client('127.0.0.1', 2000)
    client.set_timeout(10.0)
    # Read the opendrive file to a string
    xodr_path = "speedway_5lanes.xodr"
    #xodr_path = "Crossing8Course.xodr"
    od_file = open(xodr_path)
    data = od_file.read()

    # Load the opendrive map
    vertex_distance = 2.0  # in meters
    max_road_length = 10.0 # in meters #Changed from 50.0 
    wall_height = 5.0      # in meters
    extra_width = 0.6      # in meters
    world = client.generate_opendrive_world(
        data, carla.OpendriveGenerationParameters(
        vertex_distance=vertex_distance,
        max_road_length=max_road_length,
        wall_height=wall_height,
        additional_width=extra_width,
        smooth_junctions=True,
        enable_mesh_visibility=True))


    transform = waypoints = world.get_map().generate_waypoints(10.0)
    #Change targetLane and waypoint index depending on where you want the vehicle to spawn
    targetLane = -3 
    waypoint = waypoints[500] 
    waypoint = change_lane(waypoint, targetLane - waypoint.lane_id)
    location = waypoint.transform.location + carla.Vector3D(0, 0, 1.5)
    rotation = waypoint.transform.rotation
    transform = carla.Transform(location, rotation)

    vehicle_1 = Autonomous_Vehicle(['lidar', 'collision', 'camera'], transform, Disparity_Extender(), Pure_Pursuit_Controller())
    vehicle_list.append(vehicle_1)

    vehicle_1.start_driving()

    while(True):
        time.sleep(0.1)



### Local Planner ###

class Pure_Pursuit_Controller(): 
    def __init__(self): 
        global vehicle_list

        #Vehicle properties setup
        self.KP = 0.2
        self.KD = 0.1 
        self.past_error = 0
        self.error = 0
        self.sum_error = 0

        #Will be set up by calling set_physics_control()
        self.max_steer = None
        self.wheelbase = None

    def set_physics_control(self, physics_control, location):
        self.max_steer = physics_control.wheels[0].max_steer_angle
        rear_axle_center = (physics_control.wheels[2].position + physics_control.wheels[3].position)/200
        offset = rear_axle_center - location
        self.wheelbase = np.linalg.norm([offset.x, offset.y, offset.z])
        

    def throttle_control(self, speed):
        if speed >= 30:
            return 0.7
        else:
            return 1.0

    def steering_control(self, vehicle_tr, waypoint_tr):
        wp_loc_rel = relative_location(vehicle_tr, waypoint_tr.location) + carla.Vector3D(self.wheelbase, 0, 0)
        x = wp_loc_rel.x
        y = wp_loc_rel.y
        steer_rad = math.tan(y / x)
        steer_deg = np.degrees(steer_rad)
        if abs(steer_deg) < 50:
            return steer_deg / (2 * self.max_steer)
        if abs(steer_deg) < 5:
            return 0
        steer = np.clip(steer_deg, -self.max_steer, self.max_steer)

        return steer / self.max_steer

    def cartesian_steering_control(self, cartesian_point):
        range = 40
        x = cartesian_point[0]
        if x < 0:
            x = x + 3
        if x > 0: 
            x = x - 3
        y = cartesian_point[1] 
        steer_rad = np.arctan2(x, y) #Changed from tan to atan
        steer_deg = np.degrees(steer_rad)
        steer = np.clip(steer_deg, -self.max_steer, self.max_steer)
        if abs(steer_deg) < 50:
            return - steer_deg / (2 * self.max_steer)
        if abs(steer_deg) < 5:
            return 0

        return - steer / self.max_steer

### Global Planner ###

class Disparity_Extender():
    def __init__(self):
        return 

    def get_target_cartesian(self, cartesian_coordinates):
        #cartesian coordinates are in the car's reference frame
        max_distance = 0
        max_disparity_pair = [[],[]]
        #The disparities are in the form of polar coordinates, so
        #the pair is in the form [[radius1, degrees1, height], [radius2, degrees2, height]
        #max_disparity_pair[0] is to the left of max_disparity_pair[1]
        if cartesian_coordinates is None:
            return
        for i in range(len(cartesian_coordinates) - 1):
            if cartesian_coordinates[i][1] > 0:
                x1 = cartesian_coordinates[i][0]
                y1 =  cartesian_coordinates[i][1]
                x2 = cartesian_coordinates[i+1][0]
                y2 =  cartesian_coordinates[i+1][1]
                distance = math.sqrt((x1-x2)**2 + (y1-y2)**2)
                if distance > max_distance and distance > 5:
                    max_distance = distance
                    index1 = i
                    index2 = i + 1

        return cartesian_coordinates[index1]


class Autonomous_Vehicle(object):

    def __init__(self, sensor_list, transform, global_planner, local_planner):
        global world
        global spectator
        global vehicle_list

        blueprint = world.get_blueprint_library().filter('vehicle.*model3*')[0]
        self.vehicle = world.spawn_actor(blueprint, transform)
        self.vehicle.set_simulate_physics(True)
        vehicle_list.append(self.vehicle)

        self.actor_list = []
        self.global_planner = global_planner
        self.local_planner = local_planner
        self.local_planner.set_physics_control(self.vehicle.get_physics_control(), self.vehicle.get_location())


        self.camera = None
        self.spectator = None
        if 'lidar' in sensor_list:
            self.attach_lidar_sensor()
        if 'collision' in sensor_list:
            self.attach_collision_sensor()
        if 'camera' in sensor_list:
            self.camera, self.spectator = self.attach_camera_sensor()

        #Starting values
        self.steer = 0
        self.throttle = 0

        self.lidar_cartesian_points = None

    def start_driving(self):
        
        while(True):
            if self.lidar_cartesian_points is None:
                continue
            target_cartesian = self.global_planner.get_target_cartesian(self.lidar_cartesian_points)
            speed = math.sqrt(self.vehicle.get_velocity().x ** 2 + self.vehicle.get_velocity().y ** 2)
            throttle = self.local_planner.throttle_control(speed)
            steer = self.local_planner.cartesian_steering_control(target_cartesian)


            self.debug_draw_cartesian(target_cartesian)
            print("steer: " + str(steer))
            control = carla.VehicleControl(throttle, steer)
            #control = carla.VehicleControl(1.0, 0)
            self.spectator.set_transform(self.camera.get_transform())
            self.vehicle.apply_control(control)
            time.sleep(1)

    def destroy(self):
        for actor in self.actor_list:
           actor.destroy()

    def attach_lidar_sensor(self):
        transform = carla.Transform(carla.Location(x=0.8, z=1.7))
        #configure LIDAR sensor to only output 2d
        # Find the blueprint of the sensor.
        lidar_bp = world.get_blueprint_library().find('sensor.lidar.ray_cast')
        # Set the time in seconds between sensor captures
        #lidar_bp.set_attribute('sensor_tick', '0.1')
        lidar_bp.set_attribute('channels', '1')
        lidar_bp.set_attribute('upper_fov', '0')
        lidar_bp.set_attribute('lower_fov', '0')
        lidar_bp.set_attribute('range', '50') #10 is default

        lidar_bp.set_attribute('points_per_second', '500')
        #With 2 channels, and 100 points per second, here are 250 points per scan

        lidar_sensor = world.spawn_actor(lidar_bp, transform, attach_to=self.vehicle)
        self.actor_list.append(lidar_sensor) #Add at actor_list[2]

        lidar_sensor.listen(lambda data: self.save_lidar_image(data, world, self.vehicle))

        return lidar_sensor

    def debug_draw_cartesian(self, cartesian_coordinate):
        #print(cartesian_coordinate)
        #Rotate into the car's frame
        transform = self.vehicle.get_transform()
        transform = [transform.location.x, transform.location.y, transform.location.z]
        vehicle_rotation = self.vehicle.get_transform().rotation
        roll = vehicle_rotation.roll
        pitch = vehicle_rotation.pitch
        yaw = vehicle_rotation.yaw + (np.pi / 2)
        R = transforms3d.euler.euler2mat(roll,pitch,yaw).T

        print(cartesian_coordinate)
        world_point = np.dot(R, cartesian_coordinate)
        world_point = np.add(transform, world_point) 
        print(world_point)

        relative_loc = carla.Location(x = world_point[0], y = world_point[1], z = 0.0)
        world.debug.draw_point(relative_loc, life_time=5)
        world.debug.draw_line(self.vehicle.get_transform().location, relative_loc, life_time=5)

        return
        

    def save_lidar_image(self, image, world, vehicle):
        global lock1

        if not lock1.acquire(False):
            return

        #Convert raw data to coordinates (x,y,z)
        points = np.frombuffer(image.raw_data, dtype=np.dtype('f4'))
        points = np.reshape(points, (int(points.shape[0] / 3), 3))
        points = points.tolist()
        #Sort the points by radius

        points.sort(key = lambda point: (np.arctan2(point[1], point[0]) * 180  / math.pi))
        points = [[-p[0], -p[1], 0] for p in points]
        #addig the negative signs FIXED THE LEFT HAND ISSUE

        self.lidar_cartesian_points = points

        #NOTE: some points have a negative angle, so sorting is seperated
        #Might be worth fixing later

        lock1.release()

    def attach_collision_sensor(self):
        transform = carla.Transform(carla.Location(x=0.8, z=1.7))
        #Configure collision sensor
        collision_bp = world.get_blueprint_library().find('sensor.other.collision')
        collision_sensor = world.spawn_actor(collision_bp, transform, attach_to=self.vehicle)
        self.actor_list.append(collision_sensor) #Add at actor_list[3]

        collision_sensor.listen(lambda data: self.destroy())

        return collision_sensor
    
    def attach_camera_sensor(self):
        global world
        global spectator

        camera_bp = world.get_blueprint_library().find('sensor.camera.rgb')
        camera_transform = carla.Transform(carla.Location(x=-10,z=10), carla.Rotation(-30,0,0))
        camera = world.spawn_actor(camera_bp, camera_transform, attach_to=self.vehicle)
        self.actor_list.append(camera) #Add to actor_list at [1]

        spectator = world.get_spectator()
        spectator.set_transform(camera.get_transform())
        return camera, spectator

def relative_location(frame, location):
    origin = frame.location
    forward = frame.get_forward_vector()
    right = frame.get_right_vector()
    up = frame.get_up_vector()
    disp = location - origin
    x = np.dot([disp.x, disp.y, disp.z], [forward.x, forward.y, forward.z])
    #x = np.clip(x, -10, 10)
    y = np.dot([disp.x, disp.y, disp.z], [right.x, right.y, right.z])
    #y = np.clip(y, -10, 10)
    z = np.dot([disp.x, disp.y, disp.z], [up.x, up.y, up.z])

    return carla.Vector3D(x, y, z)

def get_right_lane_nth(waypoint, n):
    out_waypoint = waypoint
    for i in range(n):
        out_waypoint = out_waypoint.get_right_lane()
    return out_waypoint

def get_left_lane_nth(waypoint, n):
    out_waypoint = waypoint
    for i in range(n):
        out_waypoint = out_waypoint.get_left_lane()
    return out_waypoint

def change_lane(waypoint, n):
    if (n > 0):
        return get_left_lane_nth(waypoint, n)
    else:
        return get_right_lane_nth(waypoint, n)


if __name__ == "__main__":

    try:
        main()
    except KeyboardInterrupt:
        pass
    finally:
        print('destroying actors')
        for vehicle in vehicle_list:
           vehicle.destroy()
        print('\ndone.')

