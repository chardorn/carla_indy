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
# lock1 = threading.Lock()
# lock = threading.Lock()



def main():
    global world
    global vehicle_list
    # global lock

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
    
    # world = client.get_world()
    settings = world.get_settings()
    settings.synchronous_mode = True  # Enables synchronous mode
    settings.fixed_delta_seconds = 0.1  # simulation time between two frames
    world.apply_settings(settings)

    tick_rate = 10.0  # number of ticks per second, assuming tick() runs in zero time


    transform = waypoints = world.get_map().generate_waypoints(10.0)
    #Change targetLane and waypoint index depending on where you want the vehicle to spawn
    targetLane = -3 
    waypoint = waypoints[500] 
    waypoint = change_lane(waypoint, targetLane - waypoint.lane_id)
    location = waypoint.transform.location + carla.Vector3D(0, 0, 0.5)
    rotation = waypoint.transform.rotation
    transform = carla.Transform(location, rotation)

    vehicle_1 = Autonomous_Vehicle(['lidar', 'collision', 'camera'], transform, Disparity_Extender(), Pure_Pursuit_Controller())
    vehicle_list.append(vehicle_1)

    world.on_tick(lambda world_snapshot: vehicle_1.control_loop())

    while True:
        time.sleep(1/tick_rate)
        world.tick()


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
        y = cartesian_point[0]
        x = cartesian_point[1] 

        steer_rad = np.arctan2(x, y) #Changed from tan to atan
        steer_deg = np.degrees(steer_rad)
        steer = np.clip(steer_deg, -self.max_steer, self.max_steer)
        if abs(steer_deg) < 50:
            return steer_deg / (2 * self.max_steer)
        if abs(steer_deg) < 5:
            return 0

        return steer / self.max_steer

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
            x1 = cartesian_coordinates[i][0]
            y1 =  cartesian_coordinates[i][1]
            x2 = cartesian_coordinates[i+1][0]
            y2 =  cartesian_coordinates[i+1][1]
            distance = math.sqrt((x1-x2)**2 + (y1-y2)**2)
            if distance > max_distance:
                max_distance = distance
                index1 = i #Left point
                index2 = i + 1 #Right point


        extnded_cartesian = self.extend_disparity(cartesian_coordinates[index1])
        return extnded_cartesian

    def extend_disparity(self, cc):
        distance_to_extend = 2.5
        extended_cc = [cc[0], cc[1], cc[2]]
        if(cc[1] > 0):
            extended_cc[1] = cc[1] - distance_to_extend
        
        else:
            extended_cc[1] = cc[1] + distance_to_extend
        
        return extended_cc


class Autonomous_Vehicle(object):

    def __init__(self, sensor_list, transform, global_planner, local_planner):
        global world
        global spectator
        global vehicle_list

        self.graph_lidar = True
        self.graph_target = True

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
        self.lidar_transform = None

    def control_loop(self):
        
        if self.lidar_cartesian_points is None:
            return

        settings = world.get_settings()
        t_step = settings.fixed_delta_seconds

        if self.graph_lidar:
            self.debug_draw_cartesians(self.lidar_cartesian_points, t_step)

        target_cartesian = self.global_planner.get_target_cartesian(self.lidar_cartesian_points)

        if self.graph_target:
            loc = carla.Location(x=target_cartesian[0], y=target_cartesian[1], z=target_cartesian[2])
            target = self.lidar_transform.transform(loc)

        #print(self.lidar_transform.location)
        #world.debug.draw_point(target, life_time=t_step, color=carla.Color(255, 0, 0))
        world.debug.draw_line(self.lidar_transform.location, target, life_time=t_step, color=carla.Color(255, 0, 0))

        speed = math.sqrt(self.vehicle.get_velocity().x ** 2 + self.vehicle.get_velocity().y ** 2)
        throttle = self.local_planner.throttle_control(speed)
        steer = self.local_planner.cartesian_steering_control(target_cartesian)
        print(target_cartesian)
        print(steer)

        # print("steer: " + str(steer))
        control = carla.VehicleControl(throttle, steer)
        self.spectator.set_transform(self.camera.get_transform())
        self.vehicle.apply_control(control)

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
        lidar_bp.set_attribute('range', '50')  # 10 is default
        lidar_bp.set_attribute('rotation_frequency', '100')

        #lidar_bp.set_attribute('points_per_second', '500')
        #With 2 channels, and 100 points per second, here are 250 points per scan

        lidar_sensor = world.spawn_actor(lidar_bp, transform, attach_to=self.vehicle)
        self.actor_list.append(lidar_sensor)

        lidar_sensor.listen(lambda data: self.save_lidar_image(data, world, self.vehicle))

        return lidar_sensor

    def debug_draw_cartesians(self, cartesian_coordinates, t_step):



        for coordinate in cartesian_coordinates:
            loc = carla.Location(x=coordinate[0], y=coordinate[1], z=coordinate[2])
            target = self.lidar_transform.transform(loc)
            world.debug.draw_point(target, life_time=t_step, color=carla.Color(0, 255, 255))
            #world.debug.draw_line(self.lidar_transform.location, target, life_time=t_step, color=carla.Color(255, 0, 0))


        return
        

    def save_lidar_image(self, image, world, vehicle):

        #Convert raw data to coordinates (x,y,z)
        points = np.frombuffer(image.raw_data, dtype=np.dtype('f4'))
        points = np.reshape(points, (int(points.shape[0] / 3), 3))

        
        #Sort the points by radius
        
        #Rotate 90 degrees clockwise
        points = np.matmul(points, np.array([[0, 1, 0], [-1, 0, 0], [0, 0, 1]]))
        points = points.tolist()

        points.sort(key=lambda point: np.arctan2(point[1], point[0]))

        # print(len(points))
        start_index = int(len(points) / 4)
        end_index = int(len(points) * 3 / 4)

        points = points[start_index:end_index]

        self.lidar_transform = image.transform 

        self.lidar_cartesian_points = points

        
        return

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

