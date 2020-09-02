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

def main():
    global world

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

    vehicle_1 = Autonomous_Vehicle(['lidar', 'collision', 'camera'], transform)

    time.sleep(5)





class Autonomous_Vehicle(object):
    def __init__(self, sensor_list, transform):
        global world
        global spectator
        blueprint = world.get_blueprint_library().filter('vehicle.*model3*')[0]
        self.vehicle = world.spawn_actor(blueprint, transform)
        self.actor_list = []

        #Vehicle properties setup
        physics_control = self.vehicle.get_physics_control()
        max_steer = physics_control.wheels[0].max_steer_angle
        rear_axle_center = (physics_control.wheels[2].position + physics_control.wheels[3].position)/200
        offset = rear_axle_center - self.vehicle.get_location()
        wheelbase = np.linalg.norm([offset.x, offset.y, offset.z])
        self.vehicle.set_simulate_physics(True)

        if 'lidar' in sensor_list:
            self.attach_lidar_sensor()
        if 'collision' in sensor_list:
            self.attach_collision_sensor()
        if 'camera' in sensor_list:
            self.attach_camera_sensor()

    def destroy():
        for actor in self.actor_list:
           actor.destroy()

    def attach_lidar_sensor(self):
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

        lidar_sensor = world.spawn_actor(lidar_bp, transform, attach_to=vehicle)
        self.actor_list.append(lidar_sensor) #Add at actor_list[2]

        return lidar_sensor

    def attach_collision_sensor(self):
        #Configure collision sensor
        collision_bp = world.get_blueprint_library().find('sensor.other.collision')
        collision_sensor = world.spawn_actor(collision_bp, transform, attach_to=vehicle)
        actor_list.append(collision_sensor) #Add at actor_list[3]

        collision_sensor.listen(lambda data: collision_event(data, world, vehicle))

        return collision_sensor
    
    def attach_camera_sensor(self):
        global world
        global spectator

        camera_bp = world.get_blueprint_library().find('sensor.camera.rgb')
        camera_transform = carla.Transform(carla.Location(x=-10,z=10), carla.Rotation(-30,0,0))
        camera = world.spawn_actor(camera_bp, camera_transform, attach_to=self.vehicle)
        actor_list.append(camera) #Add to actor_list at [1]

        spectator = world.get_spectator()
        spectator.set_transform(camera.get_transform())
        return camera, spectator
        
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
        for actor in actor_list:
           actor.destroy()
        print('\ndone.')

