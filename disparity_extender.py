import carla
import random
import math
import numpy as np
import time
import transforms3d
import sklearn
from sklearn.cluster import KMeans
# import matplotlib.pyplot as plt

actor_list = []
waypoint = carla.Location()
target_cartesian = None
# Order = vehicle, spectator camera, sensor1, sensor2


def control_loop(world_snapshot, vehicle, controller, spectator, camera):
    global target_cartesian

    spectator.set_transform(camera.get_transform())

    if (target_cartesian is None):
        print("Target not calculated yet.")
        return

    # Calculate the control command based on target_cartesian
    speed = math.sqrt(vehicle.get_velocity().x **
                      2 + vehicle.get_velocity().y ** 2)
    throttle = controller.throttle_control(speed)
    steer = controller.cartesian_steering_control(target_cartesian)
    print("steer: " + str(steer))
    control = carla.VehicleControl(throttle, steer)
    vehicle.apply_control(control)


def main():
    global target_cartesian

    client = carla.Client('127.0.0.1', 2000)
    client.set_timeout(10.0)

    # Read the opendrive file to a string
    xodr_path = "speedway_5lanes.xodr"
    # xodr_path = "Crossing8Course.xodr"
    od_file = open(xodr_path)
    data = od_file.read()

    # Load the opendrive map
    vertex_distance = 2.0  # in meters
    max_road_length = 10.0  # in meters #Changed from 50.0
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
    settings.fixed_delta_seconds = 0.01  # simulation time between two frames
    world.apply_settings(settings)

    spectator = world.get_spectator()

    vehicle, camera, controller = spawn_actor(world)

    world.on_tick(lambda world_snapshot: control_loop(
        world_snapshot, vehicle, controller, spectator, camera))

    tick_rate = 100.0  # number of ticks per second, assuming tick() runs in zero time
    while True:
        time.sleep(1/tick_rate)
        world.tick()


def spawn_actor(world):
    waypoints = world.get_map().generate_waypoints(10.0)

    targetLane = -3
    waypoint = waypoints[500]
    waypoint = change_lane(waypoint, targetLane - waypoint.lane_id)

    # Set spawning location as initial waypoint
    blueprint = world.get_blueprint_library().filter('vehicle.*model3*')[0]
    location = waypoint.transform.location + carla.Vector3D(0, 0, 0.5)
    rotation = waypoint.transform.rotation
    vehicle = world.spawn_actor(blueprint, carla.Transform(location, rotation))
    actor_list.append(vehicle)

    # Vehicle properties setup
    physics_control = vehicle.get_physics_control()
    max_steer = physics_control.wheels[0].max_steer_angle
    rear_axle_center = (
        physics_control.wheels[2].position + physics_control.wheels[3].position)/200
    offset = rear_axle_center - vehicle.get_location()
    wheelbase = np.linalg.norm([offset.x, offset.y, offset.z])
    vehicle.set_simulate_physics(True)

    # Add spectator camera
    camera_bp = world.get_blueprint_library().find('sensor.camera.rgb')
    camera_transform = carla.Transform(
        carla.Location(x=-10, z=10), carla.Rotation(-20, 0, 0))
    camera = world.spawn_actor(camera_bp, camera_transform, attach_to=vehicle)
    actor_list.append(camera)  # Add to actor_list at [1]

    transform = carla.Transform(carla.Location(x=0.8, z=2))
    attach_lidar(world, vehicle, transform)
    attach_collision_sensor(world, vehicle, transform)

    controller = Pure_Pursuit_Controller(max_steer, wheelbase, world)

    return vehicle, camera, controller


def attach_lidar(world, vehicle, transform):
    # configure LIDAR sensor to only output 2d
    # Find the blueprint of the sensor.
    lidar_bp = world.get_blueprint_library().find('sensor.lidar.ray_cast')
    # Set the time in seconds between sensor captures
    # lidar_bp.set_attribute('sensor_tick', '0.1')
    lidar_bp.set_attribute('channels', '1')
    lidar_bp.set_attribute('upper_fov', '0')
    lidar_bp.set_attribute('lower_fov', '0')
    lidar_bp.set_attribute('range', '50')  # 10 is default
    lidar_bp.set_attribute('rotation_frequency', '100')
    # lidar_bp.set_attribute('dropoff_general_rate', '0')
    # lidar_bp.set_attribute('dropoff_zero_intensity', '0')

    lidar_sensor = world.spawn_actor(lidar_bp, transform, attach_to=vehicle)
    actor_list.append(lidar_sensor)  # Add at actor_list[2]

    # Commented out to decrease processing
    lidar_sensor.listen(lambda data: process_lidar_data(
        data, world, vehicle))

    return lidar_sensor


def attach_collision_sensor(world, vehicle, transform):
    # Configure collision sensor
    collision_bp = world.get_blueprint_library().find('sensor.other.collision')
    collision_sensor = world.spawn_actor(
        collision_bp, transform, attach_to=vehicle)
    actor_list.append(collision_sensor)  # Add at actor_list[3]

    collision_sensor.listen(lambda data: collision_event(data, world, vehicle))

    return collision_sensor


def collision_event(data, world, vehicle):
    print("COLLISION")

    a = len(actor_list)
    for b in range(a):
        actor_list[a-1].destroy()
        actor_list.pop(a-1)
        print(actor_list)
        a = a-1

    spawn_actor(world)
    time.sleep(1)


def process_lidar_data(image, world, vehicle):
    global target_cartesian

    settings = world.get_settings()
    t_step = settings.fixed_delta_seconds

    # Convert raw data to coordinates (x,y,z) in lidar's coords?
    points = np.frombuffer(image.raw_data, dtype=np.dtype('f4'))
    points = np.reshape(points, (int(points.shape[0] / 3), 3))

    # To compensate for bug in carla 0.9.9.4:
    # (x, y, z) to (-y, x, z) i.e. rotate 90 degrees counterclockwise
    points = np.matmul(points, np.array([[0, 1, 0], [-1, 0, 0], [0, 0, 1]]))

    # For easy enumeration and sorting.
    points = points.tolist()
    # Sort the points by angle
    points.sort(key=lambda point: np.arctan2(point[1], point[0]))

    lidar_transform = image.transform
    for index in range(len(points)-1):
        point = points[index]
        loc = carla.Location(x=point[0], y=point[1], z=point[2])
        intensity = int(index/2)
        world.debug.draw_point(
            lidar_transform.transform(loc), life_time=t_step, color=carla.Color(0, intensity, intensity))

    index1, index2, found = find_disparity(points)
    if (not found):
        print("No disparity found.")
        return

    # visualize the disparity points
    point1 = points[index1]
    point2 = points[index2]
    loc = carla.Location(x=point1[0], y=point1[1], z=point1[2])
    world.debug.draw_point(
        lidar_transform.transform(loc), size=0.2, life_time=t_step, color=carla.Color(255, 0, 0))
    loc = carla.Location(x=point2[0], y=point2[1], z=point2[2])
    world.debug.draw_point(
        lidar_transform.transform(loc), size=0.2, life_time=t_step, color=carla.Color(255, 0, 0))

    # TODO: change from lidar to vehicle's coordinates
    target_cartesian = [(point1[0]+point2[0])*0.5,
                        (point1[1]+point2[1])*0.5, (point1[2]+point2[2])*0.5]
    loc = carla.Location(
        x=target_cartesian[0], y=target_cartesian[1], z=target_cartesian[2])
    world.debug.draw_point(
        lidar_transform.transform(loc), size=0.2, life_time=t_step, color=carla.Color(0, 255, 0))


class Pure_Pursuit_Controller():
    def __init__(self, max_steer, wheelbase, world):  # TODO: Delete world??
        self.KP = 0.2
        self.KD = 0.1
        self.past_error = 0
        self.error = 0
        self.sum_error = 0
        self.wheelbase = wheelbase
        self.max_steer = max_steer

    def throttle_control(self, speed):
        return 3.0
        # if speed >= 30:
        #     return 0.7
        # else:
        #     return 1.0

    def steering_control(self, vehicle_tr, waypoint_tr):
        wp_loc_rel = relative_location(
            vehicle_tr, waypoint_tr.location) + carla.Vector3D(self.wheelbase, 0, 0)
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

    # The point coordinates are in vehicle's frame.
    def cartesian_steering_control(self, cartesian_point):
        x = cartesian_point[0]
        y = cartesian_point[1]
        steer_rad = math.atan(y / x)
        steer_deg = np.degrees(steer_rad)
        steer = np.clip(steer_deg, -self.max_steer, self.max_steer)

        return steer / self.max_steer


"""Find the biggest disparity

The given points are sorted by their angle around the down (up) vector in the lidar's frame.
"""


def find_disparity(points):
    # Default value is in the middle of the array which should be in front of the car.
    index1 = 0
    index2 = 0
    found = False

    min_d2 = 25  # A disparity must be at least 5 meters
    max_d2 = 0
    for i in range(len(points) - 1):
        if points[i][0] > 0:  # Ignore points behind lidar
            x1 = points[i][0]
            y1 = points[i][1]
            x2 = points[i+1][0]
            y2 = points[i+1][1]
            d2 = (x1-x2)**2 + (y1-y2)**2
            if d2 > max_d2 and d2 > min_d2:  # Found a better disparity
                found = True
                max_d2 = d2
                index1 = i
                index2 = i + 1

    return index1, index2, found


"""Change coordinates

Given global coordinates of a location, and a target frame,
find the the coordinates of the location in the target frame.
The frame is given as a transform in the global frame.
"""


def relative_location(frame, location):
    origin = frame.location
    forward = frame.get_forward_vector()
    right = frame.get_right_vector()
    up = frame.get_up_vector()
    disp = location - origin
    x = np.dot([disp.x, disp.y, disp.z], [forward.x, forward.y, forward.z])
    # x = np.clip(x, -10, 10)
    y = np.dot([disp.x, disp.y, disp.z], [right.x, right.y, right.z])
    # y = np.clip(y, -10, 10)
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
        for actor in actor_list:
            actor.destroy()
        print('\ndone.')
