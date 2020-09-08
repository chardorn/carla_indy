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
angle = 0
target_cartesian = None
# Order = vehicle, spectator camera, sensor1, sensor2


def control_loop(world_snapshot, vehicle, controller, spectator, camera):
    global target_cartesian
    print(target_cartesian)
    # When all actors have been spawned
    speed = math.sqrt(vehicle.get_velocity().x **
                      2 + vehicle.get_velocity().y ** 2)
    throttle = controller.throttle_control(speed)
    steer = controller.cartesian_steering_control(target_cartesian)
    print("steer: " + str(steer))
    control = carla.VehicleControl(throttle, steer)
    spectator.set_transform(camera.get_transform())
    vehicle.apply_control(control)


def main():
    global target_cartesian

    client = carla.Client('127.0.0.1', 2000)
    client.set_timeout(10.0)

    # Read the opendrive file to a string
    xodr_path = "speedway_5lanes.xodr"
    #xodr_path = "Crossing8Course.xodr"
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
    settings = world.get_settings()
    settings.synchronous_mode = True  # Enables synchronous mode
    settings.fixed_delta_seconds = 0.01  # simulation time between two frames
    world.apply_settings(settings)

    spectator = world.get_spectator()

    vehicle, camera, controller = spawn_actor(world)

    world.on_tick(lambda world_snapshot: control_loop(
        world_snapshot, vehicle, controller, spectator, camera))

    # if (target_cartesian is not None):  # wait until first Lidar scan
    tick_rate = 100.0  # number of ticks per second, assuming tick() runs in zero time
    while True:
        # time.sleep(1/tick_rate)
        world.tick()


def spawn_actor(world):
    waypoints = world.get_map().generate_waypoints(10.0)

    targetLane = -3
    waypoint = waypoints[500]
    waypoint = change_lane(waypoint, targetLane - waypoint.lane_id)

    location = waypoint.transform.location + carla.Vector3D(0, 0, 1.5)
    rotation = waypoint.transform.rotation

    # Set spawning location as initial waypoint
    blueprint = world.get_blueprint_library().filter('vehicle.*model3*')[0]
    location = waypoint.transform.location + carla.Vector3D(0, 0, 1.5)
    rotation = waypoint.transform.rotation
    vehicle = world.spawn_actor(blueprint, carla.Transform(location, rotation))
    actor_list.append(vehicle)

    transform = carla.Transform(carla.Location(x=0.8, z=1.7))

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
        carla.Location(x=-50, z=50), carla.Rotation(-30, 0, 0))
    camera = world.spawn_actor(camera_bp, camera_transform, attach_to=vehicle)
    actor_list.append(camera)  # Add to actor_list at [1]

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
    # lidar_bp.set_attribute('points_per_second', '500')
    # With 2 channels, and 100 points per second, here are 250 points per scan

    lidar_sensor = world.spawn_actor(lidar_bp, transform, attach_to=vehicle)
    actor_list.append(lidar_sensor)  # Add at actor_list[2]

    # Commented out to decrease processing
    lidar_sensor.listen(lambda data: save_lidar_image(data, world, vehicle))

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


def save_lidar_image(image, world, vehicle):
    global target_cartesian

    # Convert raw data to coordinates (x,y,z) in lidar's coords?
    points = np.frombuffer(image.raw_data, dtype=np.dtype('f4'))
    points = np.reshape(points, (int(points.shape[0] / 3), 3))
    points = points.tolist()

    lidar_transform = image.transform
    # lidar_loc = lidar_transform.location
    # forward =
    for point in points:
        loc = carla.Location(x=point[0], y=point[1], z=point[2])
        world.debug.draw_point(
            lidar_transform.transform(loc), life_time=0.01, color=carla.Color(0, 255, 255))

    # Sort the points by radius
    points.sort(key=lambda point: (
        np.arctan2(point[1], point[0]) * 180 / math.pi))
    # for point in points:
    #print((np.arctan2(point[1], point[0]) * 180  / math.pi))
    for point in points:
        point[0] = -point[0]

    # NOTE: some points have a negative angle, so sorting is seperated
    # Might be worth fixing later

    # Rotate into the car's frame
    transform = vehicle.get_transform()
    transform = [transform.location.x,
                 transform.location.y, transform.location.z]
    vehicle_rotation = vehicle.get_transform().rotation
    roll = vehicle_rotation.roll
    pitch = vehicle_rotation.pitch
    yaw = vehicle_rotation.yaw + (np.pi / 2)
    R = transforms3d.euler.euler2mat(roll, pitch, yaw).T

    left_hand_points = [[p[0], -p[1], 0] for p in points]
    world_points = [np.dot(R, point) for point in left_hand_points]
    # Move location into car's frame
    # world_points[:] = [np.add(transform, point) for point in world_points]

    index1, index2 = find_disparity(points)
    # Switched to points from world_points
    target_cartesian = points[index1]

    # graph_cartesian_points(points, points[index1], points[index2])

    relative_loc = carla.Location(
        x=world_points[index1][0], y=world_points[index1][1], z=0.0)
    world.debug.draw_point(relative_loc, life_time=0.5)
    relative_loc = carla.Location(
        x=world_points[index2][0], y=world_points[index2][1], z=0.0)
    world.debug.draw_point(relative_loc, life_time=0.5,
                           color=carla.Color(0, 255, 255))

    #print("TARGET: " + str(relative_loc))
    # for point in world_points:
    #     loc = carla.Location(x=point[0], y=point[1], z=0.0)
    #     world.debug.draw_point(
    #         loc, life_time=5, color=carla.Color(0, 255, 255))


# def graph_cartesian_points(points, target_point, target_point_2):
#     x = [point[0] for point in points]
#     y = [point[1] for point in points]

#     plt.scatter(x, y, marker='o', color='r')
#     plt.plot(target_point[0], target_point[1], marker='o', color='g')
#     plt.plot(target_point_2[0], target_point_2[1], marker='o', color='b')

#     #plt.axis([-500, 1500, -800, 100])
#     plt.style.use('seaborn-whitegrid')
#     plt.savefig("cart_plot.png")
#     plt.clf()
#     #print("plot saved")


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
        if speed >= 30:
            return 0.7
        else:
            return 1.0

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

    def cartesian_steering_control(self, cartesian_point):
        x = cartesian_point[0]
        if x < 0:
            x = x + 3
        if x > 0:
            x = x - 3
        y = cartesian_point[1] * 2.5  # Added a scale/extension of 300%
        steer_rad = math.atan(x / y)  # Changed from tan to atan
        steer_deg = np.degrees(steer_rad)
        print(steer_deg)
        steer = np.clip(steer_deg, -self.max_steer, self.max_steer)
        if abs(steer_deg) < 50:
            return steer_deg / (2 * self.max_steer)
        if abs(steer_deg) < 5:
            return 0

        return steer / self.max_steer


def find_disparity(cartesian_coordinates):
    # cartesian coordinates are in the car's reference frame
    max_distance = 0
    max_disparity_pair = [[], []]
    # The disparities are in the form of polar coordinates, so
    # the pair is in the form [[radius1, degrees1, height], [radius2, degrees2, height]
    # max_disparity_pair[0] is to the left of max_disparity_pair[1]

    for i in range(len(cartesian_coordinates) - 1):
        if cartesian_coordinates[i][1] > 0:
            x1 = cartesian_coordinates[i][0]
            y1 = cartesian_coordinates[i][1]
            x2 = cartesian_coordinates[i+1][0]
            y2 = cartesian_coordinates[i+1][1]
            distance = math.sqrt((x1-x2)**2 + (y1-y2)**2)
            if distance > max_distance and distance > 5:
                max_distance = distance
                index1 = i
                index2 = i + 1

    return index1, index2


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
        for actor in actor_list:
            actor.destroy()
        print('\ndone.')
