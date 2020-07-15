import carla
import random
import math
import numpy as np
import time
import transforms3d
import sklearn
from sklearn.cluster import KMeans

actor_list = [] 
waypoint = carla.Location()
#Order = vehicle, spectator camera, sensor1, sensor2

def main():
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
    
    spectator = world.get_spectator()

    spawn_actor(world)

    while True:
        #When all actors have been spawned
        if(len(actor_list) >= 4):
            control_vehicle(actor_list[0])
            camera = actor_list[1]
            spectator.set_transform(camera.get_transform())
        timestamp = world.wait_for_tick()
        delta_seconds = timestamp.delta_seconds
        time.sleep(delta_seconds)

def control_vehicle(vehicle):
    global waypoint
    physics_control = vehicle.get_physics_control()
    max_steer = physics_control.wheels[0].max_steer_angle
    rear_axle_center = (physics_control.wheels[2].position + physics_control.wheels[3].position)/200
    offset = rear_axle_center - vehicle.get_location()
    wheelbase = np.linalg.norm([offset.x, offset.y, offset.z])
    throttle = 0.5
    vehicle_transform = vehicle.get_transform()
    waypoint_location = carla.Location(waypoint[0], waypoint[1], 0)
    steer = control_pure_pursuit(vehicle_transform, waypoint_location, max_steer, wheelbase)
    print("steer")
    print(steer)
    control = carla.VehicleControl(throttle, steer)
    vehicle.apply_control(control)



def spawn_actor(world):
    blueprint = world.get_blueprint_library().filter('vehicle.*model3*')[0]
    waypoints = world.get_map().generate_waypoints(2.0)

    targetLane = -3
    waypoint = waypoints[0]
    waypoint = change_lane(waypoint, targetLane - waypoint.lane_id)

    location = waypoint.transform.location + carla.Vector3D(0, 0, 1.5)
    rotation = waypoint.transform.rotation

    vehicle = world.spawn_actor(blueprint, carla.Transform(location, rotation))
    actor_list.append(vehicle)
    vehicle.set_simulate_physics(True)
    transform = carla.Transform(carla.Location(x=0.8, z=1.7))

    #Add spectator camera
    camera_bp = world.get_blueprint_library().find('sensor.camera.rgb')
    camera_transform = carla.Transform(carla.Location(x=-10,z=10), carla.Rotation(-45,0,0))
    camera = world.spawn_actor(camera_bp, camera_transform, attach_to=vehicle)
    actor_list.append(camera) #Add to actor_list at [1]

    attach_lidar(world, vehicle, transform)
    attach_collision_sensor(world, vehicle, transform)

    return vehicle, camera

def attach_lidar(world, vehicle, transform):
    #configure LIDAR sensor to only output 2d
    # Find the blueprint of the sensor.
    lidar_bp = world.get_blueprint_library().find('sensor.lidar.ray_cast')
    # Set the time in seconds between sensor captures
    #lidar_bp.set_attribute('sensor_tick', '0.1')
    lidar_bp.set_attribute('channels', '1')
    lidar_bp.set_attribute('upper_fov', '0')
    lidar_bp.set_attribute('lower_fov', '0')
    lidar_bp.set_attribute('range', '30') #10 is default

    lidar_bp.set_attribute('points_per_second', '10000')
    #With 2 channels, and 100 points per second, here are 250 points per scan


    lidar_sensor = world.spawn_actor(lidar_bp, transform, attach_to=vehicle)
    actor_list.append(lidar_sensor) #Add at actor_list[2]

    #Commented out to decrease processing
    lidar_sensor.listen(lambda data: save_lidar_image(data, world, vehicle))

    return lidar_sensor



def attach_collision_sensor(world, vehicle, transform):
    #Configure collision sensor
    collision_bp = world.get_blueprint_library().find('sensor.other.collision')
    collision_sensor = world.spawn_actor(collision_bp, transform, attach_to=vehicle)
    actor_list.append(collision_sensor) #Add at actor_list[3]

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

    
#Returns array of flattened, 2D points
def filter_scan(points):
    #Removes 3rd dimension of points
    filtered_points = np.array(points[:, :2])

    kmeans = KMeans(n_clusters = 2)
    kmeans.fit(filtered_points)
    y_kmeans = kmeans.predict(filtered_points)

    cluster1 = []
    cluster2 = []

    for i in range(len(points)):
        if y_kmeans[i] == 0:
            cluster1.append(points[i])
        else:
            cluster2.append(points[i])

    if (np.mean(np.array(cluster1[:])) < np.mean(np.array(cluster2[:]))):
        left_cluster = cluster1
        right_cluster = cluster2
    else:
        left_cluster = cluster2
        right_cluster = cluster1

    left_center = get_center_cluster(left_cluster)
    print("left center" + str(left_center))
    right_center = get_center_cluster(right_cluster)
    print("right center" + str(right_center))


    return left_cluster, right_cluster, left_center, right_center

def get_center_cluster(cluster):

    x = [p[0] for p in cluster]
    y = [p[1] for p in cluster]

    return (sum(x) / len(cluster), sum(y) / len(cluster))

def get_front_points(cluster, vehicle_location):
    front_cluster = []

    for point in cluster:
         if (carla.Location(float(point[0]), float(point[1]), vehicle_location.z) - vehicle_location).x > 0:
            front_cluster.append(point)

    return front_cluster

def waypoint_from_centers(left_center, right_center):
    middle_point = (((left_center[0] + right_center[0]) / 2) + 3, (left_center[1] + right_center[1]) / 2)

    return middle_point

def control_pure_pursuit(vehicle_tr, waypoint_location, max_steer, wheelbase):
  # TODO: convert vehicle transform to rear axle transform
  print("waypoint: " + str(waypoint_location))
  print("vehicle transform: " + str(vehicle_tr))

  wp_loc_rel = relative_location(vehicle_tr, waypoint_location) + carla.Vector3D(wheelbase, 0, 0)
  print("relative location")
  print(wp_loc_rel)
  wp_ar = [wp_loc_rel.x, wp_loc_rel.y]
  d2 = wp_ar[0]**2 + wp_ar[1]**2
  steer_rad = math.atan(2 * wheelbase * wp_loc_rel.y / d2)
  steer_deg = math.degrees(steer_rad)
  steer_deg = np.clip(steer_deg, -max_steer, max_steer)
  return steer_deg / max_steer
    
def save_lidar_image(image, world, vehicle):
    global waypoint
    #Convert raw data to coordinates (x,y,z)
    print("SCAN:")
    points = np.frombuffer(image.raw_data, dtype=np.dtype('f4'))
    print(image.horizontal_angle)
    print(points.shape)
    points = np.reshape(points, (int(points.shape[0] / 3), 3))
    #reshape from list of 4d points to an array of 3d points 
    transform = vehicle.get_transform()
    transform = [transform.location.x, transform.location.y, transform.location.z]

    #Rotate into the car's frame
    vehicle_rotation = vehicle.get_transform().rotation
    roll = vehicle_rotation.roll 
    pitch = vehicle_rotation.pitch
    yaw = vehicle_rotation.yaw + (np.pi / 2)
    R = transforms3d.euler.euler2mat(roll,pitch,yaw).T

    points = points.copy()

    points[:] = [np.dot(R, point) for point in points]
    points[:] = [np.add(transform, point)  for point in points]       #Move location into car's frame

    left_cluster, right_cluster, left_center, right_center = filter_scan(points)

    left = carla.Location(x=float(left_center[0]), y=float(left_center[1]), z=float(0))
    world.debug.draw_point(left, life_time=1, color = carla.Color(0, 255, 255))

    right = carla.Location(x=float(right_center[0]), y=float(right_center[1]), z=float(0))
    world.debug.draw_point(left, life_time=1, color = carla.Color(0, 255, 255))


    left_front= get_front_points(left_cluster, vehicle.get_transform().location)
    right_front = get_front_points(right_cluster, vehicle.get_transform().location)

    waypoint = waypoint_from_centers(get_center_cluster(left_front), get_center_cluster(right_front))

    left_location = carla.Location(x=float(waypoint[0]), y=float(waypoint[1]), z=float(0))
    world.debug.draw_point(left_location, life_time=1, color = carla.Color(0, 255, 255))

    #for point in left_front:
    #    left = carla.Location(x=float(point[0]), y=float(point[1]), z=float(0))
    #    world.debug.draw_point(left, life_time=1, color = carla.Color(0, 255, 255))

    #for point in right_front:
    #    right = carla.Location(x=float(point[0]), y=float(point[1]), z=float(0))
    #    world.debug.draw_point(right, life_time=1, color = carla.Color(0, 255, 255))

    
    
    

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

def relative_location(frame, location):
    origin = frame.location
    forward = frame.get_forward_vector()
    right = frame.get_right_vector()
    up = frame.get_up_vector()
    disp = location - origin
    x = np.dot([disp.x, disp.y, disp.z], [forward.x, forward.y, forward.z])
    y = np.dot([disp.x, disp.y, disp.z], [right.x, right.y, right.z])
    z = np.dot([disp.x, disp.y, disp.z], [up.x, up.y, up.z])
    return carla.Vector3D(x, y, z)

def get_transform(vehicle_location, angle, d=6.4):
        a = math.radians(angle)
        location = carla.Location(d * math.cos(a), d * math.sin(a), 2.0) + vehicle_location
        return carla.Transform(location, carla.Rotation(yaw=180 + angle, pitch=-15))

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