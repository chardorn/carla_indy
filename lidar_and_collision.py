import carla
import random
import math
import numpy as np
import time
import transforms3d

actor_list = []

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

def control_pid(vehicle_transform, waypoint_transform):
  steer = 0
  steer += (waypoint_transform.rotation.yaw - vehicle_transform.rotation.yaw)*0.2
  return steer

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

def control_pure_pursuit(vehicle_tr, waypoint_tr, max_steer, wheelbase):
  # TODO: convert vehicle transform to rear axle transform
  wp_loc_rel = relative_location(vehicle_tr, waypoint_tr.location) + carla.Vector3D(wheelbase, 0, 0)
  wp_ar = [wp_loc_rel.x, wp_loc_rel.y]
  d2 = wp_ar[0]**2 + wp_ar[1]**2
  steer_rad = math.atan(2 * wheelbase * wp_loc_rel.y / d2)
  steer_deg = math.degrees(steer_rad)
  steer_deg = np.clip(steer_deg, -max_steer, max_steer)
  return steer_deg / max_steer

def get_transform(vehicle_location, angle, d=6.4):
    a = math.radians(angle)
    location = carla.Location(d * math.cos(a), d * math.sin(a), 2.0) + vehicle_location
    return carla.Transform(location, carla.Rotation(yaw=180 + angle, pitch=-15))

def save_lidar_image(image, world, vehicle):

    #Convert raw data to coordinates (x,y,z)
    points = np.frombuffer(image.raw_data, dtype=np.dtype('f4'))
    points = np.reshape(points, (int(points.shape[0] / 3), 3))
    transform = vehicle.get_transform()
    transform = [transform.location.x, transform.location.y, transform.location.z]
    #print(transform)

    for point in points:

        #Rotate into the car's frame
        vehicle_rotation = vehicle.get_transform().rotation
        roll = vehicle_rotation.roll 
        pitch = vehicle_rotation.pitch
        yaw = vehicle_rotation.yaw + (np.pi / 2)
        R = transforms3d.euler.euler2mat(roll,pitch,yaw).T
        point = np.dot(R, point)

        #Move location into car's frame
        point = np.add(transform, point)
        location = carla.Location(x=float(point[0]), y=float(point[1]), z=float(point[2]))

        #Draw in world
        world.debug.draw_point(location, life_time=0)


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
    max_road_length = 50.0 # in meters
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

    blueprint = world.get_blueprint_library().filter('vehicle.*model3*')[0]
    waypoints = world.get_map().generate_waypoints(2.0)

    targetLane = -3
    waypoint = waypoints[0]
    waypoint = change_lane(waypoint, targetLane - waypoint.lane_id)

    location = waypoint.transform.location + carla.Vector3D(0, 0, 1.5)
    rotation = waypoint.transform.rotation

    print(location)
    print(rotation)


    vehicle = world.spawn_actor(blueprint, carla.Transform(location, rotation))
    actor_list.append(vehicle) #Add actor to list in order to destroy when program completes
    vehicle.set_simulate_physics(False)
    physics_control = vehicle.get_physics_control()
    max_steer = physics_control.wheels[0].max_steer_angle
    rear_axle_center = (physics_control.wheels[2].position + physics_control.wheels[3].position)/200
    offset = rear_axle_center - vehicle.get_location()
    wheelbase = np.linalg.norm([offset.x, offset.y, offset.z])
    vehicle.set_simulate_physics(True)

 
    transform = carla.Transform(carla.Location(x=0.8, z=1.7))

    #Configure collision sensor
    collision_bp = world.get_blueprint_library().find('sensor.other.collision')
    collision_sensor = world.spawn_actor(collision_bp, transform, attach_to=vehicle)
    actor_list.append(collision_sensor)



    #configure LIDAR sensor to only output 2d
    # Find the blueprint of the sensor.
    lidar_bp = world.get_blueprint_library().find('sensor.lidar.ray_cast')
    # Set the time in seconds between sensor captures
    lidar_bp.set_attribute('sensor_tick', '0.1')
    lidar_bp.set_attribute('channels', '1')
    lidar_bp.set_attribute('upper_fov', '0')
    #lidar_bp.set_attribute('lower_fov', '0')
    lidar_bp.set_attribute('range', '10') #10 is default

    lidar_bp.set_attribute('points_per_second', '500')
    #With 2 channels, and 100 points per second, here are 250 points per scan


    lidar_sensor = world.spawn_actor(lidar_bp, transform, attach_to=vehicle)
    actor_list.append(lidar_sensor)
    lidar_sensor.listen(lambda data: save_lidar_image(data, world, vehicle))

    throttle = 0.85
    control = carla.VehicleControl(throttle, 0)
    vehicle.apply_control(control)

    while True:
        spectator.set_transform(get_transform(vehicle.get_location(), 145))







if __name__ == '__main__':

  try:
      main()
  except KeyboardInterrupt:
      pass
  finally:
    print('destroying actors')
    for actor in actor_list:
        actor.destroy()
    print('\ndone.')
