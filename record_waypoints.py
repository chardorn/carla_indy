import carla
import random
import math
import numpy as np

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

def first_lap(world, vehicle, targetLane, max_steer, wheelbase, spectator):

    locations = []
    locations_array = []

    i = 0
    while True:
        i = i + 1
        # Find next waypoint 6 meters ahead.
        vehicle_transform = vehicle.get_transform()
        vehicle_location = vehicle_transform.location
        vehicle_waypoint = world.get_map().get_waypoint(vehicle_location, project_to_road=True, lane_type=carla.LaneType.Driving)
        waypoint = vehicle_waypoint.next(20.0)[0] #changed from 8.0 to 20.0
        world.debug.draw_point(waypoint.transform.location, life_time=0)
        waypoint = change_lane(waypoint, targetLane - waypoint.lane_id)
        throttle = 0.85
        #world.debug.draw_point(vehicle_location, life_time=0)
        steer = control_pure_pursuit(vehicle_transform, waypoint.transform, max_steer, wheelbase)
        control = carla.VehicleControl(throttle, steer)

        vehicle.apply_control(control)
        if i%10 == 0:
            locations.append(vehicle_location)
            locations_array.append([vehicle_location.x, vehicle_location.y])
            print(vehicle_transform.rotation)
            print(vehicle_location)

        spectator.set_transform(get_transform(vehicle.get_location(), 145))
    
    return locations_array


    

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
    wall_height = 1.0      # in meters
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

    locations = first_lap(world, vehicle, targetLane, max_steer, wheelbase, spectator)
    print(locations)
    #np.savetxt("waypoints.csv", np_locations, delimiter=",")






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
