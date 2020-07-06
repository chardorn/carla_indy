import carla
import random
import math
import numpy as np

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

def main():
  client = carla.Client('127.0.0.1', 2000)
  client.set_timeout(10.0)

  world = client.get_world()

  blueprint = world.get_blueprint_library().filter('vehicle.*model3*')[0]
  waypoints = world.get_map().generate_waypoints(2.0)

  targetLane = -3
  waypoint = waypoints[0]
  waypoint = change_lane(waypoint, targetLane - waypoint.lane_id)

  location = waypoint.transform.location + carla.Vector3D(0, 0, 1.5)
  rotation = waypoint.transform.rotation
  vehicle = world.spawn_actor(blueprint, carla.Transform(location, rotation))

  vehicle.set_simulate_physics(False)
  physics_control = vehicle.get_physics_control()
  max_steer = physics_control.wheels[0].max_steer_angle
  rear_axle_center = (physics_control.wheels[2].position + physics_control.wheels[3].position)/200
  offset = rear_axle_center - vehicle.get_location()
  wheelbase = np.linalg.norm([offset.x, offset.y, offset.z])
  vehicle.set_simulate_physics(True)

  while True:
    # Find next waypoint 6 meters ahead.
    vehicle_transform = vehicle.get_transform()
    vehicle_location = vehicle_transform.location
    vehicle_waypoint = world.get_map().get_waypoint(vehicle_location, project_to_road=True, lane_type=carla.LaneType.Driving)
    waypoint = vehicle_waypoint.next(8.0)[0]
    waypoint = change_lane(waypoint, targetLane - waypoint.lane_id)
    throttle = 0.85
    world.debug.draw_point(vehicle_location, life_time=0)
    steer = control_pure_pursuit(vehicle_transform, waypoint.transform, max_steer, wheelbase)
    control = carla.VehicleControl(throttle, steer)

    vehicle.apply_control(control)

if __name__ == '__main__':

  try:
      main()
  except KeyboardInterrupt:
      pass
  finally:
      print('\ndone.')
