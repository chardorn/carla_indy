import carla

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


def main():
  client = carla.Client('127.0.0.1', 2000)
  client.set_timeout(10.0)

  world = client.get_world()

  blueprint = world.get_blueprint_library().filter('vehicle.*model3*')[0]
  waypoints = world.get_map().generate_waypoints(10.0)

  targetLane = -2
  waypoint = waypoints[0]
  waypoint = change_lane(waypoint, targetLane - waypoint.lane_id)

  location = waypoint.transform.location + carla.Vector3D(0, 0, 2)
  rotation = waypoint.transform.rotation
  world.spawn_actor(blueprint, carla.Transform(location, rotation))

if __name__ == '__main__':

  try:
      main()
  except KeyboardInterrupt:
      pass
  finally:
      print('\ndone.')
