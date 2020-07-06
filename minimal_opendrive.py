import carla

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

  blueprint = world.get_blueprint_library().filter('vehicle.*model3*')[0]
  spawn_points = world.get_map().get_spawn_points() # get_spawn_points() returns list(carla.Transform)
  spawn_point = spawn_points[0]
  world.spawn_actor(blueprint, spawn_point)

if __name__ == '__main__':

  try:
      main()
  except KeyboardInterrupt:
      pass
  finally:
      print('\ndone.')
