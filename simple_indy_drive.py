import carla
import numpy as np
import time
import cv2

actor_list = []
IM_WIDTH = 640
IM_HEIGHT = 480

def process_img(image, l_images):
    i = np.array(image.raw_data) 
    # print(i.shape)
    i2 = i.reshape((IM_HEIGHT, IM_WIDTH, 4)) #rgba, a for alpha
    i3 = i2[:, :, :3] # /255.0 # entire height, entire width, only rgb (no alpha)
    print(i3[1 , 1, :])
    #import pdb; pdb.set_trace()
    #cv2.imshow("image", i3)
    #cv2.waitKey(0)
    l_images.append(i3)
    return #i3/255.0 # normalize the data


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

  blueprint_library = world.get_blueprint_library()

  blueprint = blueprint_library.filter('vehicle.*model3*')[0]
  spawn_points = world.get_map().get_spawn_points() # get_spawn_points() returns list(carla.Transform)
  spawn_point = spawn_points[0]
  #world.spawn_actor(blueprint, spawn_point)
  vehicle = world.spawn_actor(blueprint, spawn_point)
  vehicle.apply_control(carla.VehicleControl(throttle=1.0, steer=0.0))
  actor_list.append(vehicle)

  

  cam_bp = blueprint_library.find("sensor.camera.rgb")
  cam_bp.set_attribute("image_size_x", f"{IM_WIDTH}")
  cam_bp.set_attribute("image_size_y", f"{IM_HEIGHT}")
  cam_bp.set_attribute("fov", "110")

  spawn_point = carla.Transform(carla.Location(x=2.5, z=0.7))
  sensor = world.spawn_actor(cam_bp, spawn_point, attach_to=vehicle)
  actor_list.append(sensor)
  
  
  #sensor.listen(lambda data: process_img(data))
  l_images = []
  sensor.listen(lambda data: process_img(data, l_images))


  time.sleep(1)
    
  sensor.stop()
    
  for im in l_images:
      cv2.imshow('image', im)
      cv2.waitKey(16)
  
  #dir() - to see all the attributes?

  for i in range(10):
    location = vehicle.get_location()
    print(location)
    time.sleep(1)





if __name__ == '__main__':

  try:
      main()
  except KeyboardInterrupt:
    pass
  finally:
    for actor in actor_list:
      actor.destroy()
    print('\ndone.')
