import carla
import matplotlib.pyplot as plt
import numpy as np
import math

def main():

    ##Modifiable Variables
    targetLane = -3

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

    map = world.get_map()
    waypoint_list = map.generate_waypoints(40)

    print("Length: " + str(len(waypoint_list)))
    
    #Take only the waypoints from the targetLane
    waypoints = single_lane(waypoint_list, targetLane)

    #Remove all unneccesary waypoints along the straights
    curvy_waypoints = get_curvy_waypoints(waypoints)

    #Save graph of plotted points as bezier.png
    x = [p.transform.location.x for p in curvy_waypoints]
    y = [p.transform.location.y for p in curvy_waypoints]
    plt.plot(x, y, marker = 'o')
    plt.savefig("bezier.png")

    #Set spawning location as initial waypoint
    waypoint = curvy_waypoints[0]
    blueprint = world.get_blueprint_library().filter('vehicle.*model3*')[0]
    location = waypoint.transform.location + carla.Vector3D(0, 0, 1.5)
    rotation = waypoint.transform.rotation
    vehicle = world.spawn_actor(blueprint, carla.Transform(location, rotation))
    print("SPAWNED!")
    
    #Vehicle properties setup
    physics_control = vehicle.get_physics_control()
    max_steer = physics_control.wheels[0].max_steer_angle
    rear_axle_center = (physics_control.wheels[2].position + physics_control.wheels[3].position)/200
    offset = rear_axle_center - vehicle.get_location()
    wheelbase = np.linalg.norm([offset.x, offset.y, offset.z])
    vehicle.set_simulate_physics(True)

    #Add spectator camera to get the view to move with the car 
    camera_bp = world.get_blueprint_library().find('sensor.camera.rgb')
    camera_transform = carla.Transform(carla.Location(x=-10,z=10), carla.Rotation(-45,0,0))
    camera = world.spawn_actor(camera_bp, camera_transform, attach_to=vehicle)

    ##INSERT MODIFYING WAYPOINTS HERE

    while True:

        #Update the camera view
        spectator.set_transform(camera.get_transform())

        #Get next waypoint
        waypoint = get_next_waypoint(world, vehicle, curvy_waypoints)
        world.debug.draw_point(waypoint.transform.location, life_time=5)

        #Control vehicle's throttle and steering
        throttle = 0.85
        vehicle_transform = vehicle.get_transform()
        vehicle_location = vehicle_transform.location
        steer = control_pure_pursuit(vehicle_transform, waypoint.transform, max_steer, wheelbase)
        control = carla.VehicleControl(throttle, steer)
        vehicle.apply_control(control)

def get_next_waypoint(world, vehicle, waypoints):
    vehicle_location = vehicle.get_location()
    min_distance = 1000
    next_waypoint = None

    for waypoint in waypoints:
        waypoint_location = waypoint.transform.location

        #Only check waypoints that are in the front of the vehicle (if x is negative, then the waypoint is to the rear)
        #TODO: Check if this applies for all maps
        if (waypoint_location - vehicle_location).x > 0:

            #Find the waypoint closest to the vehicle, but once vehicle is close to upcoming waypoint, search for next one
            if vehicle_location.distance(waypoint_location) < min_distance and vehicle_location.distance(waypoint_location) > 5:
                min_distance = vehicle_location.distance(waypoint_location)
                next_waypoint = waypoint

    return next_waypoint


def control_pure_pursuit(vehicle_tr, waypoint_tr, max_steer, wheelbase):
    # TODO: convert vehicle transform to rear axle transform
    wp_loc_rel = relative_location(vehicle_tr, waypoint_tr.location) + carla.Vector3D(wheelbase, 0, 0)
    wp_ar = [wp_loc_rel.x, wp_loc_rel.y]
    d2 = wp_ar[0]**2 + wp_ar[1]**2
    steer_rad = math.atan(2 * wheelbase * wp_loc_rel.y / d2)
    steer_deg = math.degrees(steer_rad)
    steer_deg = np.clip(steer_deg, -max_steer, max_steer)
    return steer_deg / max_steer

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

#Returns only the waypoints in one lane
def single_lane(waypoint_list, lane):
    waypoints = []
    for i in range(len(waypoint_list) - 1):
        if waypoint_list[i].lane_id == lane:
            waypoints.append(waypoint_list[i])
    return waypoints
    
#Returns only the waypoints that are not along the straights
def get_curvy_waypoints(waypoints):
    curvy_waypoints = []
    for i in range(len(waypoints) - 1):
        x1 = waypoints[i].transform.location.x
        y1 = waypoints[i].transform.location.y
        x2 = waypoints[i+1].transform.location.x
        y2 = waypoints[i+1].transform.location.y
        if (abs(x1 - x2) > 1) and (abs(y1 - y2) > 1):
            print("x1: " + str(x1) + "  x2: " + str(x2))
            print(abs(x1 - x2))
            print("y1: " + str(y1) + "  y2: " + str(y2))
            print(abs(y1 - y2))
            curvy_waypoints.append(waypoints[i])
      
    #To make the path reconnect to the starting location
    curvy_waypoints.append(curvy_waypoints[0])

    return curvy_waypoints

if __name__ == '__main__':

  try:
      main()
  except KeyboardInterrupt:
      pass
  finally:
      print('\ndone.')