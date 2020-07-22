import carla
import matplotlib.pyplot as plt
import numpy as np
import math
import time

actor_list = []
first_quarter_time = None
second_quarter_time = None
third_quarter_time = None
fourth_quarter_time = None


def main():
    global first_quarter_time, second_quarter_time, third_quarter_time, fourth_quarter_time
    global actor_list

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

    modify_waypoints(curvy_waypoints)

    #Save graph of plotted points as bezier.png
    x = [- p.transform.location.x for p in curvy_waypoints]
    y = [p.transform.location.y for p in curvy_waypoints]
    plt.plot(x, y, marker = 'o')


    i = 5 #initial point

    #Set spawning location as initial waypoint
    spawnpoint = curvy_waypoints[i]
    blueprint = world.get_blueprint_library().filter('vehicle.*model3*')[0]
    location = spawnpoint.transform.location + carla.Vector3D(0, 0, 1.5)
    rotation = spawnpoint.transform.rotation
    
    print("location: " + str(location))
    print("rotation: " + str(rotation))
    vehicle = world.spawn_actor(blueprint, carla.Transform(location, rotation))
    actor_list.append(vehicle)
    print("spawn location: " + str(vehicle.get_transform().location))
    print("spawn rotation: " + str(vehicle.get_transform().rotation))

    print(world.get_actor(vehicle.id).get_transform())


    print("SPAWNED!")
    #plt.plot(-vehicle.get_location().x, vehicle.get_location().y, marker = 'o', color = 'r')

    num = 8

    plt.axis([-1400, 400, -750, 50])
    plt.savefig("new_bezier.png")
    
    #Vehicle properties setup
    physics_control = vehicle.get_physics_control()
    max_steer = physics_control.wheels[0].max_steer_angle
    rear_axle_center = (physics_control.wheels[2].position + physics_control.wheels[3].position)/200
    offset = rear_axle_center - vehicle.get_location()
    wheelbase = np.linalg.norm([offset.x, offset.y, offset.z])
    vehicle.set_simulate_physics(True)

    #Add spectator camera to get the view to move with the car 
    camera_bp = world.get_blueprint_library().find('sensor.camera.rgb')
    camera_transform = carla.Transform(carla.Location(x=-10,z=10), carla.Rotation(-30,0,0))
    camera = world.spawn_actor(camera_bp, camera_transform, attach_to=vehicle)
    actor_list.append(camera)

    transform = carla.Transform(carla.Location(x=0.8, z=1.7))

    ##INSERT MODIFYING WAYPOINTS HERE

    max_x = -10000
    max_y = -10000
    min_x = 10000
    min_y = 10000
    for waypoint in curvy_waypoints:
        if waypoint.transform.location.x < min_x:
            min_x = waypoint.transform.location.x
        if waypoint.transform.location.x > max_x:
            max_x = waypoint.transform.location.x
        if waypoint.transform.location.y < min_y:
            min_y = waypoint.transform.location.y
        if waypoint.transform.location.y > max_y:
            max_y = waypoint.transform.location.y
    
    mid_x = (max_x + min_x) / 2
    mid_y = (max_y + min_y) / 2

    print("mid_x, mid_y " + str(mid_x) + "  " + str(mid_y))


    start_time = time.clock()
    section = 1

    while True:

        loc = vehicle.get_location()
        #print(loc)

        if section == 1:
            if loc.y < mid_y:
                first_quarter_time = time.clock() - start_time
                section = 2
                start_time = time.clock()
                print("first time: " + str(first_quarter_time))
                continue
        if section == 2:
            if loc.x < mid_x:
                second_quarter_time = time.clock() - start_time
                section = 3
                start_time = time.clock()
        if section == 3:
            if loc.y > mid_y:
                third_quarter_time = time.clock() - start_time
                section = 4
                start_time = time.clock()
                continue
        if section == 4:
            if loc.x > mid_x:
                fourth_quarter_time = time.clock() - start_time
                section = 1
                start_time = time.clock()
                continue


        #Update the camera view
        spectator.set_transform(camera.get_transform())

        #Get next waypoint
        waypoint = curvy_waypoints[i]
        if(vehicle.get_location().distance(curvy_waypoints[i].transform.location) < 20):
            i = i + 1
        world.debug.draw_point(waypoint.transform.location, life_time=5)

        #Control vehicle's throttle and steering
        throttle = 0.5
        vehicle_transform = vehicle.get_transform()
        vehicle_location = vehicle_transform.location
        steer = control_pure_pursuit(vehicle_transform, waypoint.transform, max_steer, wheelbase, world)
        control = carla.VehicleControl(throttle, steer)
        vehicle.apply_control(control)
        #print("steer: " + str(steer))

        time.sleep(0.1)


def modify_waypoints(curvy_waypoints):
    num_per_group = int(len(curvy_waypoints) /4)
    #for i in range(num_per_group):
        
    print("number per group " + str(num_per_group))

def control_pure_purs    wp_loc_rel = relative_location(vehicle_tr, waypoint_tr.location) + carla.Vector3D(wheelbase, 0, 0)
uit(vehicle_tr, waypoint_tr, max_steer, wheelbase, world):
    # TODO: convert vehicle transform to rear axle transform
    wp_ar = [wp_loc_rel.x, wp_loc_rel.y]
    d2 = wp_ar[0]**2 + wp_ar[1]**2
    steer_rad = math.atan(2 * wheelbase * wp_loc_rel.y / d2)
    steer_deg = math.degrees(steer_rad)
    steer_deg = np.clip(steer_deg, -max_steer, max_steer)

    yaw = vehicle_tr.rotation.yaw
    angle = yaw + steer_deg
    angle_rad = math.radians(angle)

    print("v loc: " + str(vehicle_tr.location))
    print("yaw: " + str(yaw))
    print("angle: " + str(angle))

    #trying to draw on arrow
    y = math.sin(angle_rad) * 10  + vehicle_tr.location.y
    x = math.cos(angle_rad) * 10 + vehicle_tr.location.x

    print("x, y " + str(x) + " " + str(y))

    #world.debug.draw_point(carla.Location(x,y,0), life_time = 1.0)
    world.debug.draw_arrow(vehicle_tr.location, carla.Location(x,y,0), life_time = 1)
    
    return steer_deg / max_steer


def relative_location(frame, location):
    origin = frame.location
    forward = frame.get_forward_vector()
    right = frame.get_right_vector()
    up = frame.get_up_vector()
    disp = location - origin
    x = np.dot([disp.x, disp.y, disp.z], [forward.x, forward.y, forward.z])
    if x > 50:
        x = 50
    y = np.dot([disp.x, disp.y, disp.z], [right.x, right.y, right.z])
    if y > 50:
        x = 50
    z = np.dot([disp.x, disp.y, disp.z], [up.x, up.y, up.z])

    #print(carla.Vector3D(x, y, z))
    return carla.Vector3D(x, y, z)

def degrees_to_steering_percentage(degrees):
    """ Returns a steering "percentage" value between 0.0 (left) and 1.0
    (right) that is as close as possible to the requested degrees. The car's
    wheels can't turn more than max_angle in either direction. """
    degrees = -(degrees - 90)
    print("degrees = " + str(degrees))
    max_angle = 45
    if degrees < -max_angle:
        return 1.0
    if degrees > max_angle:
        return -1.0
    if abs(degrees) < 5:
        return 0
        
    return - (degrees / max_angle)



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
        for actor in actor_list:
            actor.destroy()
        print('\ndone.')