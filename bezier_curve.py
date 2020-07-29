import carla
import matplotlib.pyplot as plt
import numpy as np
import math
import time

actor_list = []
times_array = [0,0,0,0]

def main():
    global times_array
    global actor_list

    times_array = np.load("times.npy", allow_pickle=True)
    print(times_array)
    
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


    #i = 0 #initial point
    i = len(curvy_waypoints) - 2

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

    plt.axis([-1400, 400, -750, 50])
    
    
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


    start_time = time.time()
    section = 0

    while_time = 0.1

    pid = PID_Controller(max_steer, wheelbase, world, while_time)

 

    while True:

        loc = vehicle.get_location()
        plt.plot(-loc.x, loc.y, marker = 'o', color = 'r', markersize = 2)
        plt.savefig("new_bezier.png")


        #Update the camera view
        spectator.set_transform(camera.get_transform())

        #Get next waypoint
        waypoint = curvy_waypoints[i]
        if(vehicle.get_location().distance(curvy_waypoints[i].transform.location) < 15):
            i = i + 1
            if i >= len(curvy_waypoints) - 1:
                i = 0
        world.debug.draw_point(waypoint.transform.location, life_time=5)


        #Control vehicle's throttle and steering
        speed = math.sqrt(vehicle.get_velocity().x ** 2 + vehicle.get_velocity().y ** 2)
        #print(speed)
        if speed >= 30:
            throttle = 0.7
        else:
            throttle = 1.0
        
        vehicle_transform = vehicle.get_transform()
        steer = pid.steering_control(vehicle_transform, waypoint.transform)
        control = carla.VehicleControl(throttle, steer)
        vehicle.apply_control(control)

        time.sleep(while_time)
        

        if section == 0:
            if loc.x > mid_x:
                section = 1
                start_time = time.time()
        if section == 1:
            if loc.y < mid_y:
                update_times(1, time.time() - start_time)
                section = 2
                start_time = time.time()
                print("first time: " + str(times_array[0]))
                
        if section == 2:
            if loc.x < mid_x:
                update_times(2, time.time() - start_time)
                section = 3
                start_time = time.time()
        if section == 3:
            if loc.y > mid_y:
                update_times(3, time.time() - start_time)
                section = 4
                start_time = time.time()
        if section == 4:
            if loc.x > mid_x:
                update_times(4, time.time() - start_time)
                section = 1
                start_time = time.time()

        time.sleep(0.1)

def update_times(section, time):
    global times_array
    if time < times_array[section - 1]:
        times_array[section - 1] = time


def modify_waypoints(curvy_waypoints):
    num_per_group = int(len(curvy_waypoints) /4)
    #for i in range(num_per_group):
        
    print("number per group " + str(num_per_group))

def control_pure_pursuit(vehicle_tr, waypoint_tr, max_steer, wheelbase, world):
    wp_loc_rel = relative_location(vehicle_tr, waypoint_tr.location) + carla.Vector3D(wheelbase, 0, 0)
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

    #print("x, y " + str(x) + " " + str(y))

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
    #x = np.clip(x, -10, 10)
    y = np.dot([disp.x, disp.y, disp.z], [right.x, right.y, right.z])
    #y = np.clip(y, -10, 10)
    z = np.dot([disp.x, disp.y, disp.z], [up.x, up.y, up.z])

    #print(carla.Vector3D(x, y, z))
    return carla.Vector3D(x, y, z)

class PID_Controller():

    def __init__(self, max_steer, wheelbase, world, while_time): #TODO: Delete world??
        self.KP = 0.2
        self.KD = 0.1 
        self.past_error = 0
        self.error = 0
        self.sum_error = 0
        self.elapsed_time= while_time
        self.wheelbase = wheelbase
        self.max_steer  = max_steer

    def steering_control(self, vehicle_tr, waypoint_tr):
        wp_loc_rel = relative_location(vehicle_tr, waypoint_tr.location) + carla.Vector3D(self.wheelbase, 0, 0)
        # TODO: convert vehicle transform to rear axle transform
        x = wp_loc_rel.x
        y = wp_loc_rel.y
        #print("x, y: " + str(x) + " " + str(y))
        steer_rad = math.tan(y / x)
        error = math.degrees(steer_rad)
        #derivative = (error - self.past_error) / self.elapsed_time
        #output = self.KP * error + self.KD * derivative
        output = self.KP * error
        past_error = error
        return output / self.max_steer

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
        np.save("times.npy", times_array)
        for actor in actor_list:
            actor.destroy()
        print('\ndone.')