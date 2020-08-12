import carla
import matplotlib.pyplot as plt
import numpy as np
import math
import time
import pandas as pd

actor_list = []
df = pd.DataFrame()


def main():
    global df
    global actor_list

    ### CONSTANTS ###

    target_lane = -3    #Center lane is -3

    target_speed = 30   #Target speed of vehicle

    i = 0               # Starting waypoint

    waypoint_spacing = 40 #Distance between each waypoints in 
    
    ### ###

    #Connect with Carla
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
    map = world.get_map()


    #Setup spectator to be used to move the view with the car as it drives
    spectator = world.get_spectator()


    #Take only the waypoints from the targetLane
    waypoint_list = map.generate_waypoints(waypoint_spacing)
    waypoints = single_lane(waypoint_list, target_lane)
    waypoints = get_curvy_waypoints(waypoints)
    spawnpoint = waypoints[i]

    #SET CUSTOM WAYPOINTS HERE WITH PATH PLANNING

    #Spawn vehicle, attach the spectator camera, and add the controller
    vehicle, camera, controller = spawn_actor(world, spawnpoint)

    print("Vehicle spawned at " + str(spawnpoint))

    while True:

        #Plot point on pyplot graph
        loc = vehicle.get_location()
        plt.plot(loc.x,loc.y, marker = 'o', color = 'r', markersize = 2)
        
        #Log point for export in csv/excel
        #Rotate into the car's frame
        vehicle_rotation = vehicle.get_transform().rotation
        point = [[time.clock(), loc.x, loc.y, vehicle.get_velocity().x, vehicle.get_velocity().y, vehicle_rotation.yaw]]
        print(point)
        df = df.append(point, ignore_index=True,sort=False)

        #Update the camera view
        spectator.set_transform(camera.get_transform())

        # Replace this section to add local path planning 
        ###
        waypoint = waypoints[i]
        #Get next waypoint
        if(vehicle.get_location().distance(waypoints[i].transform.location) < 30):
            i = i + 1
            if i >= len(waypoints) - 1:
                i = 0
        world.debug.draw_point(waypoint.transform.location, life_time=5)
        plt.plot(waypoint.transform.location.x, waypoint.transform.location.y, color = 'g', marker = 'o', markersize = 2)
        ###

        #Control vehicle's throttle and steering
        vehicle_transform = vehicle.get_transform()
        speed = math.sqrt(vehicle.get_velocity().x ** 2 + vehicle.get_velocity().y ** 2)
        #print("speed: " + str(vehicle.get_velocity())) 
        throttle = controller.throttle_control(speed)
        #throttle = 0 # for set velocity
        steer = controller.steering_control(vehicle_transform, waypoint.transform)
        control = carla.VehicleControl(throttle, steer)
        #velocity_vector = carla.Vector3D(vehicle_transform.get_forward_vector())
        #velocity_vector = carla.Vector3D(20, 0, 0)
        #vehicle.set_velocity(velocity_vector)
        #print("VV " + str(velocity_vector))
        vehicle.apply_control(control)

        print("throttle, steer: " + str(throttle) + ", " + str(steer))

        plt.axis([-500, 1500, -800, 100])
        plt.savefig("path.png")


def spawn_actor(world, spawnpoint):
    global actor_list

    #Set spawning location as initial waypoint
    blueprint = world.get_blueprint_library().filter('vehicle.*model3*')[0]
    location = spawnpoint.transform.location + carla.Vector3D(0, 0, 1.5)
    rotation = spawnpoint.transform.rotation
    vehicle = world.spawn_actor(blueprint, carla.Transform(location, rotation))
    actor_list.append(vehicle)

    transform = carla.Transform(carla.Location(x=0.8, z=1.7))

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

    #Configure collision sensor
    collision_bp = world.get_blueprint_library().find('sensor.other.collision')
    collision_sensor = world.spawn_actor(collision_bp, transform, attach_to=vehicle)
    actor_list.append(collision_sensor)
    collision_sensor.listen(lambda data: collision_reset())


    ##Choose controller here##
    #controller = P_Controller(max_steer, wheelbase, world)
    controller = Pure_Pursuit_Controller(max_steer, wheelbase, world)

    return vehicle, camera, controller

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

def collision_reset():
    global actor_list
    for actor in actor_list:
        actor.destroy()

#Returns only the waypoints in one lane
def single_lane(waypoint_list, lane):
    waypoints = []
    for i in range(len(waypoint_list) - 1):
        if waypoint_list[i].lane_id == lane:
            waypoints.append(waypoint_list[i])
    return waypoints

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


class P_Controller():

    def __init__(self, max_steer, wheelbase, world): #TODO: Delete world??
        self.KP = 0.2
        self.KD = 0.1 
        self.past_error = 0
        self.error = 0
        self.sum_error = 0
        self.wheelbase = wheelbase
        self.max_steer  = max_steer

    def steering_control(self, vehicle_tr, waypoint_tr):
        wp_loc_rel = relative_location(vehicle_tr, waypoint_tr.location) + carla.Vector3D(self.wheelbase, 0, 0)
        # TODO: convert vehicle transform to rear axle transform
        x = wp_loc_rel.x
        y = wp_loc_rel.y
        steer_rad = math.tan(y / x)
        error = math.degrees(steer_rad)
        output = self.KP * error
        past_error = error
        return output / self.max_steer

    def throttle_control(self, speed):
        if speed >= 30:
            return 0.7
        else:
            return 1.0

#class Lyapunov_Controller():
#    def __init__():
#    def throttle_control(self, speed):

class Pure_Pursuit_Controller():
    def __init__(self, max_steer, wheelbase, world): #TODO: Delete world??
        self.KP = 0.2
        self.KD = 0.1 
        self.past_error = 0
        self.error = 0
        self.sum_error = 0
        self.wheelbase = wheelbase
        self.max_steer  = max_steer
    def throttle_control(self, speed):
        if speed >= 30:
            return 0.7
        else:
            return 1.0
    def steering_control(self, vehicle_tr, waypoint_tr):
        wp_loc_rel = relative_location(vehicle_tr, waypoint_tr.location) + carla.Vector3D(self.wheelbase, 0, 0)
        x = wp_loc_rel.x
        y = wp_loc_rel.y
        steer_rad = math.tan(y / x)
        steer_deg = np.degrees(steer_rad)
        if abs(steer_deg) < 50:
            return steer_deg / (2 * self.max_steer)
        if abs(steer_deg) < 5:
            return 0
        steer = np.clip(steer_deg, -self.max_steer, self.max_steer)

        return steer / self.max_steer




if __name__ == '__main__':

    try:
        main()
    except KeyboardInterrupt:
        pass
        df.columns = ["Time","X", "Y", "X Vel", "Y Vel", "Yaw"]
        df.to_excel("path.xlsx")
        df.to_csv("path.csv")
        for actor in actor_list:
            actor.destroy()
        print('\ndone.')

