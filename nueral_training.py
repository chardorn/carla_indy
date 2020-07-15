import carla
import random
import math
import numpy as np
import time
import transforms3d

actor_list = [] 
#Order = vehicle, spectator camera, sensor1, sensor2
vehicles = []

def sigmoid(x):
    return 1.0/(1 + np.exp(-x))

def sigmoid_derivative(x):
    return x * (1-0 - x)


class NueralNetwork():

    #initialize variables: input, weights1, weights2, y, output
    def __init__(self, input, y):
        self.input =    x #Input Layer
        self.weights1 = np.random.rand(self.input.shape[1],4) #Hidden Layer 1
        self.weights2 = np.random.rand(4,1) #Hidden Layer 2
        #self.y =        y   #Desired output
        self.output =   np.zeros(self.y.shape)  #Output Layer

    #Feedforward function
    #To calculate the value of 
    #y = sigmoid(Weight2 * sigmoid(Weight1 * x + b1) + b2)
    def feedforward(self, y):
        self.layer1 = sigmoid(np.dot(self.input, self.weights1))
        self.output = sigmoid(np.dot(self.layer1, self.weights2))

    #Loss function
    #We'll use a sum-of-squares loss function
    # = Sum from i = 1 to n of (ypredicted - yactual)^2

    #Gradient Descent
    #The derivative of the loss function dictates which direction is best to move
    #Use chain rule to get derivate of loss = 2(ypredicted - yactual) * z * (1 - z) * x where z = Wx + b

    def backprop(self, y):
        d_weights2 = np.dot(self.layer1.T, (2*(self.y - self.output) * sigmoid_derivative(self.output)))
        d_weights1 = np.dot(self.input.T, (np.dot(2*(self.y - self.output) * sigmoid_derivative(self.output), self.weights2.T) * sigmoid_derivative(self.layer1)))

        self.weights1 += d_weights1
        self.weights2 += d_weights2

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
    max_road_length = 10.0 # in meters #Changed from 50.0 
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

    spawn_actor(world)

    while True:
        print(actor_list)
        #When all actors have been spawned
        if(len(actor_list) >= 4):
            print("true")
            control_vehicle(actor_list[0])
            camera = actor_list[1]
            spectator.set_transform(camera.get_transform())
        timestamp = world.wait_for_tick()
        delta_seconds = timestamp.delta_seconds
        time.sleep(delta_seconds)
        print("update")

def control_vehicle(vehicle):
    throttle = 0.8
    steer = 0.0
    control = carla.VehicleControl(throttle, steer)
    print("control")
    vehicle.apply_control(control)

def spawn_actor(world):
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
    actor_list.append(vehicle)
    vehicle.set_simulate_physics(False)
    physics_control = vehicle.get_physics_control()
    max_steer = physics_control.wheels[0].max_steer_angle
    rear_axle_center = (physics_control.wheels[2].position + physics_control.wheels[3].position)/200
    offset = rear_axle_center - vehicle.get_location()
    wheelbase = np.linalg.norm([offset.x, offset.y, offset.z])
    vehicle.set_simulate_physics(True)

    transform = carla.Transform(carla.Location(x=0.8, z=1.7))

    #Add spectator camera
    camera_bp = world.get_blueprint_library().find('sensor.camera.rgb')
    camera_transform = carla.Transform(carla.Location(x=-10,z=10), carla.Rotation(-45,0,0))
    camera = world.spawn_actor(camera_bp, camera_transform, attach_to=vehicle)
    actor_list.append(camera) #Add to actor_list at [1]

    attach_lidar(world, vehicle, transform)
    attach_collision_sensor(world, vehicle, transform)

    return vehicle, camera

def attach_lidar(world, vehicle, transform):
    #configure LIDAR sensor to only output 2d
    # Find the blueprint of the sensor.
    lidar_bp = world.get_blueprint_library().find('sensor.lidar.ray_cast')
    # Set the time in seconds between sensor captures
    #lidar_bp.set_attribute('sensor_tick', '0.1')
    lidar_bp.set_attribute('channels', '1')
    lidar_bp.set_attribute('upper_fov', '0')
    #lidar_bp.set_attribute('lower_fov', '0')
    lidar_bp.set_attribute('range', '10') #10 is default

    lidar_bp.set_attribute('points_per_second', '100')
    #With 2 channels, and 100 points per second, here are 250 points per scan


    lidar_sensor = world.spawn_actor(lidar_bp, transform, attach_to=vehicle)
    actor_list.append(lidar_sensor) #Add at actor_list[2]

    #Commented out to decrease processing
    lidar_sensor.listen(lambda data: save_lidar_image(data, world, vehicle))

    return lidar_sensor



def attach_collision_sensor(world, vehicle, transform):
    #Configure collision sensor
    collision_bp = world.get_blueprint_library().find('sensor.other.collision')
    collision_sensor = world.spawn_actor(collision_bp, transform, attach_to=vehicle)
    actor_list.append(collision_sensor) #Add at actor_list[3]

    collision_sensor.listen(lambda data: collision_event(data, world, vehicle))

    return collision_sensor


def collision_event(data, world, vehicle):
    print("COLLISION")

    a = len(actor_list)
    for b in range(a):
           actor_list[a-1].destroy()
           actor_list.pop(a-1)
           print(actor_list)
           a = a-1

    spawn_actor(world)
    time.sleep(1)

    
#Returns array of flattened, 2D points
def filter_scan(points):
    print(points)
    #Removes 3rd dimension of points
    filtered_points = np.array(points[:, :2])
    if len(filtered_points) < 5:
        return
    

    print("number of points: " + str(len(filtered_points)))
    return filtered_points
    
def save_lidar_image(image, world, vehicle):

    #Convert raw data to coordinates (x,y,z)
    points = np.frombuffer(image.raw_data, dtype=np.dtype('f4'))
    points = np.reshape(points, (int(points.shape[0] / 3), 3))
    transform = vehicle.get_transform()
    transform = [transform.location.x, transform.location.y, transform.location.z]
    #print(transform)

    #Rotate into the car's frame
    vehicle_rotation = vehicle.get_transform().rotation
    roll = vehicle_rotation.roll 
    pitch = vehicle_rotation.pitch
    yaw = vehicle_rotation.yaw + (np.pi / 2)
    R = transforms3d.euler.euler2mat(roll,pitch,yaw).T

    points = points.copy()

    points[:] = [np.dot(R, point) for point in points]
    points[:] = [np.add(transform, point)  for point in points]       #Move location into car's frame

    filter_scan(points)

    for point in points:
        #Draw in world
        location = carla.Location(x=float(point[0]), y=float(point[1]), z=float(0))
        world.debug.draw_point(location, life_time=0)


    
    #points = filter_scan(points)

    #for point in points:


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

def get_transform(vehicle_location, angle, d=6.4):
        a = math.radians(angle)
        location = carla.Location(d * math.cos(a), d * math.sin(a), 2.0) + vehicle_location
        return carla.Transform(location, carla.Rotation(yaw=180 + angle, pitch=-15))

def main2():

    #Input Array
    x = np.array([[0,0,1],
                  [0,1,1],
                  [1,0,1],
                  [1,1,1]])

    #Actual Output Array
    y = np.array([[0], [1], [1], [0]])


    n = NueralNetwork(x)
    
    for i in range(1500):
        n.feedforward(y)
        n.backprop(y)

    print(n.output)

if __name__ == "__main__":

    try:
        main()
    except KeyboardInterrupt:
        pass
    finally:
        print('destroying actors')
        for actor in actor_list:
           actor.destroy()
        print('\ndone.')