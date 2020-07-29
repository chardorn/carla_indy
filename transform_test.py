import carla
import numpy
import matplotlib.pyplot as plt
import numpy as np


def relative_location(frame, location):
    origin = frame.location
    forward = frame.get_forward_vector()
    print("forward: " + str(forward))
    right = frame.get_right_vector()
    print("right: " + str(right))
    up = frame.get_up_vector()
    print("up: " + str(up))
    disp = location - origin
    print("disp: " + str(disp))
    x = np.dot([disp.x, disp.y, disp.z], [forward.x, forward.y, forward.z])
    y = np.dot([disp.x, disp.y, disp.z], [right.x, right.y, right.z])
    z = np.dot([disp.x, disp.y, disp.z], [up.x, up.y, up.z])

    #print(carla.Vector3D(x, y, z))
    return (x, y, z)

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

location1 = carla.Location(10,10,0)
rotation1 = carla.Rotation(0,0,0)
transform1 = carla.Transform(location1, rotation1)
print(transform1)

plt.plot(location1.x, location1.y, marker = 'o')
plt.quiver(location1.x, location1.y, np.cos(np.radians(rotation1.yaw)), np.sin(np.radians(rotation1.yaw)), scale = 4)

location2 = carla.Location(5,5,0)
rotation2 = carla.Rotation(0,45,0)
transform2 = carla.Transform(location2, rotation2)
print(transform2)

plt.plot(location2.x, location2.y, marker = 'o')

plt.xlim(0,15)
plt.ylim(0,15)
plt.savefig("graph.png")

print(relative_location(transform1, location2))

