import shapely
from shapely.geometry import LinearRing, LineString, Point, MultiPoint
from numpy import sin, cos, pi, sqrt
from random import random

# A prototype simulation of a differential-drive robot with one sensor

# Constants
###########
R = 0.02  # radius of wheels in meters
L = 0.10  # distance between wheels in meters

W = 4.0  # width of arena
H = 4.0  # height of arena

robot_timestep = 0.1        # 1/robot_timestep equals update frequency of robot
simulation_timestep = 0.01  # timestep in kinematics sim (probably don't touch..)

# the world is a rectangular arena with width W and height H
world = LinearRing([(W/2,H/2),(-W/2,H/2),(-W/2,-H/2),(W/2,-H/2)])
obstacles: list[LinearRing] = [
    world,
    LinearRing([(1,1),(1,0.5),(0.5,0.75)]),
]

world_file = open("world.dat", "w")
for obstacle in obstacles:
    for x, y in obstacle.coords:
        world_file.write( str(x) + ", " + str(y) + "\n")
    world_file.write("\n")
world_file.close()

# Variables 
###########

x = 0.0   # robot position in meters - x direction - positive to the right 
y = 0.0   # robot position in meters - y direction - positive up
q = 0.0   # robot heading with respect to x-axis in radians 

left_wheel_velocity =  random()   # robot left wheel velocity in radians/s
right_wheel_velocity =  random()  # robot right wheel velocity in radians/s

dist_to_wall = 0.5 # Distance to the wall where the sensors are activated
turn_speed = 0.4 

# Kinematic model
#################
# updates robot position and heading based on velocity of wheels and the elapsed time
# the equations are a forward kinematic model of a two-wheeled robot - don't worry just use it
def simulationstep():
    global x, y, q

    for step in range(int(robot_timestep/simulation_timestep)):     #step model time/timestep times
        v_x = cos(q)*(R*left_wheel_velocity/2 + R*right_wheel_velocity/2) 
        v_y = sin(q)*(R*left_wheel_velocity/2 + R*right_wheel_velocity/2)
        omega = (R*right_wheel_velocity - R*left_wheel_velocity)/(2*L)    
    
        x += v_x * simulation_timestep
        y += v_y * simulation_timestep
        q += omega * simulation_timestep

# Simulation loop
#################
file = open("trajectory.dat", "w")

for cnt in range(8000):
    #simple single-ray sensor
    # ray = LineString([(x, y), (x+cos(q)*2*(W+H),(y+sin(q)*2*(W+H))) ])  # a line from robot to a point outside arena in direction of q
    # s = world.intersection(ray)
    # distance = sqrt((s.x-x)**2+(s.y-y)**2)                    # distance to wall

    wall_right = False
    wall_left = False

    #simple multiple-ray sensor
    for degree in range(0,360, 5):

        if degree >= 90 and degree < 270:
            continue

        radian = degree * pi/180

        ray = LineString([(x, y), (x+cos(q+radian)*2*(W+H),(y+sin(q+radian)*2*(W+H)))])  # a line from robot to a point outside arena in direction of q
        
        distance = None

        for obstacle in obstacles:
            s = obstacle.intersection(ray)
            points = []

            if isinstance(s, Point):
                points = [s]
            elif isinstance(s, MultiPoint):
                points = s.geoms

            for point in points:
                distance_next = sqrt((point.x-x)**2+(point.y-y)**2)
                if distance is None or distance_next < distance:
                    distance = distance_next


        if distance < dist_to_wall:
            if degree < 90:
                wall_left = True
            if degree >= 270:
                wall_right = True
    
    #simple controller - change direction of wheels every 10 seconds (100*robot_timestep) unless close to wall then turn on spot
    if wall_right:
        #print("left")
        left_wheel_velocity = -turn_speed
        right_wheel_velocity = turn_speed
    elif wall_left:
        #print("right")
        left_wheel_velocity = turn_speed
        right_wheel_velocity = -turn_speed
    else:                
        #print("straigt")
        if cnt%100==0:
            left_wheel_velocity = random()
            right_wheel_velocity = random()
        
    #step simulation
    simulationstep()

    #check collision with arena walls 
    if (world.distance(Point(x,y))<L/2):
        break
        
    if cnt%50==0:
        file.write( str(x) + ", " + str(y) + ", " + str(cos(q)*0.2) + ", " + str(sin(q)*0.2) + "\n")

file.close()
    
