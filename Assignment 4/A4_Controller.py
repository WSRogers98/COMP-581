"""A4_Controller controller."""

# You may need to import some classes of the controller module. Ex:
#  from controller import Robot, Motor, DistanceSensor
from controller import Robot, Motor, DistanceSensor

# create the Robot instance.
MAX_SPEED = 5.24
MAX_SENSOR_NUMBER = 16
TIME_STEP = 64
Min_dist = .2
walldist=.3
followWall =False
wallDetected =False
endwall = False
followDist=.4
checkahead=2
twometers=2
absmax=3
toofar=5
tick =0
walk=0
dontbump=0
robot = Robot()

# get the time step of the current world.
timestep = int(robot.getBasicTimeStep())


# You should insert a getDevice-like function in order to get the
# instance of a device of the robot. Something like:
#  motor = robot.getMotor('motorname')
#  ds = robot.getDistanceSensor('dsname')
#  ds.enable(timestep)
sensors = []
sensorNames = [
    'so0', 'so1', 'so2', 'so3', 'so4', 'so5', 'so6', 'so7',
    'so8', 'so9', 'so10', 'so11', 'so12', 'so13', 'so14', 'so15' 
]


for i in range(MAX_SENSOR_NUMBER):
    sensors.append(robot.getDistanceSensor(sensorNames[i]))
    sensors[i].enable(timestep)
    
rightMotor = robot.getMotor('right wheel')
leftMotor = robot.getMotor('left wheel')

rightMotor.setPosition(float('inf'))
leftMotor.setPosition(float('inf'))

rightMotor.setVelocity(0.0)
leftMotor.setVelocity(0.0)

# Main loop:
# - perform simulation steps until Webots is stopping the controller
while robot.step(timestep) != -1:
    # Read the sensors:
    sensor_values = []
    distance_values = []
    # Enter here functions to read sensor data, like:
    #  val = ds.getValue()
    for i in range(MAX_SENSOR_NUMBER):
        sensor_values.append(sensors[i].getValue())
        distance_values.append(5.0 * (1.0 - (sensor_values[i] / 1024)))
    # Process sensor data here.
    forward_obstacle = distance_values[4] < Min_dist  or distance_values[3] <Min_dist or distance_values[2] <Min_dist or distance_values[1] <Min_dist or distance_values[5] <Min_dist or distance_values[0] <Min_dist
    left_obstacle = distance_values[0] < walldist  or distance_values[15] < walldist
    deadahead= distance_values[5] < followDist
    stayWithin = distance_values[0] > followDist  or distance_values[15] > followDist or distance_values[1] > followDist
    nothingahead= (distance_values[1] > checkahead and distance_values[2] > checkahead and distance_values[3] > checkahead and distance_values[0] > checkahead and distance_values[4] > checkahead and distance_values[5] > checkahead and distance_values[15] > checkahead)
    leftSpeed  = 0.5 * MAX_SPEED
    rightSpeed = 0.5 * MAX_SPEED
    # Enter here functions to send actuator commands, like:
    #  motor.setPosition(10.0)
    if forward_obstacle and not endwall:
        # turn right   
        leftSpeed  = .75 
        rightSpeed = -.75 
   
    elif left_obstacle and not endwall:
        #Follow Along     
        followWall =True
        leftSpeed  = 0.5 * MAX_SPEED
        rightSpeed = 0.5 * MAX_SPEED
        
    elif nothingahead and followWall and not endwall:
        print('end wall')
        followWall =False
        endwall =True
        leftSpeed  = 0.5 * MAX_SPEED
        rightSpeed = 0.5 * MAX_SPEED
             
    elif stayWithin and followWall and not endwall:
    
        leftSpeed  = 0
        rightSpeed = 1
    elif endwall:
         if forward_obstacle:
            print('deadahead')
            endwall=False
            tick =0
            walk=0
            dontbump=0
         if dontbump<50:
             print('bump: ', dontbump)
             dontbump+=1
             leftSpeed  = 0.5 * MAX_SPEED
             rightSpeed = 0.5 * MAX_SPEED
         if tick <65 and dontbump>=50:
             print('tick: ', tick)
             leftSpeed  = 0
             rightSpeed = 0.5 * MAX_SPEED
             tick+=1      
         if tick >=65 and walk <225 and dontbump>=50:  
             print('walk: ', walk) 
             leftSpeed  = 0.5 * MAX_SPEED
             rightSpeed = 0.5 * MAX_SPEED
             walk+=1
         if tick >=65 and walk>=225 and dontbump>=50:
             leftSpeed  = 0
             rightSpeed = 0
             pass
        
    elif not endwall:
        #go straight
        leftSpeed  = 0.5 * MAX_SPEED
        rightSpeed = 0.5 * MAX_SPEED
    # write actuators inputs
    leftMotor.setVelocity(leftSpeed)
    rightMotor.setVelocity(rightSpeed)
    #  motor.setPosition(10.0)
    pass


# Enter here exit cleanup code.
