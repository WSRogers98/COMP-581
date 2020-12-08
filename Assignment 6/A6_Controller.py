
"""A6_Controller controller."""

# You may need to import some classes of the controller module. Ex:
#  from controller import Robot, Motor, DistanceSensor

#using math because i dont want to download numpy
import math
from controller import Robot, Motor, DistanceSensor, PositionSensor, GPS

#Sample Wall following provided by Enzo Huang
MAX_SPEED = 5.24
MAX_SENSOR_NUMBER = 16
TIME_STEP = 64
Min_dist = .3
Max_dist = 1
count = 0
hit = False
wall = 0
timer = 0
clock =0
goal= [1.96,0,-1.96]
setup = 0
forceEnd=False
x = 0
y = 0
# create the Robot instance.
robot = Robot()

# get the time step of the current world.
timestep = int(robot.getBasicTimeStep())

posSensor = []
posSensorNames = [
'left wheel sensor','right wheel sensor'
]

sensors = []
sensorNames = [
    'so0', 'so1', 'so2', 'so3', 'so4', 'so5', 'so6', 'so7',
    'so8', 'so9', 'so10', 'so11', 'so12', 'so13', 'so14', 'so15' 
]

for i in range(MAX_SENSOR_NUMBER):
    sensors.append(robot.getDistanceSensor(sensorNames[i]))
    sensors[i].enable(timestep)
for j in range(2):
    posSensor.append(robot.getPositionSensor(posSensorNames[j]))
    posSensor[j].enable(timestep)
    
leftMotor = robot.getMotor('left wheel')
rightMotor = robot.getMotor('right wheel')

# leftMotor.setPosition(float(100.0))
# rightMotor.setPosition(float(100.0))

leftMotor.setPosition(float('inf'))
rightMotor.setPosition(float('inf'))

curGPS= robot.getGPS('gps')
curGPS.enable(timestep)
 
while robot.step(timestep) != -1:
    # Read the sensors:
    sensor_values = []
    distance_values = []
    location= curGPS.getValues()
    # Enter here functions to read sensor data, like:
    #  val = ds.getValue()
    for i in range(MAX_SENSOR_NUMBER):
        sensor_values.append(sensors[i].getValue())
        distance_values.append(5.0 * (1.0 - (sensor_values[i] / 1024)))#MAX_SENSOR_VALUE=1024

    # Process sensor data here.
    # print(sensor_values[0])
    # print(distance_values[0])
    # Enter here functions to send actuator commands, like:
    left_obstacle = distance_values[0] < Min_dist or distance_values[1] < Min_dist or distance_values[2] < Min_dist or distance_values[3] < Min_dist
    right_obstacle = distance_values[5] < Min_dist or distance_values[6] < Min_dist or distance_values[7] < Min_dist
    forward_obstacle = distance_values[0] < Min_dist or distance_values[1] < Min_dist or distance_values[2] < Min_dist or distance_values[3] < Min_dist or distance_values[4] < Min_dist or distance_values[5] < Min_dist
 
    max = distance_values[2] > Min_dist or distance_values[3] > Min_dist
    
    # quick maths B= arc tan (a,b)
    x1 =x
    y1=y
    x= curGPS.getValues()[0]
    y= curGPS.getValues()[2]
    diffx =goal[0]-x
    diffy = goal[2]-y
    Mdirr = math.atan(diffy/diffx)*(180/3.14)
    curDirr=math.atan((y-y1)/(x-x1))*(180/3.14)
    
    #force end before 3 minutes
    clock+=1;
    if clock >=5608:
        forceEnd=True
        print(forceEnd)
        leftSpeed = 0
        rightSpeed = 0
        leftMotor.setVelocity(leftSpeed)
        rightMotor.setVelocity(rightSpeed)
        #the fact that break doesn't make the robot stop makes me angry
        break
    #start    
    if setup <20 and not forceEnd:
        setup+=1
        leftSpeed  = 0.9 * MAX_SPEED
        rightSpeed = 0.1 * MAX_SPEED
    #goal // distance formula
    if math.sqrt((diffx)**2+(diffy)**2) <.4:
        leftSpeed = 0
        rightSpeed = 0
        leftMotor.setVelocity(leftSpeed)
        rightMotor.setVelocity(rightSpeed)
        #the fact that break doesn't make the robot stop makes me angry
        break
    
    if setup >=20 and not forceEnd:    
        leftSpeed  = 0.5 * MAX_SPEED
        rightSpeed = 0.5 * MAX_SPEED
    # modify speeds according to obstacles
    if forward_obstacle and setup >= 20:
        hit = True
       
    if  Mdirr >0:
        Mdirr -=180
     #jiggling, but hopefully it jiggles better than my wall following did
     #TA if you read this please note the speed of which it turns directly impacts if the wheel gets stuck 
     #since the robot can only turn  right it wont detect objects along right side and may get stuck based off turning radius
     #I tried detecting right turn but it did not really work
     #I cant find a foolproof turn radius but .1 and .3 work the best
     # honestly super frustrated and the fact webots is so glithcy makes it worse so I'm giving up here
     #if you want to see something hilarious, run world 2 of the sample worlds you gave us and watch this lil roomba shmoove
     #nevermind i fixed it but change .4 to .3 on line 111 and its the funniest little robot that works that youll ever see
    if curDirr >=Mdirr -1 and curDirr <= Mdirr+1 and setup >=20  and not forceEnd and not left_obstacle and not forward_obstacle and not right_obstacle:
        print('phase 1')
        leftSpeed  = 0.5 * MAX_SPEED
        rightSpeed = 0.5 * MAX_SPEED
    #adjust left
    elif curDirr >Mdirr+1 and setup >=20 and not forceEnd and not left_obstacle and not forward_obstacle and not right_obstacle:
        print('phase 2')
        leftSpeed  = 0.1 * MAX_SPEED
        rightSpeed = 0.3 * MAX_SPEED
    #adjust right
    elif curDirr <Mdirr-1 and setup >=20 and not forceEnd and not left_obstacle and not forward_obstacle and not right_obstacle:
        print('phase 3')
        leftSpeed  = 0.3 * MAX_SPEED
        rightSpeed = 0.1 * MAX_SPEED
    
    if hit and setup >= 20 and not forceEnd:
        if left_obstacle or forward_obstacle:
            print('phase 4')
            leftSpeed  = 0.9 * MAX_SPEED
            rightSpeed = 0.1 * MAX_SPEED
            wall = 0
            count += 1
            
     #attempt at fixing bumbping a wall breaking my code       
     #   elif right_obstacle:
     #       leftSpeed  = 0.1 * MAX_SPEED
      #      rightSpeed = 0.9 * MAX_SPEED
     #       wall = 0
    #        count += 1
        
      #  elif max and count >= 10 and not forceEnd:
       #     print('phase 5')
       #     leftSpeed  = 0.1 * MAX_SPEED
       #     rightSpeed = 0.9 * MAX_SPEED
       #     count -= 1
            
            #if this was my bug im going to scream
       # elif not left_obstacle and not forceEnd:
      #      print('phase 6')
      #      wall += 1
      #      timer += 1
   
       #     if timer >= 20 and timer <= 30 and not forceEnd:
       #         print('phase 7')
       #         leftSpeed  = 0.1 * MAX_SPEED
        #        rightSpeed = 0.9 * MAX_SPEED
             
    # write actuators inputs
    leftMotor.setVelocity(leftSpeed)
    rightMotor.setVelocity(rightSpeed)
    #  motor.setPosition(10.0)
    pass

# Enter here exit cleanup code.