from controller import Robot
from controller import Compass
from controller import GPS

class Tile:
    visited = False
    neighbours = []
    
def drive_forward():
    #print('drive forward')
    leftSpeed = 1.0
    rightSpeed = 1.0
    wheelFL.setVelocity(leftSpeed)
    wheelBL.setVelocity(leftSpeed)
    wheelFR.setVelocity(rightSpeed)
    wheelBR.setVelocity(rightSpeed)
    
def turn_right():
    #print('drive to right')
    leftSpeed = 1.0
    rightSpeed = -1.0
    wheelFL.setVelocity(leftSpeed)
    wheelBL.setVelocity(leftSpeed)
    wheelFR.setVelocity(rightSpeed)
    wheelBR.setVelocity(rightSpeed)
    
TIME_STEP = 64
robot = Robot()

num_cols = 7
num_rows = 7

startX = 2
startZ = 4

row = [Tile() for i in range(num_cols)]
map = [list(row) for i in range(num_rows)]

#map = [[-1,-1,-1,-1,-1,-1,-1],
 #      [-1,-1,-1,-1,-1,-1,-1],
#       [-1,-1,-1,-1,-1,-1,-1],
 #      [-1,-1,-1,-1,-1,-1,-1],
#       [-1,-1,-1,-1,-1,-1,-1],
#       [-1,-1,-1,-1,-1,-1,-1],
#       [-1,-1,-1,-1,-1,-1,-1]]
    
#distance sensor for walls detection
ds_front = robot.getDevice('distance_sensor_front')
ds_front.enable(TIME_STEP)
ds_back = robot.getDevice('distance_sensor_back')
ds_back.enable(TIME_STEP)
ds_left = robot.getDevice('distance_sensor_left')
ds_left.enable(TIME_STEP)
ds_right = robot.getDevice('distance_sensor_right')
ds_right.enable(TIME_STEP)

#detect lines on ground
line_sensor = robot.getDevice('line_sensor')
line_sensor.enable(TIME_STEP)

#light sensors
light_sensor = robot.getDevice('light_sensor')
light_sensor.enable(TIME_STEP)

compass = robot.getDevice('compass')
compass.enable(TIME_STEP);

gps = robot.getDevice('gps')
gps.enable(TIME_STEP)

#wheels settup
#get wheels
wheelFL = robot.getDevice('FLWHEEL')
wheelBL = robot.getDevice('BLWHEEL')
wheelFR = robot.getDevice('FRWHEEL')
wheelBR = robot.getDevice('BRWHEEL')

#set position
wheelFL.setPosition(float('inf'))
wheelBL.setPosition(float('inf'))
wheelFR.setPosition(float('inf'))
wheelBR.setPosition(float('inf'))

#set speed
wheelFL.setVelocity(0.0)
wheelBL.setVelocity(0.0)
wheelFR.setVelocity(0.0)
wheelBR.setVelocity(0.0)

#obstacle avoidance if wall in front turn right 90 degrees 
#avoidObstavleCounter == 93 to turn about 90 degrees
avoidObstacleCounter = 0
while robot.step(TIME_STEP) != -1:
    #get direction by compass
    #print(compass.getValues())
    
    #get coords and matrix block index of current block
    #print('X: ', gps.getValues()[0], ' blockX: ', round(gps.getValues()[0]/0.3))
    #print('Y: ', gps.getValues()[1], ' blockY: ', round(gps.getValues()[1]/0.3))
    #print('Z: ', gps.getValues()[2], ' blockZ: ', 1 + round(gps.getValues()[2]/0.3))
    
    print('front distance: ', ds_front.getValue())
    print('back distance: ', ds_back.getValue())
    print('left distance: ', ds_left.getValue())
    print('right distance: ', ds_right.getValue())
    
    currentTileX = round(gps.getValues()[0]/0.3) - 1
    currentTileZ = round(gps.getValues()[2]/0.3)
    
    print('X: ', currentTileX, ' Z: ', currentTileZ)
    
    map[currentTileX][currentTileZ].visited = True
    
    #maze end found
    if light_sensor.getValue() >= 1000:
        #initialize going back to start
        print('Found exit')
    
    #check for wall or line if found turn right else drive forward
    if avoidObstacleCounter > 0:
        avoidObstacleCounter -= 1
        turn_right()
    else:  # read sensors if wall infront or if black line on ground
        if ds_front.getValue() < 950.0 or line_sensor.getValue() > 500.0:
            print('wall or line')
            avoidObstacleCounter = 93
        drive_forward()          