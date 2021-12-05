from controller import Robot

TIME_STEP = 64
robot = Robot()
    
#distance sensor for wall in front detection
distance_sensor = robot.getDevice('distance_sensor')
distance_sensor.enable(TIME_STEP)

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
#avoidObstavleCounter == 93 to turn 90 degrees
avoidObstacleCounter = 0
while robot.step(TIME_STEP) != -1:
    leftSpeed = 1.0
    rightSpeed = 1.0
    if avoidObstacleCounter > 0:
        avoidObstacleCounter -= 1
        leftSpeed = 1.0
        rightSpeed = -1.0
    else:  # read sensors
        if distance_sensor.getValue() < 950.0:
            avoidObstacleCounter = 93
                
    wheelFL.setVelocity(leftSpeed)
    wheelBL.setVelocity(leftSpeed)
    wheelFR.setVelocity(rightSpeed)
    wheelBR.setVelocity(rightSpeed)