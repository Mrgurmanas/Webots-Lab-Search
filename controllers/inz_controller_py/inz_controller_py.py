from controller import Robot
from controller import Compass
from controller import GPS
from controller import InertialUnit
from controller import LED
from controller import Display, ImageRef


front = False
bottom = False
right = False
left = False

def LeeAlgo(map, startx, starty, endx, endy, maxx, maxy):
  tile = map[startx][starty]
  tile.distance = 0
  LeeAlgoRec(map, startx, starty, maxx, maxy)

  
#  totalDist = newmap[endx][endy].distance

  
  target = GetListToTile(map, endx, endy, startx, starty, maxx, maxy)
  for Tile in map:
    for Tile2 in Tile:
      Tile2.distance = -1
  return target
 # return newmap


def GetListToTile(map, endx, endy, startx, starty, maxx, maxy):
  target = []
  distance = map[endx][endy].distance
  x = endx
  y = endy
  while distance > 0:
      robot.step(TIME_STEP)
      tile = map[x][y]
      wallnumber = tile.wallnumber + tile.wallnumberlines
     # print("tilas ", x, " ", y, " atstumas ", tile.distance, " sienos numeris ", wallnumber)
      if wallnumber >= 8:
         wallnumber = wallnumber - 8
      elif y > 0 and map[x][y-1].distance < tile.distance:
         target.append("E")
         distance = map[x][y-1].distance
         x = x
         y = y-1
         continue
      if wallnumber >= 4:
        wallnumber = wallnumber - 4
      elif x > 0 and map[x-1][y].distance < tile.distance:
         target.append("N")
         distance = map[x-1][y].distance
         x = x-1
         y = y
         continue
      

      if wallnumber >= 2:
        wallnumber = wallnumber - 2
      elif y < maxy-1 and map[x][y+1].distance < tile.distance:
         target.append("W")
         distance = map[x][y+1].distance
         x = x
         y = y+1
         continue
      

      if wallnumber >= 1:
        wallnumber = wallnumber - 1
      elif x < maxx-1 and map[x+1][y].distance < tile.distance:
         target.append("S")
         distance = map[x+1][y].distance
         x = x+1
         y = y
         continue
  return target[::-1]


def LeeAlgoRec(map, x, y, maxx, maxy):
    tile = map[x][y]
    wallnumber = tile.wallnumber + tile.wallnumberlines
    if tile.seen:
    #  print("pradinis tilo wall number ", wallnumber, " tilas ", x, " ", y)
      N = False
      E = False
      S = False
      W = False
      if wallnumber >= 8:
        wallnumber = wallnumber - 8
    #    print("naujas tilo wall number ", wallnumber, " tilas ", x, " ", y)
      elif y > 0 and (map[x][y-1].distance == -1 or map[x][y-1].distance > tile.distance + 1) and map[x][y-1].seen:
        map[x][y-1].distance = tile.distance + 1
        W = True
      

      if wallnumber >= 4:
        wallnumber = wallnumber - 4
     #   print("naujas tilo wall number ", wallnumber, " tilas ", x, " ", y)
      elif x > 0 and (map[x-1][y].distance == -1 or map[x-1][y].distance > tile.distance + 1) and map[x-1][y].seen:
        map[x-1][y].distance = tile.distance + 1
        S = True
      

      if wallnumber >= 2:
        wallnumber = wallnumber - 2
    #    print("naujas tilo wall number ", wallnumber, " tilas ", x, " ", y)
      elif y < maxy-1 and (map[x][y+1].distance == -1 or map[x][y+1].distance > tile.distance + 1) and map[x][y+1].seen:
        map[x][y+1].distance = tile.distance + 1
        E = True
      

      if wallnumber >= 1:
        wallnumber = wallnumber - 1
   #     print("naujas tilo wall number ", wallnumber, " tilas ", x, " ", y)
      elif x < maxx-1 and (map[x+1][y].distance == -1 or map[x+1][y].distance > tile.distance + 1) and map[x+1][y].seen:
        map[x+1][y].distance = tile.distance + 1
        N = True

      if W:
    #    print("tilas " , x , " " , y , " galima i W")
        LeeAlgoRec(map, x, y-1, maxx, maxy)
      if S:
     #    print("tilas " , x , " " , y , " galima i S")
         LeeAlgoRec(map, x-1, y, maxx, maxy)
      if E:
    #     print("tilas " , x , " " , y , " galima i E")
         LeeAlgoRec(map, x, y+1, maxx, maxy)
      if N:
     #   print("tilas " , x , " " , y , " galima i N")
        LeeAlgoRec(map, x+1, y,maxx, maxy)



def moveToTarget(map, direction, target):
    arrived = True
    if not target:
        arrived = False
    for dir in target:
        updateDisplay()
        if arrived == True:
            currentTileX = round(gps.getValues()[0]/0.3) - 1
            currentTileZ = round(gps.getValues()[2]/0.3)
            arrived = False
            if dir == 'N':
                rotate(2)
                arrived = drive_until(map, direction, currentTileX, currentTileX+1, currentTileZ, currentTileZ)
            if dir == 'E':
                rotate(3)
                arrived = drive_until(map, direction, currentTileX, currentTileX, currentTileZ, currentTileZ+1)
            if dir == 'S':
                rotate(4)
                arrived = drive_until(map, direction, currentTileX, currentTileX-1, currentTileZ, currentTileZ)
            if dir == 'W':
                rotate(1)
                arrived = drive_until(map, direction, currentTileX, currentTileX, currentTileZ, currentTileZ-1)
    return arrived

class Tile:
    visited = False  
    seen = False
    wallnumber = 15
    wallnumberlines = 0
    distance = -1
    def __init__(self):
      pass
def drive_forward():
    #print('drive forward')
    leftSpeed = 1.0
    rightSpeed = 1.0
    wheelFL.setVelocity(leftSpeed)
    wheelBL.setVelocity(leftSpeed)
    wheelFR.setVelocity(rightSpeed)
    wheelBR.setVelocity(rightSpeed)
    
def drive_until(map, direction, currentTileX, xpos, currentTileZ, zpos):
       startX = currentTileX
       startZ = currentTileZ
       foundLine = False
       while (abs(currentTileX - xpos) > 0.15 or abs(currentTileZ - zpos) > 0.15) and foundLine == False:
        updateDisplay()
        currentTileX = round(gps.getValues()[0]/0.3, 2) - 1
        currentTileZ = round(gps.getValues()[2]/0.3, 2)
        robot.step(TIME_STEP)
     #   print(abs(currentTileX - xpos))
     #   print(abs(currentTileZ - zpos))
        if line_sensor.getValue() > 500.0:
            foundLine = True
        drive_forward() 
       stop()
       if foundLine == True:
           currentTileX = round(gps.getValues()[0]/0.3) - 1
           currentTileZ = round(gps.getValues()[2]/0.3)
           if direction == 1:
              map[currentTileX][currentTileZ].wallnumberlines += 8
              direction = rotate(3)
           elif direction == 2:
              map[currentTileX][currentTileZ].wallnumberlines += 1
              direction = rotate(4)
           elif direction == 3:
              map[currentTileX][currentTileZ].wallnumberlines += 2
              direction = rotate(1)
           elif direction == 4:
              map[currentTileX][currentTileZ].wallnumberlines += 4
              direction = rotate(2)
           currentTileX = round(gps.getValues()[0]/0.3,2) - 1
           currentTileZ = round(gps.getValues()[2]/0.3,2)
           drive_until(map, direction, currentTileX, startX, currentTileZ, startZ)
           return False
       else:
            return True
    
def stop():
    #print('drive forward')
    leftSpeed = 0
    rightSpeed = 0
    wheelFL.setVelocity(leftSpeed)
    wheelBL.setVelocity(leftSpeed)
    wheelFR.setVelocity(rightSpeed)
    wheelBR.setVelocity(rightSpeed)
    

def updateDisplay():
    front = False
    bottom = False
    right = False
    left = False
    if ds_front.getValue() < 1000:
        front = True
    if ds_back.getValue() < 1000:
        bottom = True
    if ds_left.getValue() < 1000:
        left = True
    if ds_right.getValue() < 1000:
        right = True
    
    #print("front value: ", front)
    if not front and not bottom and not right and not left:
            displayWallDetection = detected_wall_display.imageLoad("BlankWallDetectionDisplay.png")
            detected_wall_display.imagePaste(displayWallDetection,0,0,True)
    if front and not bottom and not right and not left:
            displayWallDetection = detected_wall_display.imageLoad("FrontWallDetectionDisplay.png")
            detected_wall_display.imagePaste(displayWallDetection,0,0,True)
    if not front and bottom and not right and not left:
            displayWallDetection = detected_wall_display.imageLoad("BottomWallDetectionDisplay.png")
            detected_wall_display.imagePaste(displayWallDetection,0,0,True)
    if not front and not bottom and right and not left:
            displayWallDetection = detected_wall_display.imageLoad("RightWallDetectionDisplay.png")
            detected_wall_display.imagePaste(displayWallDetection,0,0,True)
    if not front and not bottom and not right and left:
            displayWallDetection = detected_wall_display.imageLoad("LeftWallDetectionDisplay.png")
            detected_wall_display.imagePaste(displayWallDetection,0,0,True)
            
    if front and bottom and not right and not left:
            displayWallDetection = detected_wall_display.imageLoad("TopAndBottomWallDetectionDisplay.png")
            detected_wall_display.imagePaste(displayWallDetection,0,0,True)
    if front and not bottom and right and not left:
            displayWallDetection = detected_wall_display.imageLoad("TopAndRightWallDetection.png")
            detected_wall_display.imagePaste(displayWallDetection,0,0,True)
    if front and not bottom and not right and left:
            displayWallDetection = detected_wall_display.imageLoad("LeftAndTopWallDetection.png")
            detected_wall_display.imagePaste(displayWallDetection,0,0,True)
    if not front and bottom and right and not left:
            displayWallDetection = detected_wall_display.imageLoad("RightAndBottomWallDetectionDisplay.png")
            detected_wall_display.imagePaste(displayWallDetection,0,0,True)
            
    if not front and bottom and not right and left:
            displayWallDetection = detected_wall_display.imageLoad("LeftAndBottomWallDetectionDisplay.png")
            detected_wall_display.imagePaste(displayWallDetection,0,0,True)
            
            
    if not front and not bottom and right and left:
            displayWallDetection = detected_wall_display.imageLoad("LeftAndBottomWallDetectionDisplay.png")
            detected_wall_display.imagePaste(displayWallDetection,0,0,True)
                
    if front and bottom and right and  not left:
            displayWallDetection = detected_wall_display.imageLoad("RightTopAndBottomWallDetectionDisplay.png")
            detected_wall_display.imagePaste(displayWallDetection,0,0,True)
    if not front and bottom and not right and left:
            displayWallDetection = detected_wall_display.imageLoad("LeftAndBottomWallDetectionDisplay.png")
            detected_wall_display.imagePaste(displayWallDetection,0,0,True)
    if not front and bottom and right and left:
            displayWallDetection = detected_wall_display.imageLoad("LeftRightBottomWallDetectionDisplay.png")
            detected_wall_display.imagePaste(displayWallDetection,0,0,True)
    if not front and not bottom and right and left:
            displayWallDetection = detected_wall_display.imageLoad("LeftRightWallDetectionDisplay.png")
            detected_wall_display.imagePaste(displayWallDetection,0,0,True)
    if front and bottom and right and left:
            displayWallDetection = detected_wall_display.imageLoad("AllWallDetectionDisplay.png")
            detected_wall_display.imagePaste(displayWallDetection,0,0,True)
    
          
       
def rotate(direction):
    done = False
    while done == False:
        
        
        updateDisplay()
        robot.step(TIME_STEP)
        delta = 0.05#1.5708/2
        currentPitch = round(inertial_unit.getRollPitchYaw()[2],4)
        #print('dir: ', direction, ' v: ', currentPitch, ' f: ', ((-1.5708*2) + delta), ' a: ',((1.5708*2) - delta))
        if direction == 1 and (currentPitch > ((-1.5708*2) + delta)) and (currentPitch < ((1.5708*2) - delta)):#(currentPitch > ((-1.5708*2) + delta) and currentPitch < ((1.5708*2) - delta)):
            turn_right()
        elif direction == 2 and (currentPitch > (1.5708 + delta) or currentPitch < (1.5708 - delta)):
            turn_right()
        elif direction == 3 and (currentPitch > (0 + delta) or currentPitch < (0 - delta)):
           # print('3 ', currentPitch, ' > ', (0 +  delta), ' or ', currentPitch, ' < ', (0 - delta))
            turn_right()
        elif direction == 4 and (currentPitch > (-1.5708 + delta) or currentPitch < ((-1.5708) - delta)):
           # print('4 ', currentPitch, ' > ', (-1.5708 + delta), ' or ', currentPitch, ' < ', ((-1.5708) - delta))
            turn_right()
        else:
           done = True
    stop()
    return direction

def turn_right():
    #print('drive to right')
    leftSpeed = 1.0
    rightSpeed = -1.0
    wheelFL.setVelocity(leftSpeed)
    wheelBL.setVelocity(leftSpeed)
    wheelFR.setVelocity(rightSpeed)
    wheelBR.setVelocity(rightSpeed)

def print_current_dir(x):
    #left 1 top 2 right 3 bottom 4
    if x == 1:
        print('left')
    if x == 2:
        print('up')
    if x == 3:        
        print('right')
    if x == 4:
        print('bottom') 
    
TIME_STEP = 64
robot = Robot()

num_x = 7
num_z= 7

front = False
bottom = False
right = False
left = False



maxx, maxy = 7, 7
map = [[Tile() for x in range(num_x)] for y in range(num_z)] 
    
#distance sensor for walls detection
ds_front = robot.getDevice('distance_sensor_front')
ds_front.enable(TIME_STEP)
print(ds_front.getType())

ds_back = robot.getDevice('distance_sensor_back')
ds_back.enable(TIME_STEP)
ds_left = robot.getDevice('distance_sensor_left')
ds_left.enable(TIME_STEP)
ds_right = robot.getDevice('distance_sensor_right')
ds_right.enable(TIME_STEP)

#Display setup
detected_wall_display = robot.getDevice('Wall_Detection_Display')
displayWallDetection = detected_wall_display.imageLoad("BlankWallDetectionDisplay.png")
detected_wall_display.imagePaste(displayWallDetection,0,0,True)

#detect lines on ground
line_sensor = robot.getDevice('line_sensor')
line_sensor.enable(TIME_STEP)

#light sensors
light_sensor = robot.getDevice('light_sensor')
light_sensor.enable(TIME_STEP)

#detect position
inertial_unit = robot.getDevice('intertialUnit')
inertial_unit.enable(TIME_STEP)

compass = robot.getDevice('compass')
compass.enable(TIME_STEP);

#detect position
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
newPitch = round(inertial_unit.getRollPitchYaw()[2],4)
print(newPitch)
direction = 2
#left 1 top 2 right 3 bottom 4
robot.step(TIME_STEP)
startX = round(gps.getValues()[0]/0.3) - 1
startZ = round(gps.getValues()[2]/0.3)
exploring = True
nextTileX = []
nextTileZ = []
map[startX][startZ].visited = True
map[startX][startZ].seen = True
newTarget = False
while robot.step(TIME_STEP) != -1:
    
    
    updateDisplay()
    
    if exploring:
        

        if light_sensor.getValue() >= 1000:
             direction = rotate(2)
             exploring = False
             print('Found exit')
             continue
        newTarget = False
        direction = rotate(2)
        currentTileX = round(gps.getValues()[0]/0.3) - 1
        currentTileZ = round(gps.getValues()[2]/0.3)
        map[currentTileX][currentTileZ].visited = True
        print("priekis: ",ds_front.getValue()) 
        wallnumber = 0
        if ds_front.getValue() < 1000:
          
          front = True
          wallnumber = wallnumber + 1
        elif currentTileX < num_x-1 and map[currentTileX +1][currentTileZ].visited == False:
          if map[currentTileX +1][currentTileZ].seen == False:
              print("prideta ", currentTileX +1, " ", currentTileZ)
              nextTileX.append(currentTileX + 1)
              nextTileZ.append(currentTileZ)
              map[currentTileX +1][currentTileZ].seen = True
              newTarget = True
          map[currentTileX + 1][currentTileZ].wallnumber = map[currentTileX + 1][currentTileZ].wallnumber - 4
        print("desine: ",ds_right.getValue()) 
        if ds_right.getValue() < 1000:
            wallnumber = wallnumber + 2
            right = True
            
        elif currentTileZ < num_z-1 and map[currentTileX][currentTileZ+1].visited == False:
            if map[currentTileX][currentTileZ+1].seen == False:
                print("prideta ", currentTileX, " ", currentTileZ+1)
                nextTileX.append(currentTileX)
                nextTileZ.append(currentTileZ+1)
                newTarget = True
                map[currentTileX][currentTileZ+1].seen = True
            map[currentTileX][currentTileZ+1].wallnumber = map[currentTileX][currentTileZ+1].wallnumber - 8
        print("galas: ",ds_back.getValue()) 
        if ds_back.getValue() < 1000:
            wallnumber = wallnumber + 4
            bottom = True
            
        elif currentTileX > 0 and map[currentTileX-1][currentTileZ].visited == False:
              if map[currentTileX-1][currentTileZ].seen == False:
                  print("prideta ", currentTileX -1, " ", currentTileZ)
                  nextTileX.append(currentTileX -1)
                  nextTileZ.append(currentTileZ)
                  newTarget = True
                  map[currentTileX-1][currentTileZ].seen = True
              map[currentTileX-1][currentTileZ].wallnumber = map[currentTileX-1][currentTileZ].wallnumber - 1
              
        print("kaire: ",ds_left.getValue()) 
        if ds_left.getValue() < 1000:
              left = True
              
              wallnumber = wallnumber + 8
        elif currentTileZ > 0 and map[currentTileX][currentTileZ-1].visited == False: 
              if map[currentTileX][currentTileZ-1].seen == False:
                  print("prideta ", currentTileX, " ", currentTileZ-1)
                  nextTileX.append(currentTileX)
                  nextTileZ.append(currentTileZ-1)
                  map[currentTileX][currentTileZ-1].seen = True
                  newTarget = True
              map[currentTileX][currentTileZ-1].wallnumber = map[currentTileX][currentTileZ-1].wallnumber - 2
              
        map[currentTileX][currentTileZ].wallnumber = wallnumber
        
        
        # cia buvo tie ifai displayjaus
        #front = False
        #bottom = False
        #right = False
        #left = False
        
        
        
        
         
        arrived = False
        print("sienos numberis ", wallnumber)
        print( nextTileX)
        print( nextTileZ)
        if newTarget == True:

            targetX = nextTileX.pop()
            targetZ = nextTileZ.pop()
          #  print("dabartinis tile ", currentTileX, " ", currentTileZ)
         #   print("kitas tile ", targetX, " ", targetZ)
            if targetX - currentTileX == 1:
                arrived = drive_until(map, direction, currentTileX, targetX, currentTileZ, targetZ)
            elif currentTileX - targetX == 1:
                direction = rotate(4)
                arrived = drive_until(map, direction, currentTileX, targetX, currentTileZ, targetZ)
            elif targetZ - currentTileZ == 1:
                direction = rotate(3)
                arrived = drive_until(map, direction, currentTileX, targetX, currentTileZ, targetZ)
            elif currentTileZ - targetZ == 1:
                direction = rotate(1)
                arrived = drive_until(map, direction, currentTileX, targetX, currentTileZ, targetZ)
        if arrived == False:
            while arrived == False:
                robot.step(TIME_STEP)
                targetX = nextTileX.pop()
                targetZ = nextTileZ.pop()
            #    print(nextTileX)
             #   print(nextTileZ)
                target = LeeAlgo(map, currentTileX, currentTileZ, targetX, targetZ, num_x, num_z)
         #       print("naujas taikinys: " , target)
                arrived = moveToTarget(map, direction, target)
        
                
    else:
      currentTileX = round(gps.getValues()[0]/0.3) - 1
      currentTileZ = round(gps.getValues()[2]/0.3)
      target = LeeAlgo(map, currentTileX, currentTileZ, startX, startZ, num_x, num_z)
      print("naujas taikinys: " , target)
      arrived = moveToTarget(map, direction, target)
      stop()
      break          