from controller import Robot
from controller import Compass
from controller import GPS
from controller import InertialUnit

class Tile:
	visited = False
	seen = False
	wallnumber = 15  	#"Sienos numeris", gaunamas: Siena priekyje = +1, siena dešinėje = +2, siena gale = +4, siena dešinėje = +8
	wallnumberlines = 0  #Papildomas skaičius linijoms ant žemės
	distance = -1
	def __init__(self):
    	pass

def LeeAlgo(map, startx, starty, endx, endy, maxx, maxy):   #Kelio radimo algoritmas, remiantis Lee algoritmu
	tile = map[startx][starty]
	tile.distance = 0
	LeeAlgoRec(map, startx, starty, maxx, maxy)	#Kviečiame metodą pradžios langeliui

	target = GetListToTile(map, endx, endy, startx, starty, maxx, maxy) #metodas, gražinantis sąrašą krypčių iki pabaigos langelio
	for Tile in map:
    	for Tile2 in Tile:
        	Tile2.distance = -1  #Išvalome atstumus, kad galėtume vėliau vėl naudoti šį metodą
	return target

def GetListToTile(map, endx, endy, startx, starty, maxx, maxy): 	#Metodas, gražinantis sąrašą krypčių iki pabaigos langelio
	target = []
	distance = map[endx][endy].distance  #Pabaigos atstumas
	x = endx
	y = endy
	while distance > 0:  #Eisime per langelius kol pasieksime pradžią
    	robot.step(TIME_STEP)
    	tile = map[x][y]
    	wallnumber = tile.wallnumber + tile.wallnumberlines  #langelio "sienos numeris"
    	if wallnumber >= 8: 	#Siena kairėje
        	wallnumber = wallnumber - 8   #Išsiskaičiuosime kitas sienas, pašalinę šią
    	elif y > 0 and map[x][y - 1].distance < tile.distance:	#sienos nėra, pasižiūrime atstumą į langelį kairėje
        	target.append('E')       	#jei geras atstumas, įsidedame kryptį į dabartinį langelį į sąrašą
        	distance = map[x][y - 1].distance
        	x = x   	#kito langelio koordinatės
        	y = y - 1
        	continue	#Einame per šį naują langelį
    	if wallnumber >= 4:   #Jei prieš tai buvusi kryptis netiko, tikriname sieną apačioje
        	wallnumber = wallnumber - 4
    	elif x > 0 and map[x - 1][y].distance < tile.distance:   #tas pats, langelis apačioje
        	target.append('N')
        	distance = map[x - 1][y].distance
        	x = x - 1
        	y = y
        	continue
    	if wallnumber >= 2:	#Siena dešinėje
        	wallnumber = wallnumber - 2
    	elif y < maxy - 1 and map[x][y + 1].distance < tile.distance:	#langelis dešinėje
        	target.append('W')
        	distance = map[x][y + 1].distance
        	x = x
        	y = y + 1
        	continue
    	if wallnumber >= 1:   	#Siena viršuje
        	wallnumber = wallnumber - 1
    	elif x < maxx - 1 and map[x + 1][y].distance < tile.distance:   #langelis viršuje
        	target.append('S')
        	distance = map[x + 1][y].distance
        	x = x + 1
        	y = y
        	continue
	return target[::-1]   	#kadangi ėjome iš galo į pradžią, apsukame krypčių sąrašą


def LeeAlgoRec(map, x, y, maxx, maxy):      	#rekursinis atstumo iki langelio skaičiavimas
	tile = map[x][y]
	wallnumber = tile.wallnumber + tile.wallnumberlines  #Langelio "Sienos numeris"
	if tile.seen:     	#Jeigu langelis buvo matytas, į jį yra žinomas patekimas
    	N = False
    	E = False
    	S = False
    	W = False
    	if wallnumber >= 8:       	#Siena kairėje
        	wallnumber = wallnumber - 8   #atimame sieną kairėje, taip galėsime patikrinti kitas sienas
    	elif y > 0 and (map[x][y - 1].distance == -1 or map[x][y - 1].distance > tile.distance + 1) and map[x][y - 1].seen:  #atstumas iki langelio kairėje nežinomas arba didesnis už dabartinio +1
        	map[x][y - 1].distance = tile.distance + 1   #atnaujiname atstumą iki langelio kairėje
        	W = True  	#pasižymime kad langelis kairėje yra dar nesutvarkytas

    	if wallnumber >= 4:	#Taip pat patikriname sieną apačioje
        	wallnumber = wallnumber - 4
    	elif x > 0 and (map[x - 1][y].distance == -1 or map[x - 1][y].distance > tile.distance + 1) and map[x - 1][y].seen:
        	map[x - 1][y].distance = tile.distance + 1  
        	S = True 	 

    	if wallnumber >= 2:  #Siena dešinėje
        	wallnumber = wallnumber - 2
    	elif y < maxy - 1 and (map[x][y + 1].distance == -1 or map[x][y + 1].distance > tile.distance + 1) and map[x][y + 1].seen:
        	map[x][y + 1].distance = tile.distance + 1
        	E = True

    	if wallnumber >= 1: #siena viršuje
        	wallnumber = wallnumber - 1
    	elif x < maxx - 1 and (map[x + 1][y].distance == -1 or map[x + 1][y].distance > tile.distance + 1) and map[x + 1][y].seen:
        	map[x + 1][y].distance = tile.distance + 1
        	N = True

    	if W:    	#rekursiškai kviečiame pažymėtams langeliams
        	LeeAlgoRec(map, x, y - 1, maxx, maxy)
    	if S:
        	LeeAlgoRec(map, x - 1, y, maxx, maxy)
    	if E:
        	LeeAlgoRec(map, x, y + 1, maxx, maxy)
    	if N:
        	LeeAlgoRec(map, x + 1, y, maxx, maxy)

def moveToTarget(map, direction, target):	#važiavimas pagal kryptis
	arrived = True
	if not target:    	#nėra krypčių pasiekti taikinį (galbūt už žemėlapio ribų)
    	arrived = False
	for dir in target:  	#eisime per kryptis
		updateDisplay()
    	if arrived == True:
        	currentTileX = round(gps.getValues()[0] / 0.3) - 1  #gauname dabartines koordinates
        	currentTileZ = round(gps.getValues()[2] / 0.3)
        	arrived = False   #naudojama pasitikrinimui, ar sėkmingai pavyksta važiuoti
        	if dir == 'N':   #bandome važiuoti nurodyta kryptimi
            	rotate(2)
            	arrived = drive_until(map, direction, currentTileX, currentTileX + 1, currentTileZ, currentTileZ)  
        	if dir == 'E':
            	rotate(3)
            	arrived = drive_until(map, direction, currentTileX, currentTileX, currentTileZ, currentTileZ + 1)
        	if dir == 'S':
            	rotate(4)
            	arrived = drive_until(map, direction, currentTileX, currentTileX - 1, currentTileZ, currentTileZ)
        	if dir == 'W':
            	rotate(1)
            	arrived = drive_until(map, direction, currentTileX, currentTileX, currentTileZ, currentTileZ - 1)
	return arrived

def drive_forward():  #Paprastas važiavimas tiesiai
	leftSpeed = 1.0
	rightSpeed = 1.0
	wheelFL.setVelocity(leftSpeed)
	wheelBL.setVelocity(leftSpeed)
	wheelFR.setVelocity(rightSpeed)
	wheelBR.setVelocity(rightSpeed)


def drive_until(map, direction, currentTileX, xpos, currentTileZ, zpos):  #važiavimas į kitą langelį
	startX = currentTileX   #pasižymime pradines langelio korodinates
	startZ = currentTileZ
	foundLine = False	#tam atvejui jei ant linijos užvažiuos
	while (abs(currentTileX - xpos) > 0.15 or abs(currentTileZ - zpos) > 0.15) and foundLine == False:   #tikriname su paklaida
    	updateDisplay()
		currentTileX = round(gps.getValues()[0] / 0.3, 2) - 1
    	currentTileZ = round(gps.getValues()[2] / 0.3, 2)
    	robot.step(TIME_STEP)
    	if line_sensor.getValue() > 500.0:  #jei pamatė liniją
        	foundLine = True
    	drive_forward()
	stop()
	if foundLine == True:	#Užvažiavo ant linijos
    	currentTileX = round(gps.getValues()[0] / 0.3) - 1
    	currentTileZ = round(gps.getValues()[2] / 0.3)
    	if direction == 1:  	#apsisukame
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
    	currentTileX = round(gps.getValues()[0] / 0.3, 2) - 1
    	currentTileZ = round(gps.getValues()[2] / 0.3, 2)
    	drive_until(map, direction, currentTileX, startX, currentTileZ, startZ)  #Bandome važiuoti atgal į langelio vidurį
    	return False   #planuoto tikslo nepasiekėme
	else:
    	return True   #sėkmingai atvažiavome


def stop():	#roboto sustabdymas
	leftSpeed = 0
	rightSpeed = 0
	wheelFL.setVelocity(leftSpeed)
	wheelBL.setVelocity(leftSpeed)
	wheelFR.setVelocity(rightSpeed)
	wheelBR.setVelocity(rightSpeed)

def updateDisplay():
    #set variables:
    front = False
    bottom = False
    right = False
    left = False
    #check wall sensors values 
    # if sensor value less than 1000, it means our robots sensor "sees" the wall
    if ds_front.getValue() < 1000: 
        front = True
    if ds_back.getValue() < 1000:
        bottom = True
    if ds_left.getValue() < 1000:
        left = True
    if ds_right.getValue() < 1000:
        right = True
    
    
    #check out variables and display related image
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
            displayWallDetection = detected_wall_display.imageLoad("TopAndRightWallDetectionDisplay.png")
            detected_wall_display.imagePaste(displayWallDetection,0,0,True)
    if front and not bottom and not right and left:
            displayWallDetection = detected_wall_display.imageLoad("LeftAndTopWallDetectionDisplay.png")
            detected_wall_display.imagePaste(displayWallDetection,0,0,True)
    if not front and bottom and right and not left:
            displayWallDetection = detected_wall_display.imageLoad("RightAndBottomWallDetectionDisplay.png")
            detected_wall_display.imagePaste(displayWallDetection,0,0,True)
    if front and not bottom and right and left:
            displayWallDetection = detected_wall_display.imageLoad("LeftRightTopWallDetectionDisplay.png")
            detected_wall_display.imagePaste(displayWallDetection,0,0,True)
                    
    if not front and bottom and not right and left:
            displayWallDetection = detected_wall_display.imageLoad("LeftAndBottomWallDetectionDisplay.png")
            detected_wall_display.imagePaste(displayWallDetection,0,0,True)
            
            
    
                
    if front and bottom and right and  not left:
            displayWallDetection = detected_wall_display.imageLoad("RightTopAndBottomWallDetectionDisplay.png")
            detected_wall_display.imagePaste(displayWallDetection,0,0,True)
    if front and bottom and not right and left:
            displayWallDetection = detected_wall_display.imageLoad("LeftTopAndBottomWallDetectionDisplay.png")
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
def rotate(direction):  #sukimosi metodas į tam tikrą pusę
	done = False
	while done == False:  #dar neapsisuko
		updateDisplay()
    	robot.step(TIME_STEP)
    	delta = 0.05  #posukio paklaidos dydis
    	currentPitch = round(inertial_unit.getRollPitchYaw()[2], 4) #roboto pasisukimo pozicija radianais
    	if direction == 1 and currentPitch > -1.5708 * 2 + delta and currentPitch < 1.5708 * 2 - delta: #tikriname ar robotas pasisukes į kairę, jei ne sukamės toliau
        	turn_right()
    	elif direction == 2 and (currentPitch > 1.5708 + delta or currentPitch < 1.5708 - delta): #tikriname ar robotas pasisukes į viršų, jei ne sukamės toliau
        	turn_right()
    	elif direction == 3 and (currentPitch > 0 + delta or currentPitch < 0 - delta): #tikriname ar robotas pasisukes į dešinę, jei ne sukamės toliau
        	turn_right()
    	elif direction == 4 and (currentPitch > -1.5708 + delta or currentPitch < -1.5708 - delta): #tikriname ar robotas pasisukes į apačia, jei ne sukamės toliau
        	turn_right()
    	else:
        	done = True
	stop()
	return direction


def turn_right():	#Nustatymas suktis į dešinę pusę
	leftSpeed = 1.0
	rightSpeed = -1.0
	wheelFL.setVelocity(leftSpeed)
	wheelBL.setVelocity(leftSpeed)
	wheelFR.setVelocity(rightSpeed)
	wheelBR.setVelocity(rightSpeed)


def print_current_dir(x):

	# left 1 top 2 right 3 bottom 4

	if x == 1:
    	print 'left'
	if x == 2:
    	print 'up'
	if x == 3:
    	print 'right'
	if x == 4:
    	print 'bottom'


TIME_STEP = 64
robot = Robot()

num_x = 7 #labirinto x kraštinės langelių skaičius
num_z = 7 #labirinto z kraštinės langelių skaičius

map = [[Tile() for x in range(num_x)] for y in range(num_z)]  #pradinis žemėlapio sudarymas

ds_front = robot.getDevice('distance_sensor_front')
ds_front.enable(TIME_STEP)
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
updateDisplay()

#sensoriu priskyrimas
# detect lines on ground
line_sensor = robot.getDevice('line_sensor')
line_sensor.enable(TIME_STEP)

# light sensors
light_sensor = robot.getDevice('light_sensor')
light_sensor.enable(TIME_STEP)

# detect position of rotation
inertial_unit = robot.getDevice('intertialUnit')
inertial_unit.enable(TIME_STEP)

# detect position
gps = robot.getDevice('gps')
gps.enable(TIME_STEP)

# wheels settup
# get wheels

wheelFL = robot.getDevice('FLWHEEL')
wheelBL = robot.getDevice('BLWHEEL')
wheelFR = robot.getDevice('FRWHEEL')
wheelBR = robot.getDevice('BRWHEEL')

# set position
wheelFL.setPosition(float('inf'))
wheelBL.setPosition(float('inf'))
wheelFR.setPosition(float('inf'))
wheelBR.setPosition(float('inf'))

# set speed
wheelFL.setVelocity(0.0)
wheelBL.setVelocity(0.0)
wheelFR.setVelocity(0.0)
wheelBR.setVelocity(0.0)

direction = 2
robot.step(TIME_STEP)

startX = round(gps.getValues()[0] / 0.3) - 1   #pradinės koordinatės
startZ = round(gps.getValues()[2] / 0.3)

exploring = True  	#Ar dar ieškome pabaigos?
nextTileX = []	#Stack'as langelių, į kuriuos dar reikia užsukti
nextTileZ = []
map[startX][startZ].visited = True   #pasižymime pradinį langelį
map[startX][startZ].seen = True
newTarget = False 	#Jei naujas langelis bus pridėtas, jam nereikia kviesti kelio radimo algoritmo
while robot.step(TIME_STEP) != -1:

	updateDisplay()
	
	if exploring: 	#Dar neradome pabaigos
    	if light_sensor.getValue() >= 1000:  #aptiko pabaigą
        	direction = rotate(2)
        	exploring = False
        	print 'Found exit'
        	continue  #langelio nebetikriname
    	newTarget = False
    	direction = rotate(2)   #atsisukame į šiaurę
    	currentTileX = round(gps.getValues()[0] / 0.3) - 1   #gauname dabartines koordinates
    	currentTileZ = round(gps.getValues()[2] / 0.3)
    	map[currentTileX][currentTileZ].visited = True   #pažymime dabartinį langelį
    	wallnumber = 0	#Skaičiuosime "Sienos numerį"
    	if ds_front.getValue() < 1000:  	#siena priekyje, +1
        	wallnumber = wallnumber + 1
    	elif currentTileX < num_x - 1 and map[currentTileX+ 1][currentTileZ].visited == False:   #sienos nėra, ar langelis priekyje dar neaplankytas
        	if map[currentTileX + 1][currentTileZ].seen == False:   #jeigu langelis dar nematytas, pridedame jo koordinates į stack'ą
            	nextTileX.append(currentTileX + 1)
            	nextTileZ.append(currentTileZ)
            	map[currentTileX + 1][currentTileZ].seen = True  	#pasižymime, kad tą langelį jau matėme
            	newTarget = True    	#buvo aptiktas naujas langelis
        	map[currentTileX + 1][currentTileZ].wallnumber = map[currentTileX + 1][currentTileZ].wallnumber - 4   #Atnaujiname to naujo langelio "Sienos numerį" (nuimame apatinę sieną)
    	if ds_right.getValue() < 1000:  	#Taip pat darome su siena dešinėje
        	wallnumber = wallnumber + 2  	#siena dešinėje, +2
    	elif currentTileZ < num_z - 1 and map[currentTileX][currentTileZ + 1].visited == False:  #sienos nėra, tą patį darome su langeliu dešinėje
        	if map[currentTileX][currentTileZ + 1].seen == False:
            	print ('prideta ', currentTileX, ' ', currentTileZ + 1)
            	nextTileX.append(currentTileX)
            	nextTileZ.append(currentTileZ + 1)
            	newTarget = True
            	map[currentTileX][currentTileZ + 1].seen = True
        	map[currentTileX][currentTileZ + 1].wallnumber = map[currentTileX][currentTileZ + 1].wallnumber - 8
    	if ds_back.getValue() < 1000:   #taip pat su siena ir langeliu gale
        	wallnumber = wallnumber + 4   #siena apačioje, +4
    	elif currentTileX > 0 and map[currentTileX - 1][currentTileZ].visited == False:
        	if map[currentTileX - 1][currentTileZ].seen == False:
            	print ('prideta ', currentTileX - 1, ' ', currentTileZ)
            	nextTileX.append(currentTileX - 1)
            	nextTileZ.append(currentTileZ)
            	newTarget = True
            	map[currentTileX - 1][currentTileZ].seen = True
        	map[currentTileX - 1][currentTileZ].wallnumber = map[currentTileX - 1][currentTileZ].wallnumber - 1
    	if ds_left.getValue() < 1000:  #galiausiai siena ir langelis kairėje
        	wallnumber = wallnumber + 8  #siena kairėje, +8
    	elif currentTileZ > 0 and map[currentTileX][currentTileZ - 1].visited == False:
        	if map[currentTileX][currentTileZ - 1].seen == False:
            	print ('prideta ', currentTileX, ' ', currentTileZ - 1)
            	nextTileX.append(currentTileX)
            	nextTileZ.append(currentTileZ - 1)
            	map[currentTileX][currentTileZ - 1].seen = True
            	newTarget = True
        	map[currentTileX][currentTileZ - 1].wallnumber = map[currentTileX][currentTileZ - 1].wallnumber - 2
    	map[currentTileX][currentTileZ].wallnumber = wallnumber  #atnaujiname dabartinio langelio "Sienos numerį"
    	arrived = False   #Pasižymime kad dar nepasiekėme tikslo
    	if newTarget == True:   	#jeigu buvo pridėtas naujas langelis, jam nereikia kviesti kelio radimo, nes jis bus gretimas dabartiniam langeliui
        	targetX = nextTileX.pop()    
        	targetZ = nextTileZ.pop()
        	if targetX - currentTileX == 1:  #randame kurioje pusėje yra tas naujas langelis ir ten važiuojame
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
    	if arrived == False:     	#jeigu nepavyko atvažiuoti į naują langelį arba nebuvo pridėtas naujas langelis
        	while arrived == False:  #Bandysime kviesti kelio radimą sekančiam langeliui stack'e ir ten važiuosim, tai kartosim kol pavyks pasiekti naują langelį
            	robot.step(TIME_STEP)
            	targetX = nextTileX.pop()
            	targetZ = nextTileZ.pop()
            	target = LeeAlgo(map, currentTileX, currentTileZ, targetX, targetZ, num_x, num_z)  #apsiskaičiuojame atstumus ir gauname krypčių sąrašą
            	arrived = moveToTarget(map, direction, target)  #bandome važiuoti nurodytomis kryptimis
	else: 	#buvo rasta pabaiga
    	currentTileX = round(gps.getValues()[0] / 0.3) - 1    	#gauname dabartines koordinates
    	currentTileZ = round(gps.getValues()[2] / 0.3)
    	target = LeeAlgo(map, currentTileX, currentTileZ, startX, startZ, num_x, num_z)  #kviečiame kelio radimą starto langeliui
    	arrived = moveToTarget(map, direction, target)  #važiuojame į pradžią
    	stop()
    	break	#baigiame darbą

