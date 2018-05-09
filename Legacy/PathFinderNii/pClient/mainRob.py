import socket
import time
from croblink import CRobLink
from croblink import CRobLinkAngs
from math import *
from sensors import *
from mapgrid import *


def wander():
	 if cif.measures.irSensor[0]> 5.0\
		or cif.measures.irSensor[1]> 5.0\
		or cif.measures.irSensor[2]> 5.0\
		or cif.measures.irSensor[3]> 5.0:
	 #    print "Rotate"
		 cif.driveMotors(-0.1,+0.1)
	 elif cif.measures.irSensor[1]> 0.7:
		 cif.driveMotors(0.1,0.0)
	 elif cif.measures.irSensor[2]> 0.7:
		 cif.driveMotors(0.0,0.1)
	 else:
	 #    print "Go"
		 cif.driveMotors(0.1,0.1)

def rotateCounterClockwise():
	if not cif.measures.collision:
		updateRobotPos(-0.022428,0.022428)
	cif.driveMotors(-0.022428,0.022428)

def rotateClockwise():
	if not cif.measures.collision:
		updateRobotPos(0.022428,-0.022428)
	cif.driveMotors(0.022428,-0.022428)

def averagedCompass():
	if len(compassReadings) == 10:
		compassReadings.pop(0)
	compassReadings.append(cif.measures.compass)
	return reduce(lambda x,y : x + y, compassReadings)/len(compassReadings) * 1.0

def medianFront():
	if len(medianFrontReadings) == 3:
		medianFrontReadings.pop(0)
	medianFrontReadings.append(cif.measures.irSensor[0])
	if sorted(medianFrontReadings)[1] == 0:
		return 100
	return 1/sorted(medianFrontReadings)[1]

def medianLeft():
	if len(medianLeftReadings) == 3:
		medianLeftReadings.pop(0)
	medianLeftReadings.append(cif.measures.irSensor[1])
	if sorted(medianLeftReadings)[1] == 0:
		return 100
	return 1/sorted(medianLeftReadings)[1]

def medianRight():
	if len(medianRightReadings) == 3:
		medianRightReadings.pop(0)
	medianRightReadings.append(cif.measures.irSensor[2])
	if sorted(medianRightReadings)[1] == 0:
		return 100 #inifinite
	return 1/sorted(medianRightReadings)[1]

def medianBack():
	if len(medianBackReadings) == 3:
		medianBackReadings.pop(0)
	medianBackReadings.append(cif.measures.irSensor[3])
	if sorted(medianBackReadings)[1] == 0:
		return 100 #inifinite
	return 1/sorted(medianBackReadings)[1]

def crazyForward():
	errL = 0
	errR = 0
	try:
		templeftval = 1.0/leftval
	except:
		templeftval = 100
	try:
		temprightval = 1.0/rightval
	except:
		temprightval = 100

	if templeftval > 10.0:
		errL = 0.02
	elif templeftval > 5.0:
		errL = 0.02
	elif templeftval > 3.0:
		errL = 0.01
	#elif leftval > 2.0:
	#	errL = 0.005
	elif temprightval > 10.0:
		errR = 0.02
	elif temprightval > 5.0:
		errR = 0.02
	elif temprightval > 3.0:
		errR = 0.01
	#elif rightval > 2.0:
	#	errR = 0.005
	if not cif.measures.collision:
		updateRobotPos(0.1+errL, 0.1+errR)
	cif.driveMotors(0.1+errL,0.1+errR)

def stop():
	cif.driveMotors(0.0,0.0)
	updateRobotPos(0.0, 0.0)



def updateRobotPos(in_left_t, in_right_t):
	global out_left_t_last, out_right_t_last, ROBOT_POS_X, ROBOT_POS_Y, ROBOT_POS_THETA

	if out_left_t_last == None or out_right_t_last == None:
		out_left_t_last = 0#in_left_t
		out_right_t_last = 0#in_right_t

	out_left_t = in_left_t * 0.5 + out_left_t_last * 0.5
	out_right_t = in_right_t * 0.5 + out_right_t_last * 0.5
	out_left_t_last = out_left_t
	out_right_t_last = out_right_t

	diam = 1.0
	rot = (out_right_t - out_left_t) / diam


	lin = (out_right_t + out_left_t) / 2.0

	ROBOT_POS_X = ROBOT_POS_X + lin * cos(ROBOT_POS_THETA)
	ROBOT_POS_Y = ROBOT_POS_Y + lin * sin(ROBOT_POS_THETA)
	ROBOT_POS_THETA = ROBOT_POS_THETA + rot
	#always between 0 to 2pi


def printRobotPos():
	print ("X: " + str(ROBOT_POS_X) + " Y: " + str(ROBOT_POS_Y) + " THETA: " + str(ROBOT_POS_THETA) + " COLLISION: " + str(cif.measures.collision))

def calcDesiredRotation(amount, direction):

	final_angle = theta_before_rotation % amount
	extra = 0
	if direction == "ccw":
		if (amount - final_angle) < 0.20 * amount:
			extra = amount
		#print "CCW - Since I'm at " + str(theta_before_rotation) + " I need to rotate " + str(amount - final_angle + extra) + " RESULT: " + str(theta_before_rotation + amount - final_angle + extra)
		return (amount - final_angle + extra - 0.07) #- 0.07
	elif direction == "cw":
		if final_angle < 0.20 * amount:
			extra = amount
		#print "CW - Since I'm at " + str(theta_before_rotation) + " I need to rotate " + str(-final_angle - extra) + " RESULT: " + str(theta_before_rotation - final_angle - extra)
		return (-final_angle - extra + 0.07) #- 0.07

def getActualCellCoordinates():
	# Devolve as coordenadas da celula actual (sendo 0,0 o ponto em que o rato inicia, no centro da celula)
	x = int(ROBOT_POS_X / 2.0)
	y = int(ROBOT_POS_Y / 2.0)
	if abs(ROBOT_POS_X) % 2.0 >= 1.0:
		if ROBOT_POS_X < 0:
			x = x - 1
		else:
			x = x + 1
	if abs(ROBOT_POS_Y) % 2.0 >= 1.0:
		if ROBOT_POS_Y < 0:
			y = y - 1
		else:
			y = y + 1
	return (x+8,y+15)

def getOrientation():	# Devolve a orientacao actual do rato (Norte, Sul, Este, Oeste) em relacao ao ponto de vista do observador no simulador
	t = degrees(ROBOT_POS_THETA) % 360
	if (t >= 0 and t <= 45) or (t > 315 and t < 360):
		return "E"
	elif (t > 45 and t <= 135):
		return "N"
	elif (t > 135 and t <= 225):
		return "W"
	else:
		return "S"


def actionNeeded(my_position):	# Decide se e necessario o rato efectuar alguma curva ou parar. Em caso contrario, mantem o mesmo rumo.
	#my_position = getActualCellCoordinates()
	my_orientation = getOrientation()
	if len(nextCells) == 0:
		return "STOP"
	#print ("Desired:", nextCells[0])
	desired_cell = nextCells[0]
	if my_position == desired_cell:
		#print ("Attained!")
		nextCells.pop(0)
		if len(nextCells) == 0:
			return "STOP"
		desired_cell = nextCells[0]
	if desired_cell[1] > my_position[1]:
		if my_orientation == "N":
			return -3.14/2
		elif my_orientation == "W":
			return 3.14
		elif my_orientation == "S":
			return 3.14/2
		else:
			return "NO"
	elif desired_cell[1] < my_position[1]:
		if my_orientation == "N":
			return 3.14/2
		elif my_orientation == "E":
			return -3.14
		elif my_orientation == "S":
			return -3.14/2
		else:
			return "NO"
	elif desired_cell[0] < my_position[0]:
		if my_orientation == "E":
			return 3.14/2
		elif my_orientation == "S":
			return 3.14
		elif my_orientation == "W":
			return -3.14/2
		else:
			return "NO"
	elif desired_cell[0] > my_position[0]:
		if my_orientation == "E":
			return -3.14/2
		elif my_orientation == "N":
			return -3.14
		elif my_orientation == "W":
			return 3.14/2
		else:
			return "NO"



def recalibrateXY():
	global ROBOT_POS_X, ROBOT_POS_Y
	orientation = getOrientation()
	if orientation == "S" or orientation == "N":
		current_odo = ROBOT_POS_Y
	else:
		current_odo = ROBOT_POS_X

	center_to_wall = frontval + 0.5
	deduced_err = center_to_wall - 1
	err = abs(current_odo) % 2
	if err < 1.0:
		current_odo = current_odo - err + err/2.0 + deduced_err/2.0
		sys.stderr.write("Recalibration change: " + str((err/2.0 + deduced_err/2.0) - err) + "\n")
	else:
		current_odo = current_odo - err + err/2.0 + (2.0-deduced_err)/2
		sys.stderr.write("Recalibration change: " + str((err/2.0 + (2.0-deduced_err)/2.0) - err) + "\n")

	if orientation == "S" or orientation == "N":
		ROBOT_POS_Y = current_odo
	else:
		ROBOT_POS_X = current_odo


def calibrateTheta():
	global ROBOT_POS_THETA
	compass_reading = cif.measures.compass
	if compass_reading < 0:
		compass_reading = 180 + (180 - abs(compass_reading))
	odo_reading = degrees(ROBOT_POS_THETA) % 360
	if abs(odo_reading - compass_reading) > 180:
		if odo_reading < 180:
			final = (compass_reading * 0.1 + (odo_reading+360) * 0.9) % 360
		elif compass_reading < 180:
			final = ( (compass_reading+360) * 0.1 + odo_reading * 0.9) % 360
	else:
		final = compass_reading * 0.1 + odo_reading * 0.9

	ROBOT_POS_THETA = radians(final)


def has_time():
	#map a*star (cur_pos, cheese) + (cheese, start)
	
	cur_pos = (abs(divide(ROBOT_POS_Y) - 8 ) , divide(ROBOT_POS_X) + 15)
	path_to_cheese = list(MazeSolver(map_).astar(cur_pos, pos_cheese))
	path_to_start = list(MazeSolver(map_).astar(pos_cheese, pos_start))
	TIME_PER_CELL = 55*0.4 + 20*(1-0.4)

	finalTime = 5000
	time = cif.measures.time


	return  (len(path_to_cheese) + len(path_to_start))*TIME_PER_CELL < (finalTime-time)

path_to_cheese = []
path_to_start = []
# O nextCells contem a lista de pontos que o rato vai tentar atingir
# Observa sempre apenas o primeiro elemento e, quando atingido, elimina-o da lista
#nextCells = [(9, 15), (10, 15), (11, 15), (12, 15), (12, 14), (13, 14), (13, 13), (14, 13), (15, 13), (16, 13)]
#nextCells = [(9, 15), (10, 15), (10, 16), (11, 16), (12, 16), (13, 16), (14, 16), (15, 16), (15, 15), (15, 14), (15, 13),(14, 13), (13, 13), (13, 14), (12, 14), (12, 15), (11, 15), (10, 15), (9, 15), (8, 15) ]
nextCells = []
requestedCells = []

cif=CRobLinkAngs("Gerald",0,[0.0, 90.0, -90.0, 180], sys.argv[1])
if cif.status!=0:
	print ("Connection refused or error")
	quit()

map_ = Maze(17, 31) #com margens de erro
s_ = Sensors()
found_cheese = False
pos_cheese = (0,0)
pos_start = (8,15)
RUN=1
WAIT=2
RETURN=3
ROTATECW=4
ROTATECCW=5
STOP=6
FORWARD=7

EXPLORE = 8
GO_TO_CHEESE = 9
GO_TO_START = 10
START = 11
state = FORWARD
sim_state = START

out_left_t_last = None
out_right_t_last = None


ROBOT_POS_X = 0
ROBOT_POS_Y = 0
ROBOT_POS_THETA = 0
last_rotation_coords = [100, 100]
desired_rotation = 0
theta_before_rotation = 0
lastCell_x = 0
lastCell_y = 0
compassReadings = []
medianFrontReadings = [0,0,0]
medianLeftReadings = [0,0,0]
medianRightReadings = [0,0,0]
medianBackReadings = [0,0,0]

inMiddle = False

while 1:

	cif.readSensors()
	frontval = medianFront()
	leftval = medianLeft()
	rightval = medianRight()
	backval = medianBack()
	if cif.measures.ground==0:
		found_cheese = True
		g_y = divide(ROBOT_POS_X) + 15
		g_x = abs(divide(ROBOT_POS_Y) - 8 )
		pos_cheese = (g_x, g_y)
		cif.setVisitingLed(1);
	else:
		cif.setVisitingLed(0);

	#detetar o cheese
	#
	#

	if cif.measures.time == 0:
		continue

	#------------simulation state machine--------------
	if sim_state == START :
		#read 3 times and then  find_next_cell

		if len(medianBackReadings) < 3:
			continue
		else:
			sim_state = EXPLORE
			print "state - EXPLORE"

	# elif sim_state == EXPLORE :
	# 	#lastCell_x, lastCell_y = s_.update_map_v3(map_, ROBOT_POS_X ,ROBOT_POS_Y, (ROBOT_POS_THETA) % (2*pi) , frontval, leftval, backval, rightval  )
	# 	if found_cheese and not has_time() :
	# 		sim_state = GO_TO_CHEESE
	# 		nextCells = path_to_cheese[1::]

	# elif sim_state == GO_TO_CHEESE:
	# 	if nextCells ==[]:
	# 		#acender o led
	# 		sim_state = GO_TO_START
	# 		nextCells = path_to_start[1::]
	if sim_state == GO_TO_START:
		cif.setReturningLed(1)

	elif sim_state == STOP:
		stop()
		print("FINISHED")
	#-------------------------------------------------#
	#---------------NAVIGATION state machine----------#
	# if state==RUN:
	# 	if cif.measures.visitingLed==1:
	# 		state=WAIT
	# 	if cif.measures.ground==0:
	# 		cif.setVisitingLed(1);
	# 	wander()
	# elif state==WAIT:
	# 	cif.setReturningLed(1)
	# 	if cif.measures.visitingLed==1:
	# 		cif.setVisitingLed(0)
	# 	if cif.measures.returningLed==1:
	# 		 state=RETURN
	# 	cif.driveMotors(0.0,0.0)
	# elif state==RETURN:
	# 	if cif.measures.visitingLed==1:
	# 		cif.setVisitingLed(0)
	# 	if cif.measures.returningLed==1:
	# 		cif.setReturningLed(0)
	# 	wander()

	if state==ROTATECW:	# Neste estado, o rato efectua uma rotacao no sentido horario

		rotateClockwise()
		state = ROTATECW

		if ROBOT_POS_THETA - theta_before_rotation <= desired_rotation:
			state = FORWARD
			desired_rotation = 0
			continue

	elif state==ROTATECCW:	# Neste estado, o rato efectua uma rotacao no sentido anti-horario
		rotateCounterClockwise()
		state = ROTATECCW

		if ROBOT_POS_THETA - theta_before_rotation >= desired_rotation:
			state = FORWARD
			desired_rotation = 0
			continue

	elif state==STOP:	# Este estado deve ser atingido quando a missao terminou
		stop()
		#print(map_.drawMaze())
		state = STOP

	elif state==FORWARD:
		calibrateTheta()
		orientation = getOrientation()
		if orientation == "S" or orientation == "N": # Se estamos a navegar verticalmente, e mais facil utilizar o Y da odometria
													 # para detectar quando ultrapassamos o centro de uma celula
			axis_to_use = ROBOT_POS_Y
		else:
			axis_to_use = ROBOT_POS_X

		if abs(axis_to_use % 2) < 0.1 or (1/frontval) >= 1.3: # Estamos no centro de uma celula?
			if (1/frontval) >= 1.3:
				recalibrateXY()
			g_y = divide(ROBOT_POS_X) + 15
			g_x = abs(divide(ROBOT_POS_Y) - 8 )
			#print ("time", cif.measures.time )
			if sim_state == EXPLORE :
				#mapping
				if (lastCell_x != g_x or lastCell_y != g_y):
					print("rx, ry", ROBOT_POS_X , ROBOT_POS_Y , degrees(ROBOT_POS_THETA), " | rightval", rightval , " | left", leftval , " | frontval", frontval, " | back", backval )
					#lastCell_x, lastCell_y = s_.update_map_v2(map_, ROBOT_POS_X ,ROBOT_POS_Y, ROBOT_POS_THETA , frontval, leftval, backval, rightval  )
					lastCell_x, lastCell_y = s_.update_map_v3(map_, ROBOT_POS_X ,ROBOT_POS_Y, (ROBOT_POS_THETA)% (2*pi) , frontval, leftval, backval, rightval  )
					file_ = open('map.txt', 'w')
					file_.write(map_.drawMaze())
					file_.close()

					file_ = open('map_vis.txt', 'w')
					file_.write(map_.drawMaze_visited())
					file_.close()

				if found_cheese and not has_time() :
					if g_x == pos_cheese[0] and g_y == pos_cheese[1]:
						sim_state = GO_TO_START
						try:
							path_to_start = list(MazeSolver(map_).astar(pos_cheese, pos_start))
							nextCells = path_to_start[1::]
						except Exception, e:
							print "ERROR! : CANNOT FIND ANY PATHS TO START POSITION"
						else:
							file_ = open('map.txt', 'w')
							file_.write(map_.drawMaze(list(path_to_start)))
							file_.close()
					else:
						sim_state = GO_TO_CHEESE
						path_to_cheese = list(MazeSolver(map_).astar((g_x, g_y), pos_cheese))
						print( pos_cheese , path_to_cheese)
						nextCells = path_to_cheese[1::]
						print "GO_TO_CHEESE #####################################"
						print map_.drawMaze(list(path_to_cheese))

			#navigation
			# if (1/frontval) >= 1.1:
			# 	sys.stderr.write("Rotating by sensors. " + str(axis_to_use) + "\n")

			action = actionNeeded((g_x, g_y)) # Verificar se e necessario curvar ou parar
			if action == "STOP" and sim_state == EXPLORE: #no more cells
				sys.stderr.write("Calling findNext at " + str((g_x, g_y)) + "visited: " + str(map_.maze[g_x][g_y].vis) + "(" + "%.20f" % time.time() +")\n")


				requestedCells = map_.findNext( (g_x, g_y) , ROBOT_POS_THETA)
				if(len(requestedCells) < 1 ):
					print "IS EXPLORED"
					if found_cheese :
						has_time()
						sim_state = GO_TO_CHEESE
						print "GO_TO_CHEESE #####################################2"
						try:
							path_to_cheese = list(MazeSolver(map_).astar((g_x, g_y), pos_cheese))
							nextCells = path_to_cheese[1::]
						except Exception, e:
							print "ERROR! : CANNOT FIND ANY PATHS TO CHEESE POSITION"
						else:
							action = actionNeeded((g_x, g_y))
					else:
						print "ERROR! : CANNOT FIND ANY CHEESE POSITION"
						state = STOP
						stop()
				elif len(requestedCells) == 1:
					lastCell_y = -1
					lastCell_x = -1
					nextCells = requestedCells
					continue

				else:
					nextCells = requestedCells[1::]
					action = actionNeeded((g_x, g_y))
					sys.stderr.write("Cell from findNext("+ str(len(requestedCells)) +"): " + str(requestedCells[1]) + "\n") #+ " Desired cell: " + str(requestedCells[1]) + "\n")


			print ("Action needed:", action)
			if action == "STOP" and sim_state == GO_TO_CHEESE:
				sim_state = GO_TO_START
				print "GO_TO_START #####################################"
				try:
					path_to_start = list(MazeSolver(map_).astar(pos_cheese, pos_start))
					nextCells = path_to_start[1::]
				except Exception, e:
					print "ERROR! : CANNOT FIND ANY PATHS TO START POSITION"
				else:
					file_ = open('map.txt', 'w')
					file_.write(map_.drawMaze(list(path_to_start)))
					file_.close()

			elif action == "STOP" :
				#print (path_to_start, path_to_cheese)
				state = STOP
				continue
			elif action == "NO":
				pass
			elif action > 0: # E necessario rodar no sentido anti-horario
				theta_before_rotation = ROBOT_POS_THETA
				desired_rotation = calcDesiredRotation(action, "ccw")
				state = ROTATECCW
				continue
			else:	# E necessario rodar no sentido horario
				theta_before_rotation = ROBOT_POS_THETA
				desired_rotation = calcDesiredRotation(-action, "cw")
				state = ROTATECW
				continue

		crazyForward()
		state = FORWARD
