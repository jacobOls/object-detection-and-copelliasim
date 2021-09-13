import sim
import sys
import numpy as np
import array
import time
import cv2
import math
##object and distance detection combined with simple movement code in copelliasim for testing purposes.
#todo change object detection to single function with parameters to clean up space. make more advanced movement code and better decision making.
#constants
BALL_SIZE = 6.3
BALL_FOCAL = (330 * 8) / BALL_SIZE
TOWER_SIZE = 4
TOWER_FOCAL = (100 * 30) / TOWER_SIZE
TEAM = "red"
#init
sim.simxFinish(-1)
clientID = sim.simxStart('127.0.0.1',19999,True,True,5000,20)
score = 0
#connection test to sim
if clientID != -1:
    print ('connected to remote API server')

else:
    print ('connection failed')
    sys.exit('could not connect')

##object handles
errorCode,robot_handle = sim.simxGetObjectHandle(clientID, 'Omar',sim.simx_opmode_oneshot_wait)
errorCode,field_handle = sim.simxGetObjectHandle(clientID, 'Field',sim.simx_opmode_oneshot_wait)
errorCode,field_wall_handle = sim.simxGetObjectHandle(clientID, 'Field_wall4', sim.simx_opmode_oneshot_wait)
errorCode, ram_handle = sim.simxGetObjectHandle(clientID,'Raming_bar',sim.simx_opmode_oneshot_wait)
    #drive handles
errorCode,front_left_motor_handle = sim.simxGetObjectHandle(clientID,'rollingJoint_fl',sim.simx_opmode_oneshot_wait)
errorCode,back_left_motor_handle = sim.simxGetObjectHandle(clientID,'rollingJoint_rl',sim.simx_opmode_oneshot_wait)
errorCode,back_right_motor_handle = sim.simxGetObjectHandle(clientID,'rollingJoint_rr',sim.simx_opmode_oneshot_wait)
errorCode,front_right_motor_handle = sim.simxGetObjectHandle(clientID,'rollingJoint_fr',sim.simx_opmode_oneshot_wait)
    #vision handles
res,v1=sim.simxGetObjectHandle(clientID,'Front_camera',sim.simx_opmode_oneshot_wait)
res2,v2 = sim.simxGetObjectHandle(clientID,'Left_camera',sim.simx_opmode_oneshot_wait)
res3,v3 = sim.simxGetObjectHandle(clientID,'Right_camera',sim.simx_opmode_oneshot_wait)
    #ball handles
        #red
errorCode,red_zero = sim.simxGetObjectHandle(clientID,'Red_ball0',sim.simx_opmode_oneshot_wait)
errorCode,red_one = sim.simxGetObjectHandle(clientID,'Red_ball1',sim.simx_opmode_oneshot_wait)
errorCode,red_two = sim.simxGetObjectHandle(clientID,'Red_ball2',sim.simx_opmode_oneshot_wait)
errorCode,red_three = sim.simxGetObjectHandle(clientID,'Red_ball3',sim.simx_opmode_oneshot_wait)
errorCode,red_four = sim.simxGetObjectHandle(clientID,'Red_ball4',sim.simx_opmode_oneshot_wait)
red_balls = [red_zero,red_one,red_two,red_three,red_four]
        #blue
errorCode,Blue_zero = sim.simxGetObjectHandle(clientID,'Blue_ball0',sim.simx_opmode_oneshot_wait)
errorCode,Blue_one = sim.simxGetObjectHandle(clientID,'Blue_ball1',sim.simx_opmode_oneshot_wait)
errorCode,Blue_two = sim.simxGetObjectHandle(clientID,'Blue_ball2',sim.simx_opmode_oneshot_wait)
errorCode,Blue_three = sim.simxGetObjectHandle(clientID,'Blue_ball3',sim.simx_opmode_oneshot_wait)
errorCode,Blue_four = sim.simxGetObjectHandle(clientID,'Blue_ball4',sim.simx_opmode_oneshot_wait)
blue_balls = [Blue_zero,Blue_one,Blue_two,Blue_three,Blue_four]
    #tower handles
errorCode,tower_zero = sim.simxGetObjectHandle(clientID,'Tower_disk0',sim.simx_opmode_oneshot_wait)
errorCode,tower_one = sim.simxGetObjectHandle(clientID,'Tower_disk1',sim.simx_opmode_oneshot_wait)
errorCode,tower_two = sim.simxGetObjectHandle(clientID,'Tower_disk2',sim.simx_opmode_oneshot_wait)
errorCode,tower_three = sim.simxGetObjectHandle(clientID,'Tower_disk3',sim.simx_opmode_oneshot_wait)
errorCode,tower_four = sim.simxGetObjectHandle(clientID,'Tower_disk4',sim.simx_opmode_oneshot_wait)
errorCode,tower_five = sim.simxGetObjectHandle(clientID,'Tower_disk5',sim.simx_opmode_oneshot_wait)
errorCode,tower_six = sim.simxGetObjectHandle(clientID,'Tower_disk6',sim.simx_opmode_oneshot_wait)
errorCode,tower_seven = sim.simxGetObjectHandle(clientID,'Tower_disk7',sim.simx_opmode_oneshot_wait)
errorCode,tower_eight = sim.simxGetObjectHandle(clientID,'Tower_disk8',sim.simx_opmode_oneshot_wait)
towers = [tower_zero,tower_one,tower_two,tower_three,tower_four,tower_five,tower_six,tower_seven,tower_eight]


##control code
def set_drive(x,y,r):
    sim.simxSetJointTargetVelocity(clientID, front_left_motor_handle, x - y - r, sim.simx_opmode_streaming)
    sim.simxSetJointTargetVelocity(clientID, back_left_motor_handle, x + y - r, sim.simx_opmode_streaming)
    sim.simxSetJointTargetVelocity(clientID, back_right_motor_handle, x - y + r, sim.simx_opmode_streaming)
    sim.simxSetJointTargetVelocity(clientID, front_right_motor_handle, x + y + r, sim.simx_opmode_streaming)

foo = sim.simxReadVisionSensor

##vision
#color ranges for detection
class ColorBounds:
    def __init__(self, rgb):
        hsv = cv2.cvtColor(np.uint8([[[rgb[2], rgb[1], rgb[0]]]]), cv2.COLOR_BGR2HSV).flatten()

        lower = [hsv[0] - 15]
        upper = [hsv[0] + 15]

        if lower[0] < 0:
            lower.append(179 + lower[0]) # + negative = - abs
            upper.append(179)
            lower[0] = 0
        elif upper[0] > 179:
            lower.append(0)
            upper.append(upper[0] - 179)
            upper[0] = 179

        self.lower = [np.array([h, 100, 100]) for h in lower]
        self.upper = [np.array([h, 255, 255]) for h in upper]
#check for multi colored objects
def contains_vertical(r1, r2):
    x1, y1, w1, h1 = r1
    x2, y2, w2, h2 = r2

    return x1 <= x2 < x1 + w1 and x1 <= x2 + w2 < x1 + w1
#draw bounding box and distance
def drawLabel(w, h, x, y, text, frame, dist):
    cv2.rectangle(frame,(x,y),(x+w,y+h),(120,0,0),2)
    cv2.putText(frame, text, (x,y), cv2.FONT_HERSHEY_SIMPLEX, 1, (0,255,255), 2)
    cv2.putText(frame, dist, (x,y+h), cv2.FONT_HERSHEY_SIMPLEX, 1, (0,255,255), 2)


colorMap = {
        "Tower": ColorBounds((0, 244, 0)),
        "Blue Ball": ColorBounds((0, 0, 244)),
        "Red Ball": ColorBounds((244, 0, 0))
        }
cap = cv2.VideoCapture(0)
#distance
def calcDistance(focal_width, pixels, width):
    if pixels == 0 or width == 0:
        return ""
    distance = (focal_width * width) // (pixels)
    return distance

err, resolution, image = sim.simxGetVisionSensorImage(clientID, v1, 0, sim.simx_opmode_streaming)
err, resolution2, image2 = sim.simxGetVisionSensorImage(clientID, v2, 0, sim.simx_opmode_streaming)
err, resolution3, image3 = sim.simxGetVisionSensorImage(clientID, v3, 0, sim.simx_opmode_streaming)
##collision
balls_collected = 0
red_collected = []
blue_collected = []
def collide(loops):
    global score
    global balls_collected
    if TEAM == "blue":
        for x,ball in enumerate(red_balls):
            _, red_collide = sim.simxCheckCollision(clientID,robot_handle,ball,sim.simx_opmode_streaming) 
            if loops == 0:
                pass
            if loops > 0:
                _, red_collide = sim.simxCheckCollision(clientID,robot_handle,ball,sim.simx_opmode_buffer) 
                if red_collide == True:
                    if balls_collected < 2:
                        sim.simxSetObjectPosition(clientID,red_balls[x],-1,(100,100,0),sim.simx_opmode_oneshot)
                        balls_collected += 1
                        red_collected.append(ball)
        for x,tower in enumerate(towers):
            _, tower_collide = sim.simxCheckCollision(clientID,robot_handle,tower,sim.simx_opmode_streaming)
            _, tower_collide = sim.simxCheckCollision(clientID,robot_handle,tower,sim.simx_opmode_buffer)
            foo = red_collected.copy()

            for x in range(len(foo)):
                if tower_collide == True:
                    balls_collected -= 1
                    score += 1
                    red_collected.pop()
                    
    if TEAM == "red":
        for x, ball in enumerate(blue_balls):
            _, blue_collide = sim.simxCheckCollision(clientID,ram_handle,ball,sim.simx_opmode_streaming) 
            if loops == 0:
                pass
            if loops > 0:
                _, blue_collide = sim.simxCheckCollision(clientID,ram_handle,ball,sim.simx_opmode_buffer)
                if blue_collide == True:
                    if balls_collected < 2:
                        sim.simxSetObjectPosition(clientID,ball,-1,(100,100,0),sim.simx_opmode_oneshot)
                        balls_collected += 1
                        blue_collected.append(ball)
        for x,tower in enumerate(towers):
            _, tower_collide = sim.simxCheckCollision(clientID,robot_handle,tower,sim.simx_opmode_streaming)
            _, tower_collide = sim.simxCheckCollision(clientID,robot_handle,tower,sim.simx_opmode_buffer)
            foo = blue_collected.copy()

            for x in range(len(foo)):
                if tower_collide == True:
                    balls_collected -= 1
                    score += 1
                    blue_collected.pop()



#get front camera
def front_cam():
    err, resolution, image = sim.simxGetVisionSensorImage(clientID, v1, 0, sim.simx_opmode_buffer)
    img = np.array(image,dtype=np.uint8)
    img.resize([256,512,3])
    flip_img = cv2.flip(img,0)
    frame = flip_img

    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    rects = {}
    x_foo = 0
    w_foo = 0
    tx = 0
    txw = 0
    bad_ball = 0
    shortest_red = math.inf
    shortest_blue = math.inf
    shortest_tower = math.inf
    for name, color in colorMap.items():
        mask = cv2.inRange(hsv, color.lower[0], color.upper[0])

        if len(color.lower) == 2:
            mask = mask | cv2.inRange(hsv, color.lower[1], color.upper[1])

        conts, heirarchy = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)

        if (len(conts) == 0):
            continue

        biggest = sorted(conts, key=cv2.contourArea, reverse=True)[0]
        rect = cv2.boundingRect(biggest)
        x, y, w, h = rect
        if w < 20 or h < 20:
            continue

        rects[name] = rect
        widthF = abs(w)
        if name != "Tower":
            dist = calcDistance(BALL_FOCAL,widthF,BALL_SIZE)
            if dist < shortest_tower:
                shortest_tower = dist
                tx = x
                txw = x + w
        else:
            dist = calcDistance(TOWER_FOCAL,widthF,TOWER_SIZE)
        if name == "Blue Ball" and dist < shortest_blue:
            shortest_blue = dist
            if TEAM == "blue":
                x_foo = x
                w_foo = x + w
            if TEAM != "blue":
                bad_ball = x            
        elif name == "Red Ball" and dist < shortest_red:
            shortest_red = dist
            if TEAM == "red":
                x_foo = x
                w_foo = x + w
            if TEAM != "red":
                bad_ball = x
        drawLabel(w, h, x, y, name, frame,str(dist))
    cv2.imshow('front cam',frame)
    k = cv2.waitKey(5) & 0xFF
    if k == 27:
        return shortest_red, shortest_blue, shortest_tower, x_foo, w_foo, bad_ball, tx, txw
    return shortest_red, shortest_blue, shortest_tower, x_foo, w_foo, bad_ball, tx, txw
#get left camera
def left_cam():
    err, resolution2, image2 = sim.simxGetVisionSensorImage(clientID, v2, 0, sim.simx_opmode_buffer)
    img = np.array(image2,dtype=np.uint8)
    img.resize([256,512,3])
    flip_img = cv2.flip(img,0)
    frame = flip_img
    shortest_blue = math.inf
    shortest_red = math.inf
    shortest_tower = math.inf
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    bad_ball = 0
    rects = {}


    for name, color in colorMap.items():
        mask = cv2.inRange(hsv, color.lower[0], color.upper[0])

        if len(color.lower) == 2:
            mask = mask | cv2.inRange(hsv, color.lower[1], color.upper[1])

        conts, heirarchy = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)

        if (len(conts) == 0):
            continue

        biggest = sorted(conts, key=cv2.contourArea, reverse=True)[0]
        rect = cv2.boundingRect(biggest)
        x, y, w, h = rect
        if w < 20 or h < 20:
            continue

        rects[name] = rect
        widthF = abs(w)
        if name != "Tower":
            dist = calcDistance(BALL_FOCAL,widthF,BALL_SIZE)
            if dist < shortest_tower:
                shortest_tower = dist
        else:
            dist = calcDistance(TOWER_FOCAL,widthF,TOWER_SIZE)
        if name == "Blue Ball" and dist < shortest_blue:
            shortest_blue = dist
            if TEAM != "blue":
                bad_ball = x
        if name == "Red Ball" and dist < shortest_red:
            shortest_red = dist
            if TEAM != "red":
                bad_ball = x
        drawLabel(w, h, x, y, name, frame,str(dist))
    cv2.imshow('left cam',frame)
    k = cv2.waitKey(5) & 0xFF
    if k == 27:
        return shortest_red, shortest_blue, shortest_tower, bad_ball
    return shortest_red, shortest_blue, shortest_tower, bad_ball
#get right camera
def right_cam():
    err, resolution3, image3 = sim.simxGetVisionSensorImage(clientID, v3, 0, sim.simx_opmode_buffer)
    img = np.array(image3,dtype=np.uint8)
    img.resize([256,512,3])
    flip_img = cv2.flip(img,0)
    frame = flip_img
    bad_ball = 0
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    shortest_blue = math.inf
    shortest_red = math.inf
    shortest_tower = math.inf
    rects = {}


    for name, color in colorMap.items():
        mask = cv2.inRange(hsv, color.lower[0], color.upper[0])

        if len(color.lower) == 2:
            mask = mask | cv2.inRange(hsv, color.lower[1], color.upper[1])

        conts, heirarchy = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)

        if (len(conts) == 0):
            continue

        biggest = sorted(conts, key=cv2.contourArea, reverse=True)[0]
        rect = cv2.boundingRect(biggest)
        x, y, w, h = rect
        if w < 20 or h < 20:
            continue

        rects[name] = rect
        widthF = abs(w)
        if name != "Tower":
            dist = calcDistance(BALL_FOCAL,widthF,BALL_SIZE)
            if dist < shortest_tower:
                shortest_tower = dist
        else:
            dist = calcDistance(TOWER_FOCAL,widthF,TOWER_SIZE)
        if name == "Blue Ball" and dist < shortest_blue:
            shortest_blue = dist
            if TEAM != "blue":
                bad_ball = x
        elif name == "Red Ball" and dist < shortest_red:
            shortest_red = dist
            if TEAM != "red":
                bad_ball = x
        drawLabel(w, h, x, y, name, frame,str(dist))
    cv2.imshow('right cam',frame)
    k = cv2.waitKey(5) & 0xFF
    if k == 27:
        return shortest_red, shortest_blue, shortest_tower, bad_ball
    return shortest_red, shortest_blue, shortest_tower, bad_ball

def main():
    x_vel = 0
    y_vel = 0
    r_vel = 0
    iterations = 0
    global score
    global balls_collected
    start_time = time.process_time()
    while sim.simxGetConnectionId(clientID)!=-1 and time.process_time() - start_time < 45:   
        #set shortest distances to max
        shortest_red = math.inf 
        shortest_blue = math.inf 
        shortest_tower = math.inf
        #display cameras and get shortest from each camera
        shortest_redF, shortest_blueF, shortest_towerF, x, xw, bad, tx, txw = front_cam()
        shortest_redL, shortest_blueL, shortest_towerL, bad2 = left_cam()
        shortest_redR, shortest_blueR, shortest_towerR, bad3 = right_cam()
        #find which camera shortest distances is on
        if shortest_redF < shortest_red:
            shortest_red = shortest_redF
        if shortest_redL < shortest_red:
            shortest_red = shortest_redL
        if shortest_redR < shortest_red:
            shortest_red = shortest_redR

        if shortest_blueF < shortest_blue:
            shortest_blue = shortest_blueF
        if shortest_blueL < shortest_blue:
            shortest_blue = shortest_blueL
        if shortest_blue < shortest_blue:
            shortest_blue = shortest_blueR

        if shortest_towerF < shortest_tower:
            shortest_tower = shortest_towerF
        if shortest_towerL < shortest_tower:
            shortest_tower = shortest_towerL
        if shortest_towerR < shortest_tower:
            shortest_tower = shortest_towerR

        vel = 4
        x_vel = 0
        y_vel = 0
        r_vel = 0
        #movement control with basic decision making
        if balls_collected < 2:
            if TEAM == "red":
                #if closest distance is front
                if shortest_red == shortest_redF and shortest_red != math.inf:
                    if x > 125 and xw < 475 or xw - x > 350:
                        x_vel = 10
                    elif x < 125 and x > 75:
                        x_vel = 10
                        r_vel = -0.25
                    elif x <= 75:
                        x_vel = 7
                        r_vel = -1
                    elif xw > 460 and xw < 500:
                        x_vel = 8
                        r_vel = 0.25
                    elif xw >= 500:
                        set_drive(vel,0,1)
                        x_vel = 8
                        r_vel = 1
                    if bad < 256 and shortest_blueF < 20:
                        y_vel = -5
                    elif bad > 256 and shortest_blueF < 20:
                        y_vel = 5
                #if closest distance is left
                elif shortest_red == shortest_redL and shortest_red != math.inf:
                    x_vel = 8 / 1.5
                    r_vel = -3
                    if bad2 > 256 and shortest_blueL < 20:
                        y_vel = 5

                #if closest distance is right 
                elif shortest_red == shortest_redR and shortest_red != math.inf:
                    x_vel = 8 / 1.5
                    r_vel = 3
                    if bad3 > 256 and shortest_blueR < 20:
                        y_vel = -5
                #if nothing found
                else:
                    x_vel = -2

            elif TEAM == "blue":
                #if closest is front
                if shortest_blue == shortest_blueF and shortest_blue != math.inf:
                    if x > 125 and xw < 475 or xw - x > 350:
                        x_vel = 10
                    elif x < 125 and x > 75:
                        x_vel = 10
                        r_vel = -0.25
                    elif x <= 75:
                        x_vel = 8
                        r_vel = -1
                    elif xw > 460 and xw < 500:
                        x_vel = 8
                        r_vel = 0.25
                    elif xw >= 500:
                        set_drive(vel,0,1)
                        x_vel = 8
                        r_vel = 1
                    if bad < 256 and shortest_redF < 20:
                        y_vel = -5
                    elif bad > 256 and shortest_redF < 20:
                        y_vel = 5
                #if closest is left
                elif shortest_blue == shortest_blueL and shortest_blue != math.inf:
                    x_vel = 8 / 1.5
                    r_vel = -3
                    if bad2 > 256 and shortest_redL < 20:
                        y_vel = 5
                #if closest is right
                elif shortest_blue == shortest_blueR and shortest_blue != math.inf:
                    x_vel = 8 / 1.5
                    r_vel = 3            
                    if bad3 > 256 and shortest_redR < 20:
                        y_vel = -5

                #if nothing found
                else:
                    x_vel = -2
        #full on balls, travel closest tower
        elif balls_collected == 2:
            #if closest is front
            if shortest_tower == shortest_towerF and shortest_tower != math.inf:
                if tx > 125 and txw < 475 or txw - tx > 350:
                    x_vel = 7
                elif tx < 125 and tx > 75:
                    x_vel = 7
                    r_vel = -0.25
                elif tx <= 75:
                    x_vel = 7
                    r_vel = -1
                elif txw > 460 and txw < 500:
                    x_vel = 6
                    r_vel = 0.25
                elif txw >= 500:
                    set_drive(vel,0,1)
                    x_vel = 6
                    r_vel = 1
            #if closest is left
            elif shortest_tower == shortest_towerL and shortest_tower != math.inf:
                x_vel = 6 / 1.5
                r_vel = -3
            #if closest is right
            elif shortest_tower == shortest_towerR and shortest_tower != math.inf:
                x_vel = 6 / 1.5
                r_vel = 3
            #if nothing found
            else:
                x_vel = -2
        collide(iterations)
        print(time.process_time() - start_time)
        iterations += 1
        set_drive(x_vel,y_vel,r_vel)
    set_drive(0,0,0)
    print(score)


main()