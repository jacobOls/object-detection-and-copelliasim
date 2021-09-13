import sys
import numpy as np
import array
import time
import cv2
##real life color detection and distance calculation for VEX change up challenge field objects.
BALL_SIZE = 6.3
BALL_FOCAL = (700 * 8) / BALL_SIZE
TOWER_SIZE = 4
TOWER_FOCAL = (100 * 30) / TOWER_SIZE
##vision
class ColorBounds:
    def __init__(self, rgb):
        hsv = cv2.cvtColor(np.uint8([[[rgb[2], rgb[1], rgb[0]]]]), cv2.COLOR_BGR2HSV).flatten()

        lower = [hsv[0] - 15]
        upper = [hsv[0] + 15]

        if lower[0] < 0:
            lower.append(179 + lower[0])
            upper.append(179)
            lower[0] = 0
        elif upper[0] > 179:
            lower.append(0)
            upper.append(upper[0] - 179)
            upper[0] = 179

        self.lower = [np.array([h, 100, 100]) for h in lower]
        self.upper = [np.array([h, 255, 255]) for h in upper]

def contains_vertical(r1, r2):
    x1, y1, w1, h1 = r1
    x2, y2, w2, h2 = r2

    return x1 <= x2 < x1 + w1 and x1 <= x2 + w2 < x1 + w1

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
    return str(distance)
##main
def main():
    while True:
        _, frame = cap.read()
        frame = cv2.resize(frame, (1000, 560))
        frame = cv2.flip(frame, 1)

        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

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

            # if name == "Red Ball" or name == "Blue Ball":
                # if any([contains_vertical(rects[n], rect) for n in rects]):
                #    continue
            
            rects[name] = rect
            widthF = abs(w)
            if name != "Tower":
                dist = calcDistance(BALL_FOCAL,widthF,BALL_SIZE)
            else:
                dist = calcDistance(TOWER_FOCAL,widthF,TOWER_SIZE)
            drawLabel(w, h, x, y, name, frame,dist)

        cv2.imshow('image',frame)
        k = cv2.waitKey(5) & 0xFF
        if k == 27:
            break
    cv2.destroyAllWindows()
    cv2.waitKey(1)

main()