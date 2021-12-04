import cv2
import mediapipe as mp
import json
import re
import time
import boto3

client = boto3.client('iot-data',aws_access_key_id='ACCESS KEY',aws_secret_access_key='SECRET', region_name='us-east-2')
destination = 'pull/BionicArm'

cap = cv2.VideoCapture(0)
mpHands=mp.solutions.hands
hands = mpHands.Hands( static_image_mode=False,
               max_num_hands=2,
               model_complexity=1,
               min_detection_confidence=0.5,
               min_tracking_confidence=0.5)

# Draw hand 
mpDraw = mp.solutions.drawing_utils

#fps Cal
pTime = 0
cTime = 0

xZero = 0
yZero = 0
zZero = 0
Payloadpushtime = 0.5
oldpushtime = 0
PowerPercentThumb = None
PowerPercentIndex=None
PowerPercentMiddle=None
PowerPercentRing=None
PowerPercentPinkey =None
distance=0
while True:
    success,img = cap.read()
    imgRGB = cv2.cvtColor(img,cv2.COLOR_BGR2RGB)
    results = hands.process(imgRGB)
    #Check for multiple hands 
    # print(results.multi_hand_landmarks) # NONE when no hands detected Else XYZ given

    if results.multi_hand_landmarks:
        for EachHandLandmark in results.multi_hand_landmarks:
            for id,lndmrk in enumerate(EachHandLandmark.landmark):
                if id%4 == 0:
                    # print (id,lndmrk)
                    height,width,channel = img.shape
                    xcoordinate,ycoordinate,zcoordinate = int(lndmrk.x*width),int(lndmrk.y*height),lndmrk.z
                    # print (id,xcoordinate, ycoordinate)
                    if id == 0:
                        xZero,yZero,zZero = xcoordinate,ycoordinate,zcoordinate
                    distanceTemp = (int(100-pow(pow(xcoordinate-xZero,2)+pow(ycoordinate-yZero,2)+pow(zcoordinate-zZero,2),1/2)*100/300))
                    if distanceTemp < 0:
                        distance = 0
                    elif distance > 100:
                        distance = 100
                    else:
                        distance=distanceTemp

                    cv2.putText(img,str(distanceTemp),(xcoordinate,ycoordinate),cv2.FONT_HERSHEY_PLAIN,1,(0,0,255),1)
                    
                    if (id == 4):
                        PowerPercentThumb = distance
                    elif (id == 8):
                        PowerPercentIndex = distance
                    elif(id==12):
                        PowerPercentMiddle = distance
                    elif(id==16):
                        PowerPercentRing = distance
                    elif(id==20):
                        PowerPercentPinkey = distance
  
                    if (time.time()-oldpushtime > Payloadpushtime and id is not False):
                        Command = {"Data":[{"finger":0,"PowerPercent":PowerPercentThumb},{"finger":1,"PowerPercent":PowerPercentIndex},{"finger":2,"PowerPercent":PowerPercentMiddle},{"finger":3,"PowerPercent":PowerPercentRing},{"finger":4,"PowerPercent":PowerPercentPinkey}]}
                        client.publish(topic=destination,qos=0,payload=json.dumps(Command))
                        oldpushtime = time.time()
            mpDraw.draw_landmarks(img,EachHandLandmark,mpHands.HAND_CONNECTIONS)

    cTime = time.time()
    fps = 1/(0.00001+(cTime-pTime))
    pTime = cTime
    
    cv2.putText(img,"FPS="+str(int(fps)),(10,70),cv2.FONT_HERSHEY_PLAIN,1,(255,0,0),1)
    cv2.imshow("Image",img)
    cv2.waitKey(1)