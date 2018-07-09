#   s   set tracked star
#   c   start calibration
#   t   start/end tracking
#   l   print calibration log
#   r   reset startable

import cv2
import numpy as np
import os
import RPi.GPIO as GPIO
import time

################ Global ##############
TrackedStar = 0

CalibrationSteps = 100
StartCalibration = 0
RACalibIndicator = 0
RALog = np.empty((0,2),float)

DECCalibIndicator = 0
DECLog = np.empty((0,2),float)

slopeRA = 0
slopeDEC = 0
slopeIndicator = 0

StarTableTrans = np.empty((0,4),float)

dx = 0
dy = 0

TrackingIndicator = 0

ShowLog = 1

#TrackingLog: RA+,time,RA-,time,DEC+,time,DEC-,time
TrackingLog = np.empty((0,8),float)

RAangle = 0
DECangle = 0

######################################


def closeWindow():
    cv2.destroyAllWindows()
    cv2.waitKey(0)
    cv2.waitKey(0)
    cv2.waitKey(0)
    cv2.waitKey(0)
    cv2.waitKey(0)
    GPIO.output(26,0) #RA-
    GPIO.output(19,0) #DEC-
    GPIO.output(20,0) #RA+
    GPIO.output(21,0) #DEC+
    GPIO.cleanup()



def Initialize():
    #set picamera driver to bcm2835
    #set ports to 0
    os.system("sudo modprobe bcm2835-v4l2")
    GPIO.setmode(GPIO.BCM)
    GPIO.setup(26,GPIO.OUT)
    GPIO.setup(19,GPIO.OUT)
    GPIO.setup(21,GPIO.OUT)
    GPIO.setup(20,GPIO.OUT)

    GPIO.output(26,0)
    GPIO.output(19,0)
    GPIO.output(20,0)
    GPIO.output(21,0)

def getCoordinates():
    ret,frame = cap.read()
    TempCoordinates = np.empty((0,2),float)
    Threshold = frame.max()*0.7
    gray = cv2.cvtColor(frame,cv2.COLOR_BGR2GRAY)
    ret, thresh = cv2.threshold(gray,Threshold,255,0)
    image, cnts, hierarchy = cv2.findContours(thresh, cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
    for c in cnts:
        M = cv2.moments(c)
        if(M['m10']!=0):
            cX = int(M['m10']/M['m00'])
            cY = int(M['m01']/M['m00'])
            TempCoordinates = np.append(TempCoordinates,np.array([[cX,cY]]),axis=0)
    return TempCoordinates


def getInitialStarTable():
    Coordinates = getCoordinates()
    TempStarTable = np.empty((0,4),float)
    StarNumber = 0
    print(Coordinates)
    for x,y in Coordinates:
        TempStarTable = np.append(TempStarTable,np.array([[StarNumber,x,y,1]]),axis=0)
        StarNumber+=1
    return TempStarTable
    
def StarMatching(StarTable):
    CorrMatrix = np.empty((0,2),int)
    
    CurrentCoordinates = getCoordinates()
    i,x0,y0,onlineTable = StarTable.T
    xN,yN = CurrentCoordinates.T
    onesArrA = np.ones((StarTable.shape[0],1))
    onesArrB = np.ones((CurrentCoordinates.shape[0],1))
    x0 = np.outer(x0,onesArrB)
    xN = np.outer(onesArrA,xN)
    y0 = np.outer(y0,onesArrB)
    yN = np.outer(onesArrA,yN)
    dx = np.power(x0-xN,2)
    dy = np.power(y0-yN,2)
    R = np.sqrt(dx+dy)
    sigma = 20
    G=np.exp(-(R*R)/(2*sigma*sigma))
    G=np.round(G,0)

    i=0
    j=0
    
    while(i<G.shape[0]):
        while(j<G.shape[1]):
            if(G[i,j]==1):
               CorrMatrix = np.append(CorrMatrix,np.array([[i,j]]),axis=0)
            j+=1
        j=0
        i+=1
    for C in CorrMatrix:
        c1,c2 = C
        starnumber,x,y,online = StarTable[c1]
        xnew,ynew = CurrentCoordinates[c2]
        StarTable[c1]=starnumber,xnew,ynew,1
    return StarTable

def getSlope():
    global slopeRA
    global slopeDEC
    global RAangle
    global DECangle
    
    xRA, yRA = RALog.T
    xDEC, yDEC = DECLog.T

    slopeRA, zRA = np.polyfit(xRA,yRA,1)
    slopeDEC, zDEC = np.polyfit(xDEC,yDEC,1)
    RAangle = np.degrees(np.arctan(slopeRA))
    DECangle = np.degrees(np.arctan(slopeDEC))

############################ Calibration ############################

def PrintLog():
    for x,y in RALog:
        cv2.circle(frame,(int(x),int(y)),2,(255,255,0),-1)
    for x,y in DECLog:
        cv2.circle(frame,(int(x),int(y)),2,(255,255,0),-1)
        

def RACalibration(StarTable,TrackedStar):
    
    global RACalibIndicator
    global RALog
    if(len(RALog) <= (CalibrationSteps-1)):
        GPIO.output(20,1)
        RALog = np.append(RALog,np.array([[StarTable[TrackedStar,1],StarTable[TrackedStar,2]]]),axis=0)
    if(len(RALog) == CalibrationSteps):
        RACalibIndicator = 1
        GPIO.output(20,0)
    cv2.putText(frame,'RA Calibration on: '+str(len(RALog))+' Steps',(10,20),1,1,(255,255,255),1,0)

def DECCalibration(StarTable,TrackedStar):
    global DECCalibIndicator
    global DECLog
    if(len(DECLog) <= (CalibrationSteps-1) and RACalibIndicator == 1):
        GPIO.output(19,1)
        DECLog = np.append(DECLog,np.array([[StarTable[TrackedStar,1],StarTable[TrackedStar,2]]]),axis=0)
    if(len(DECLog) == CalibrationSteps):
        GPIO.output(19,0)
        DECCalibIndicator = 1
    cv2.putText(frame,'DEC Calibration on: '+str(len(DECLog))+' Steps',(10,32),1,1,(255,255,255),1,0)

def CoordinatesTransformation(StarTable,TrackedStar):
    global StarTableTrans
    global xreference
    global yreference

    StarTableTrans = np.empty((0,4),float)
    for Star in StarTable:
        number, x, y, online = Star
        x = x-xreference
        y = y-yreference
        motionAnglex = abs(DECangle)
        motionAngley = abs(RAangle)
        xtrans = x*np.cos(motionAnglex * np.pi/180)-y*np.sin(motionAnglex * np.pi/180)
        ytrans = x*np.sin(motionAnglex * np.pi/180)+y*np.cos(motionAnglex * np.pi/180)
        #xtrans = x*np.cos(slopeDEC)+y*np.sin(slopeDEC)
        #ytrans = y*np.cos(slopeDEC)-x*np.sin(slopeDEC)
        StarTableTrans = np.append(StarTableTrans,np.array([[number,xtrans,ytrans,1]]),axis=0)


def TrackingMarkers(StarTable,TrackedStar):
    x = StarTable[TrackedStar,1]
    y = StarTable[TrackedStar,2]
    size = 20
    x11 = x+size
    x12 = x+size
    x13 = x+size/2
    y11 = y-size
    y12 = y-size/2
    y13 = y-size

    cv2.line(frame,(int(x11),int(y11)),(int(x12),int(y12)),(255,0,0),2)
    cv2.line(frame,(int(x11),int(y11)),(int(x13),int(y13)),(255,0,0),2)

    x21 = x+size
    x22 = x+size
    x23 = x+size/2
    y21 = y+size
    y22 = y+size/2
    y23 = y+size

    cv2.line(frame,(int(x21),int(y21)),(int(x22),int(y22)),(255,0,0),2)
    cv2.line(frame,(int(x21),int(y21)),(int(x23),int(y23)),(255,0,0),2)

    x31 = x-size
    x32 = x-size
    x33 = x-size/2
    y31 = y+size
    y32 = y+size/2
    y33 = y+size

    cv2.line(frame,(int(x31),int(y31)),(int(x32),int(y32)),(255,0,0),2)
    cv2.line(frame,(int(x31),int(y31)),(int(x33),int(y33)),(255,0,0),2)

    x41 = x-size
    x42 = x-size
    x43 = x-size/2
    y41 = y-size
    y42 = y-size/2
    y43 = y-size

    cv2.line(frame,(int(x41),int(y41)),(int(x42),int(y42)),(255,0,0),2)
    cv2.line(frame,(int(x41),int(y41)),(int(x43),int(y43)),(255,0,0),2)

def Tracking(StarTableTrans):
    dDEC = StarTableTrans[TrackedStar,1]
    dRA = StarTableTrans[TrackedStar,2]
    cv2.putText(frame,'dRA: '+'{:.2f}'.format(dRA)+' dDEC: '+'{:.2f}'.format(dDEC),(300,92),1,1,(255,255,255),1,0)
    
    if(dRA > 0 ):
        GPIO.output(26,1) 
        time.sleep(0.1)
        cv2.putText(frame,'RA-',(10,120),1,1,(255,255,255),1,0)
    if(dRA < 0 ):
        GPIO.output(20,1)
        time.sleep(0.1)
        cv2.putText(frame,'RA+',(10,120),1,1,(255,255,255),1,0)
    if(dDEC < 0):
        GPIO.output(21,1)
        time.sleep(0.1)
        cv2.putText(frame,'DEC-',(10,132),1,1,(255,255,255),1,0)
    if(dDEC > 0 ):
        GPIO.output(19,1)
        time.sleep(0.1)
        cv2.putText(frame,'DEC+',(10,132),1,1,(255,255,255),1,0)

    GPIO.output(26,0)
    GPIO.output(19,0)
    GPIO.output(20,0)
    GPIO.output(21,0)  

##############################################    
Initialize()
cap = cv2.VideoCapture(0)
StarTable = np.empty((0,4),float)


i=0 #5 times for accuracy
while(i<5):
    StarTable = getInitialStarTable()
    i+=1
###############################################

xreference = StarTable[TrackedStar,1]
yreference = StarTable[TrackedStar,2]

while True:
    ret,frame = cap.read()
    Coordinates = getCoordinates()
    StarTable = StarMatching(StarTable)

    if cv2.waitKey(33) & 0xFF == ord('c'):
        StartCalibration = 1
    if(StartCalibration == 1):
        RACalibration(StarTable, TrackedStar)
        DECCalibration(StarTable,TrackedStar)
        
    if cv2.waitKey(33) & 0xFF == ord('s'):
        if(TrackedStar < len(StarTable)):
            TrackedStar+=1
        if(TrackedStar == len(StarTable)):
           TrackedStar=0
        #if(slopeIndicator == 1):
        #    xreference = StarTable[TrackedStar,1]
        #    yreference = StarTable[TrackedStar,2]
        xreference = StarTable[TrackedStar,1]
        yreference = StarTable[TrackedStar,2]

    if cv2.waitKey(33) & 0xFF == ord('r'):
        StarTable = getInitialStarTable()
        print('Startable reset done')

    cv2.putText(frame,'xref: '+str(xreference),(300,10),1,1,(255,255,255),1,0)
    cv2.putText(frame,'yref: '+str(yreference),(300,22),1,1,(255,255,255),1,0)
        
    cv2.circle(frame,(int(StarTable[TrackedStar,1]),int(StarTable[TrackedStar,2])),5,(0,255,0),-1)
    TrackingMarkers(StarTable,TrackedStar)
    

    if cv2.waitKey(33) & 0xFF == ord('l'):
        ShowLog = (ShowLog+1)%2

    if(ShowLog == 1):
        PrintLog()


    if(RACalibIndicator == 1 and DECCalibIndicator == 1 and slopeIndicator == 0):
        getSlope()
        slopeIndicator = 1
        print(slopeRA)
        print(slopeDEC)
    if(slopeIndicator == 1):
        CoordinatesTransformation(StarTable,TrackedStar)

        cv2.putText(frame,'Star selected: '+str(TrackedStar),(10,68),1,1,(255,255,255),1,0)
        cv2.putText(frame,'x: '+str(StarTable[TrackedStar,1])+' y: '+str(StarTable[TrackedStar,2]),(10,80),1,1,(255,255,255),1,0)
        dy = StarTableTrans[TrackedStar,1]
        dx = StarTableTrans[TrackedStar,2]
        cv2.putText(frame,'dRA: '+'{:.2f}'.format(dx)+' dDEC: '+'{:.2f}'.format(dy),(10,92),1,1,(255,255,255),1,0)
        cv2.putText(frame,'RA angle: ' + '{:.2f}'.format(RAangle),(10,300),1,1,(255,255,255),1,0)
        cv2.putText(frame,'DEC angle: ' + '{:.2f}'.format(DECangle),(10,312),1,1,(255,255,255),1,0)
        orthoError = 90-(abs(RAangle)+abs(DECangle))
        cv2.putText(frame,'Orthogonality Error: '+'{:.2f}'.format(orthoError),(10,324),1,1,(255,255,255),1,0)
        cv2.putText(frame,'x: '+str(StarTable[TrackedStar,1]),(10,336),1,1,(255,255,255),1,0)
        cv2.putText(frame,'y: '+str(StarTable[TrackedStar,2]),(10,348),1,1,(255,255,255),1,0)
        
        length = 100
        x2 = xreference + length * np.cos(RAangle * np.pi/180)
        y2 = yreference + length * np.sin(RAangle * np.pi/180)
        x3 = xreference + length * np.cos((DECangle) * np.pi/180)
        y3 = yreference + length * np.sin((DECangle) * np.pi/180)
        
        cv2.line(frame,(int(xreference),int(yreference)),(int(x2),int(y2)),(255,0,0),2)
        cv2.line(frame,(int(xreference),int(yreference)),(int(x3),int(y3)),(255,255,255),2)

        cv2.putText(frame,'RA+: ',(int(x2),int(y2)),1,1,(255,255,255),1,0)
        cv2.putText(frame,'DEC+: ',(int(x3),int(y3)),1,1,(255,255,255),1,0)
        

        
    if cv2.waitKey(33) & 0xFF == ord('t'):
        TrackingIndicator = (TrackingIndicator+1)%2
    if(TrackingIndicator == 1):
        Tracking(StarTableTrans)
        cv2.putText(frame,'tracking on',(300,104),1,1,(255,255,255),1,0)
    if(TrackingIndicator == 0):
        cv2.putText(frame,'tracking off',(300,104),1,1,(255,255,255),1,0)
        GPIO.output(26,0)
        GPIO.output(19,0)
        GPIO.output(20,0)
        GPIO.output(21,0)
     
    
    for Star in StarTable:
        number,x,y,online = Star
        cv2.putText(frame,str(number),(int(x),int(y)),1,1,(255,255,255),1,0)

    if cv2.waitKey(33) & 0xFF == ord('q'):
        break

    cv2.imshow('out',frame)

closeWindow()
