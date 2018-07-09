# PiStar-Tracker-keyboard-Edition
Autoguider using the PiCam, controlled via keyboard

This star tracker is using opencv algorithms and some kind of star-distinguish-algorithms.
it's just working with the raspberry Pi camera

There was no exposure/gain settings done with the camera.

A little issue with the coordinate-transformation: Depending on the camera-angle, it could be necessary to flip the axis of the controller-part of the program:

if(dRA < 0 ):
        GPIO.output(26,1) 
        time.sleep(0.1)
        cv2.putText(frame,'RA-',(10,120),1,1,(255,255,255),1,0)
    if(dRA > 0 ):
        GPIO.output(20,1)
        time.sleep(0.1)
        cv2.putText(frame,'RA+',(10,120),1,1,(255,255,255),1,0)
    if(dDEC > 0):
        GPIO.output(21,1)
        time.sleep(0.1)
        cv2.putText(frame,'DEC-',(10,132),1,1,(255,255,255),1,0)
    if(dDEC < 0 ):
        GPIO.output(19,1)
        time.sleep(0.1)
        cv2.putText(frame,'DEC+',(10,132),1,1,(255,255,255),1,0)
        
        to
        
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
