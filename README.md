# PiStar-Tracker-keyboard-Edition
Autoguider using the PiCam, controlled via keyboard for a multi-star/object autoguider

This star tracker is using opencv algorithms and some kind of star-distinguish-algorithms.
it's just working with the raspberry Pi camera

There was no exposure/gain settings done with the camera.

A little issue with the coordinate-transformation: Depending on the camera-angle, it could be necessary to flip the axis of the controller-part of the program:

if(dRA > 0 ) to if(dRA < 0 ) for all 4 GPIO channels
