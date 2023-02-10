# import the necessary packages
from picamera.array import PiRGBArray
from picamera import PiCamera
from imutils.video import FPS
from gpiozero import LED
import cv2
import time
import os

#led.on() means led is off
#connect the longer side of the led pin to the resistor
#connect the resistor with 3V(pin no.1) of raspp
#connect the shorter side to the gpio 17 i.e. pin 11
led = LED(17)
led.on()

# initialize the camera and grab a reference to the raw camera capture
camera = PiCamera()
camera.resolution = (640, 480)
camera.framerate = 32
rawCapture = PiRGBArray(camera, size=(640, 480))

# allow the camera to warmup
print("[INFO] sampling frames from `picamera` module...")
time.sleep(0.1)
first_read = False
fps = FPS().start()
        
#Initializing the face and eye cascade classifiers from xml files
face_cascade = cv2.CascadeClassifier('haarcascade_frontalface_default.xml')
eye_cascade = cv2.CascadeClassifier('haarcascade_eye_tree_eyeglasses.xml')

windowName = "Live Video Feed"
cv2.namedWindow(windowName)

# capture frames from the camera
for frame in camera.capture_continuous(rawCapture, format="bgr", use_video_port=True):
    # grab the raw NumPy array representing the image, then initialize the timestamp
    # and occupied/unoccupied text
    img = frame.array

    #Coverting the recorded image to grayscale
    gray = cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)
    
    #Applying filter to remove impurities
    gray = cv2.bilateralFilter(gray,5,1,1)

    #Detecting the face for region of image to be fed to eye classifier
    faces = face_cascade.detectMultiScale(gray, 1.1, 5,minSize=(100,100))

    if(len(faces)>0):

        for (x,y,w,h) in faces:
            cv2.rectangle(img,(x,y),(x+w,y+h),(0,255,0),2)

            #roi_face is face which is input to eye classifier
            roi_face = gray[y:y+h,x:x+w]
            roi_face_clr = img[y:y+h,x:x+w]
            eyes = eye_cascade.detectMultiScale(roi_face,1.1,2,minSize=(5,5))

            #if eyes are open keep setting time variable t0
            if(len(eyes) >= 1):
                for (ex,ey,ew,eh) in eyes:
                    cv2.rectangle(roi_face_clr,(ex,ey),(ex+ew,ey+eh),(255,0,0),2)
     
                cv2.putText(img, "Eyes open!", (70,70), cv2.FONT_HERSHEY_PLAIN, 2,(255,255,255),2)
                led.on()
                t0 = time.time()
                first_read = True
        
            # if eyes are closed start setting t1    
            if (len(eyes) < 1) and first_read:

                cv2.putText(img, "Eyes closed!", (70,70), cv2.FONT_HERSHEY_PLAIN, 2,(255,255,255),2)
                led.off()
                t1 = time.time()
                #compare t1 with last set t0 and 3 sec is enough to tell whether driver is sleepy
                if ((t1 - t0) > 3):

                    cv2.putText(img, "Driver Sleepy!", (70,30), cv2.FONT_HERSHEY_PLAIN, 2,(255,255,255),2)
                    
                    # t1 -t0 is float value with more than 10 digits precision
                    if (int(t1 - t0) % 3 == 0 ):
                        print("Driver sleepy!",int(t1-t0),"seconds")
                        sec = int(t1 - t0)
                        string = '/usr/bin/pushbullet.sh '+ str(sec)
                        os.system(string)
                        time.sleep(0.8)     #sleep for 0.8 sec so that within a second notification is done only once
                    

    else:
        cv2.putText(img,"No face detected",(100,100),cv2.FONT_HERSHEY_PLAIN, 3, (0,255,0),2)
        led.on()    
    #Controlling the algorithm with keys
    cv2.imshow(windowName,img)

    # clear the stream in preparation for the next frame
    rawCapture.truncate(0)
    fps.update()

    if cv2.waitKey(1)==27 :
        break

# stop the timer and display FPS information
fps.stop()
print("[INFO] elasped time: {:.2f}".format(fps.elapsed()))
print("[INFO] approx. FPS: {:.2f}".format(fps.fps()))


#using picamera() do not use release() rather close()
camera.close()
cv2.destroyWindow(windowName)
led.on()

    
