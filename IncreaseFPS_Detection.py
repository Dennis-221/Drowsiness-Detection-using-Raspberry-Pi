# import the necessary packages
from __future__ import print_function
from imutils.video.pivideostream import PiVideoStream
from imutils.video import FPS
from picamera.array import PiRGBArray
from picamera import PiCamera
from threading import Thread
import argparse
import imutils
import time
import cv2
import os
from gpiozero import LED
    
# construct the argument parse and parse the arguments
ap = argparse.ArgumentParser()

ap.add_argument("-n", "--num-frames", type=int, default=100, help="# of frames to loop over for FPS test")

ap.add_argument("-d", "--display", type=int, default=-1, help="Whether or not frames should be displayed")

args = vars(ap.parse_args())

#Initializing the face and eye cascade classifiers from xml files
face_cascade = cv2.CascadeClassifier('haarcascade_frontalface_default.xml')
eye_cascade = cv2.CascadeClassifier('haarcascade_eye_tree_eyeglasses.xml')

#Initialisation
led = LED(17)
led.on()
first_read = False

# initialize the camera and stream
camera = PiCamera()
camera.resolution = (640, 480)
camera.framerate = 32
rawCapture = PiRGBArray(camera, size=(640, 480))
stream = camera.capture_continuous(rawCapture, format="bgr", use_video_port=True)


# allow the camera to warmup and start the FPS counter
print("[INFO] sampling frames from `picamera` module...")
time.sleep(2.0)
fps = FPS().start()

# loop over some frames
for (i, f) in enumerate(stream):
    # grab the frame from the stream and resize it to have a maximum
    # width of 400 pixels
    frame = f.array
    frame = imutils.resize(frame, width=400)
    ###################
    img = frame

    #Coverting the recorded image to grayscale
    gray = cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)
        
    #Applying filter to remove impurities
    gray = cv2.bilateralFilter(gray,5,1,1)

    #Detecting the face for region of image to be fed to eye classifier
    faces = face_cascade.detectMultiScale(gray, 1.1, 5,minSize=(100,100))

    if(len(faces)>0):

        for (x,y,w,h) in faces:
            #cv2.rectangle(img,(x,y),(x+w,y+h),(0,255,0),2)

            #roi_face is face which is input to eye classifier
            roi_face = gray[y:y+h,x:x+w]
            roi_face_clr = img[y:y+h,x:x+w]
            eyes = eye_cascade.detectMultiScale(roi_face,1.1,2,minSize=(5,5))

            #if eyes are open keep setting time variable t0
            if(len(eyes) >= 1):
                #for (ex,ey,ew,eh) in eyes:
                    #cv2.rectangle(roi_face_clr,(ex,ey),(ex+ew,ey+eh),(255,0,0),2)
         
                #cv2.putText(img, "Eyes open!", (70,70), cv2.FONT_HERSHEY_PLAIN, 2,(255,255,255),2)
                led.on()
                t0 = time.time()
                first_read = True
            
            # if eyes are closed start setting t1    
            if (len(eyes) < 1) and first_read:

                #cv2.putText(img, "Eyes closed!", (70,70), cv2.FONT_HERSHEY_PLAIN, 2,(255,255,255),2)
                led.off()
                t1 = time.time()
                #compare t1 with last set t0 and 3 sec is enough to tell whether driver is sleepy
                if ((t1 - t0) > 3):

                    #cv2.putText(img, "Driver Sleepy!", (70,30), cv2.FONT_HERSHEY_PLAIN, 2,(255,255,255),2)
                        
                    # t1 -t0 is float value with more than 10 digits precision
                    if (int(t1 - t0) % 3 == 0 ):
                        print("Driver sleepy!",int(t1-t0),"seconds")
                        sec = int(t1 - t0)
                        string = '/usr/bin/pushbullet.sh '+ str(sec)
                        os.system(string)
                        time.sleep(0.8)     #sleep for 0.8 sec so that within a second notification is done only once
                        

    else:
        #cv2.putText(img,"No face detected",(100,100),cv2.FONT_HERSHEY_PLAIN, 3, (0,255,0),2)
        led.on()    
    ##################
    #cv2.imshow("Frame", frame)
    if cv2.waitKey(1) == 27:
        break


    # check to see if the frame should be displayed to our screen
    #if args["display"] > 0:
    #cv2.imshow("Frame", frame)
    #if cv2.waitKey(1) == 27:
     #   break
        # clear the stream in preparation for the next frame and update
    # the FPS counter
    rawCapture.truncate(0)
    fps.update()
    # check to see if the desired number of frames have been reached
    if i == args["num_frames"]:
        break
# stop the timer and display FPS information
fps.stop()
print("[INFO] elasped time: {:.2f}".format(fps.elapsed()))
print("[INFO] approx. FPS: {:.2f}".format(fps.fps()))

# do a bit of cleanup
cv2.destroyAllWindows()
stream.close()
rawCapture.close()
camera.close()




# created a *threaded *video stream, allow the camera sensor to warmup,
# and start the FPS counter
print("[INFO] sampling THREADED frames from `picamera` module...")

vs = PiVideoStream().start()
time.sleep(2.0)
fps = FPS().start()

# loop over some frames...this time using the threaded stream
while fps._numFrames < args["num_frames"]:
        # grab the frame from the threaded video stream and resize it
        # to have a maximum width of 400 pixels
        frame = vs.read()
        frame = imutils.resize(frame, width=400)
        # check to see if the frame should be displayed to our screen
        #if args["display"] > 0:
        ##################
        img = frame

        #Coverting the recorded image to grayscale
        gray = cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)
        
        #Applying filter to remove impurities
        gray = cv2.bilateralFilter(gray,5,1,1)

        #Detecting the face for region of image to be fed to eye classifier
        faces = face_cascade.detectMultiScale(gray, 1.1, 5,minSize=(100,100))

        if(len(faces)>0):

            for (x,y,w,h) in faces:
                #cv2.rectangle(img,(x,y),(x+w,y+h),(0,255,0),2)

                #roi_face is face which is input to eye classifier
                roi_face = gray[y:y+h,x:x+w]
                roi_face_clr = img[y:y+h,x:x+w]
                eyes = eye_cascade.detectMultiScale(roi_face,1.1,2,minSize=(5,5))

                #if eyes are open keep setting time variable t0
                if(len(eyes) >= 1):
                    #for (ex,ey,ew,eh) in eyes:
                        #cv2.rectangle(roi_face_clr,(ex,ey),(ex+ew,ey+eh),(255,0,0),2)
         
                    #cv2.putText(img, "Eyes open!", (70,70), cv2.FONT_HERSHEY_PLAIN, 2,(255,255,255),2)
                    led.on()
                    t0 = time.time()
                    first_read = True
            
                # if eyes are closed start setting t1    
                if (len(eyes) < 1) and first_read:

                    #cv2.putText(img, "Eyes closed!", (70,70), cv2.FONT_HERSHEY_PLAIN, 2,(255,255,255),2)
                    led.off()
                    t1 = time.time()
                    #compare t1 with last set t0 and 3 sec is enough to tell whether driver is sleepy
                    if ((t1 - t0) > 3):

                        #cv2.putText(img, "Driver Sleepy!", (70,30), cv2.FONT_HERSHEY_PLAIN, 2,(255,255,255),2)
                        
                        # t1 -t0 is float value with more than 10 digits precision
                        if (int(t1 - t0) % 3 == 0 ):
                            print("Driver sleepy!",int(t1-t0),"seconds")
                            sec = int(t1 - t0)
                            string = '/usr/bin/pushbullet.sh '+ str(sec)
                            os.system(string)
                            time.sleep(0.8)     #sleep for 0.8 sec so that within a second notification is done only once
                        

        else:
            #cv2.putText(img,"No face detected",(100,100),cv2.FONT_HERSHEY_PLAIN, 3, (0,255,0),2)
            led.on()    
        ##################
        #cv2.imshow("Frame", frame)
        if cv2.waitKey(1) == 27:
                break
        # update the FPS counter
        fps.update()

# stop the timer and display FPS information
fps.stop()
print("[INFO] elasped time: {:.2f}".format(fps.elapsed()))
print("[INFO] approx. FPS: {:.2f}".format(fps.fps()))

# do a bit of cleanup
cv2.destroyAllWindows()
vs.stop()
