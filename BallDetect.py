import cv2
import time
from controlDriver import controldriver
from controlDriver import find

import RPi.GPIO as GPIO


scot = controldriver()
scots = find()

time.sleep(0.1)
counter =0
start_time = time.time()
j=0
k=0
i=0

while True:    
    scot.offLEDSwitch()
    if i ==0:
        scot.rainbow()
        time.sleep(2)
        i = i + 1
    if scot.switchonoff():        
        while True:
            i= 0
            if scot.switchonoff() == False:
                scot.tomare()
                break
            scot.onLEDSwitch()
            if scots.iroBreak():#ball ari
                scot.found1()
                while True:
                    if j == 0:
                        print("301")
                        scots.ballAri()
                    if j < 4:
                        print("300")
                        j = j + 1
                    else: j = 0
                    if scots.iroBreak()== False:break
                    if scot.switchonoff() == False:
                        scot.tomare()
                        break
                
            else:#ball nai
                k = k+1
                scots.ballArimasen(k)
                if k > 3:k=0
    
GPIO.cleanup()
video.release()
cv2.destroyAllWindows()

