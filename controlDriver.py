import RPi.GPIO as GPIO
import time
import cv2
import numpy as np
import pigpio
from pca9685 import pca9685
import board
import neopixel
import serial

#Motor pin
AIN1=5
AIN2=12
BIN1=6
BIN2=13
USB_PORT = "/dev/ttyACM0"  # Arduino Uno WiFi Rev24

try:
   usb = serial.Serial(USB_PORT, 9600, timeout=2)
except:
   print("ERROR - Could not open USB serial port.  Please check your port name and permissions.")
   print("Exiting program.")
   exit()

# NeoPixels must be connected to D10, D12, D18 or D21 to work.
pixel_pin = board.D10

# The number of NeoPixels
num_pixels = 64
#Servo pin
srvLeft = 2
srvRight = 3

#switch pin
gpio_sw = 16
led_switch = 20

CAMERA_FLIP = 1
cam_center_y = 240
cam_center_x = 320

cam = cv2.VideoCapture( 0 )

#list = greenLow, organce, blue, pink, yellow
#Green is 0
#Organce is 2
#Blue is 4
#Pink is 6
#Yellow is 8
#Brown is 10
List_iRo = [[38,105,120],[85,255,255],[0,84,160],[10,226,216],[82,145,110],[160,255,255],[105,66,62],[193,215,255],[17,106,147],[66,255,255],[1,58,0],[30,216,109]]
iRo =10
GPIO.setmode(GPIO.BCM)

output_ports = [23]# Define the GPIO output port numbers
input_ports = [24]# Define the GPIO input port numbers
trig=23 # set trigger port
echo=24 # get echo

output_ports_left = [27]# Define the GPIO output port numbers
input_ports_left = [4]# Define the GPIO input port numbers
trig_left=27 # set trigger port
echo_left=4 # get echo

output_ports_right = [7]# Define the GPIO output port numbers
input_ports_right = [8]# Define the GPIO input port numbers
trig_right=7 # set trigger port
echo_right=8 # get echo

GPIO.setwarnings(False)

for bit in output_ports:# Set up the six output bits
    GPIO.setup (bit,GPIO.OUT)
    GPIO.output (bit,False)# Initially turn them all off

for bit in input_ports: # Set up the six input bits
    GPIO.setup (bit,GPIO.IN, pull_up_down = GPIO.PUD_DOWN)


for bit in output_ports_left:# Set up the six output bits
    GPIO.setup (bit,GPIO.OUT)
    GPIO.output (bit,False)# Initially turn them all off

for bit in input_ports_left: # Set up the six input bits
    GPIO.setup (bit,GPIO.IN, pull_up_down = GPIO.PUD_DOWN)

for bit in output_ports_right:# Set up the six output bits
    GPIO.setup (bit,GPIO.OUT)
    GPIO.output (bit,False)# Initially turn them all off

for bit in input_ports_right: # Set up the six input bits
    GPIO.setup (bit,GPIO.IN, pull_up_down = GPIO.PUD_DOWN)

GPIO.setup(srvLeft, GPIO.OUT)
GPIO.setup(srvRight, GPIO.OUT)

servoLeft = GPIO.PWM(srvLeft, 50)     #GPIO.PWM(ポート番号, 周波数[Hz])
servoRight = GPIO.PWM(srvRight, 50)



GPIO.setup(AIN1, GPIO.OUT)
GPIO.setup(AIN2, GPIO.OUT)
GPIO.setup(BIN1, GPIO.OUT)
GPIO.setup(BIN2, GPIO.OUT)


# スイッチピンを入力、プルアップに設定
GPIO.setup(gpio_sw, GPIO.IN, pull_up_down=GPIO.PUD_UP)
GPIO.setup(led_switch, GPIO.OUT)

a1 = GPIO.PWM(5, 50) #50Hz
a2 = GPIO.PWM(12, 50) #50Hz
b1 = GPIO.PWM(6, 50) #50Hz
b2 = GPIO.PWM(13, 50)

a1.start(0)
a2.start(0)
b1.start(0)
b2.start(0)

servoLeft.start(0)
servoRight.start(0)

SERVO_X_CH = 0
SERVO_Y_CH = 3

SERVO_X_REV = 1
SERVO_Y_REV = 1

SERVO_MAX_DEG = 90
SERVO_MIN_DEG = -90

DEG_MIN = 3
pi = pigpio.pi()

IDENT_AREA = 50

PCA9685_ADDR = 0x40
pwm_freq = 50


# For RGBW NeoPixels, simply change the ORDER to RGBW or GRBW.
ORDER = neopixel.GRB

pixels = neopixel.NeoPixel(
    pixel_pin, num_pixels, brightness=0.2, auto_write=False, pixel_order=ORDER
)

io_initialized = False
led1 = False
j=0

class controldriver:

    def io_init(self):
        global io_initialized
        if io_initialized:
            return
        io_initialized = True

    def rainbow(self):
        usb.write(b'led_on')
    def red(self):
        usb.write(b'red')
    def blue(self):
        usb.write(b'blue')
    def offLed(self):
        usb.write(b'led_off')
    def onLEDSwitch(self):
        GPIO.output(led_switch, GPIO.HIGH)
    def offLEDSwitch(self):
        GPIO.output(led_switch, GPIO.LOW)
    def found(self):
        usb.write(b'find')
    def found1(self):
        usb.write(b'pink')

    def onled(self):
        usb.write(b'red')
        while True:
            if find.iroBreak(self) == False:#ball ari
#                 usb.write(b'led_on')#led off
                break
            if controldriver.switchonoff(self) == False:
                print("gia tri nut bam: ",controldriver.switchonoff(self))
#                 controldriver.rainbow(self)
                break

    def switchonoff(self):
        new = GPIO.input(gpio_sw)
        global j
        global led1
        if(j==1 and new==0):
            led1 = not led1
        j=new;
        return led1


    def distance(self, trigPin, echoPin):
        distance = 0  # Set initial distance to zero

        GPIO.output (trigPin,False)# Ensure the 'Trig' pin is low for at
        time.sleep (0.03)   # least 30mS

        GPIO.output (trigPin,True)# Turn on the 'Trig' pin for 10us
        time.sleep (1e-5)

        GPIO.output (trigPin,False)# Turn off the 'Trig' pin
        time.sleep (1e-5)

        time1, time2 = time.time(), time.time()                  #init times

        while not GPIO.input (echoPin):# Wait for the start of the 'Echo' pulse
            time1 = time.time()# Get the time the 'Echo' pin goes high
            if time1 - time2 > 0.02:# If the 'Echo' pin doesn't go high after 20mS
                distance = -3 # then set 'distance' to 100
                break   # and break out of the loop

        if distance == -3: # If a sensor error has occurred
            return (100)  # then exit with 100 (sensor fault)

        time2 = time.time()
        while GPIO.input (echoPin):# Otherwise, wait for the 'Echo' pin to go low
            time2 = time.time()# Get the time the 'Echo' pin goes low
            if (time2 - time1 > 0.02):# If the 'Echo' pin doesn't go low after 20mS
                distance = -4 # then ignore it and set 'distance' to 100
                break # and break out of the loop
        if distance == -4: # If a sensor error has occurred
            return (150)
        if (time2 - time1 < 3.0e-5):# If the 'Echo' pin went low too fast
            distance = -5

        if distance < 0:    # If a sensor error has occurred
            return (distance)  # then exit with error code
           # Sound travels at approximately 2.95uS per mm
           # and the reflected sound has travelled twice
           # the distance we need to measure (sound out,
           # bounced off object, sound returned)

        distance = (time2 - time1) / 0.00000295 / 2 / 10
        # Convert the timer values into centimetres
        return (distance)   # Exit with the distance in centimetres

    def ugoki(self, c1, c2, c3, c4, c5):
        a1.ChangeDutyCycle(c1)
        a2.ChangeDutyCycle(c2)
        b1.ChangeDutyCycle(c3)
        b2.ChangeDutyCycle(c4)
        time.sleep(c5)

    def run(self, duty, jikan):
        controldriver.ugoki(self,duty,0,0,duty,jikan)

    def back(self, duty, jikan):
        controldriver.ugoki(self,0,duty,duty,0,jikan)

    def spin_left(self, duty, jikan):
        controldriver.ugoki(self,0,duty,0,duty,jikan)

    def left(self, duty, jikan):
        controldriver.ugoki(self,0,duty,0,0,jikan)

    def spin_right(self, duty, jikan):
        controldriver.ugoki(self,duty,0,duty,0,jikan)

    def right(self, duty, jikan):
        controldriver.ugoki(self,0,0,duty,0,jikan)

    def kyukei(self):
        controldriver.ugoki(self,0,0,0,0,0.2)
        time.sleep(0.5)

    def tomare(self):
        controldriver.ugoki(self,100,100,100,100,0.05)

    def dis_center(self):
        return controldriver.distance(self, trig, echo)

    def dis_left(self):
        return controldriver.distance(self, trig_left, echo_left)

    def dis_right(self):
        return controldriver.distance(self, trig_right, echo_right)

    def ballDetect(self):
        a, b, detectBall, c, d = controldriver.camDetect(self)
        i = 0
        for c in detectBall:
            area=cv2.contourArea(c)
            if area>1000:
                peri=cv2.arcLength(c,True)
                approx=cv2.approxPolyDP(c,0.02*peri,True)
                x,y,w,h=cv2.boundingRect(c)
                print(len(approx))
                if len(approx)==4:
                    i = 1
                elif len(approx)==3:
                    i = 2
                elif len(approx)>=7 and len(approx)<=9:
                    i = 3
                elif len(approx)>7:
                    i = 4
        return i

    def camDetect(self):
        ( ret, img ) = cam.read()
        if( CAMERA_FLIP == 1 ):
            img = cv2.flip( img, 0 )
        hsv_img = cv2.cvtColor( img, cv2.COLOR_BGR2HSV )
        iRoLow = np.array(List_iRo[iRo])
        a = iRo +1
        iRoHigh = np.array(List_iRo[a])
        ball_mask = cv2.inRange( hsv_img, iRoLow , iRoHigh )
        ( ditect_ball, ret ) = cv2.findContours( ball_mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        ditect_ball = sorted( ditect_ball, key=lambda x:cv2.contourArea(x), reverse=True )

        max_s = 0
        sel = [ 0, 0, 0, 0 ]

        for i in ditect_ball:
            ( x, y, w, h ) = cv2.boundingRect( i )
            s = w * h
            if ( s > max_s ):
                max_s = s
                sel = [ x, y, w, h ]
        return max_s, sel, ditect_ball, img, ball_mask

    def checkBall(self):
        dienTich, a,b,c,d = controldriver.camDetect(self)
        center, a,b = controldriver.kyori(self)
        d_x = find.xkeisan(self)
        if center > 40 and center < 70:
            if (dienTich < 10000 and dienTich > 2000):
                return 1
            return 2

        if(d_x < -40 or d_x > 40):
            print("114")
            controldriver.tomare(self)
            if ( d_x < -40):
                controldriver.spin_right(self,70,0.05)
                print("115")
            if ( d_x > 40):
                controldriver.spin_left(self,70,0.05)
                print("116")
            controldriver.tomare(self)
        return 0

    def kyori(self):
        center = controldriver.dis_center(self)
        left = controldriver.dis_left(self)
        right = controldriver.dis_right(self)
        return center, left, right

    def valid(self):
        center, left, right = controldriver.kyori(self)

        if(center > 0 and left > 0 and right > 0):
            if(center > 15 and left > 15 and right > 15):
                return 1
            elif(center < 20 or left < 20 or right < 20):
                if(center < 20):
                    return 2
                if(left < 20):
                    return 3
                if(right < 20):
                    return 4
        else:
            return -1

class find:
    k=0
    def iroBreak(self):
        max_s,a,b,c,d = controldriver.camDetect(self)
        if(max_s > 1000):
            return True
        return False

    def ykeisan(self):
        max_s, sel, img,c,d = controldriver.camDetect(self)
        if find.iroBreak(self):
            (x, y, w, h)= sel
            ball_center_y = int( y + h / 2 )
            d_y = ball_center_y - cam_center_y
        else:
            d_y = 0
        return d_y

    def xkeisan(self):
        max_s, sel, img,c ,d = controldriver.camDetect(self)
        if find.iroBreak(self):
            (x, y, w, h)= sel
            ball_center_x = int( x + w / 2 )
            d_x = ball_center_x - cam_center_x
        else:
            d_x = 0
        return d_x

    def dieukien(self, dk):
        for var1 in range(4):
            if(dk == 2 or dk == 3):
                for i in range(2):
                    controldriver.spin_right(self,100,0.01)
                    print("turn right")
                    if find.iroBreak(self):#ball ari
                        controldriver.tomare(self)
                        break
            elif(dk == 4):
                for i in range(2):
                    controldriver.spin_left(self,100,0.01)
                    print("turn left")
                    if find.iroBreak(self):#ball ari
                        controldriver.tomare(self)
                        break
            if find.iroBreak(self):#ball ari
                controldriver.tomare(self)
                break

    def ballAri(self):
        d_x = find.xkeisan(self)
        print("100")
        if(d_x > -40 and d_x < 40):

            print("101")
            controldriver.kyukei(self)
            kyori=controldriver.dis_center(self)
            i = controldriver.checkBall(self)
            if i ==2:
                controldriver.back(self,100,0.5)
                controldriver.spin_left(self,100,0.5)
            if(kyori>25):
                print("102")
                controldriver.run(self,100,0.5)
                controldriver.kyukei(self)
            elif(kyori>15 and kyori < 31):
                print("103")
                controldriver.run(self,80,0.2)
                controldriver.tomare(self)
            elif(kyori<20):
                controldriver.found(self)
                time.sleep(2)
                print("104")
                i = 0
                j = 0
                h = 0
                for var in range(60):
                    print("105")
                    i = controldriver.ballDetect(self)
                    if i == 3:
                        j = j + 1
                    else: h = h + 1
                if j > h:
                    print("106")
                    while True:
                        kyori1=controldriver.dis_center(self)
                        if find.iroBreak(self) == False:#ball ari
                            controldriver.tomare(self)
                            print("107")
                            break
                        d_x1 = find.xkeisan(self)
                        if(d_x1 < -40 or d_x1 > 40):
                            controldriver.tomare(self)
                            print("108")
                            if ( d_x1 < -40):
                                controldriver.spin_right(self,70,0.06)
                                print("109")
                                print("4")
                            if ( d_x1 > 40):
                                controldriver.spin_left(self,70,0.06)
                                print("110")
                            controldriver.tomare(self)
                        if(kyori1>3.5):
                            controldriver.run(self,70,0.06)
                            controldriver.tomare(self)
                            print("111")
                        var1 = 0
                        for var in range(5):
                            kyori2=controldriver.dis_center(self)
                            if kyori2> var1:
                                var1 = kyori2
                        if(var1<4.5 and var1>0):
                            print("おめでとうございます")
                            controldriver.onled(self)
                            while True:

                                if find.iroBreak(self) == False:#ball ari
                                    controldriver.kyukei(self)
                                    controldriver.rainbow(self)
                                    break
                else:
                    controldriver.back(self,100,0.5)
                    controldriver.spin_left(self,100,0.5)
                    print("113")

            else:
                controldriver.kyukei(self)
        if(d_x < -40 or d_x > 40):
            print("114")
            controldriver.tomare(self)
            if ( d_x < -40):
                controldriver.spin_right(self,80,0.05)
                print("115")
            if ( d_x > 40):
                controldriver.spin_left(self,80,0.05)
                print("116")
            controldriver.tomare(self)

    def ballArimasen(self, k):
        distanceA = controldriver.valid(self)
        if(distanceA == 1):#when all sensors are valid
            print("901")
            for i in range(50):
                print("902")
                controldriver.run(self,100,0.01)
                if find.iroBreak(self):#ball ari
                    controldriver.tomare(self)
                    print("903")
                    break
                a = controldriver.valid(self)
                if a == -1 or a == 2 or a == 3 or a == 4:
                    controldriver.tomare(self)
                    print("904")
                    break
                if controldriver.switchonoff(self) == False:break
            if find.iroBreak(self) == False:
                if k > 3:
                    for i in range(18):
                        print("905")
                        controldriver.spin_left(self,80,0.1)
                        controldriver.kyukei(self)
                        if find.iroBreak(self):#ball ari
                            controldriver.tomare(self)
                            print("906")
                            break
                        a = controldriver.valid(self)
                        if a == -1 or a == 2 or a == 3 or a == 4:
                            controldriver.tomare(self)
                            print("907")
                            break
                        if controldriver.switchonoff(self) == False:break

        distanceB = controldriver.valid(self)
        if find.iroBreak(self) == False:
            if(distanceB == 2 or distanceB == 3 or distanceB == 4):
                controldriver.back(self,100,1)
                print("908")
                if(distanceB == 2):
                    print("909")
                    find.dieukien(self,distanceB)
                elif(distanceB == 3):
                    print("910")
                    find.dieukien(self,distanceB)
                elif(distanceB == 4):
                    print("911")
                    find.dieukien(self,distanceB)
                controldriver.tomare(self)
        if(distanceB == -1 or distanceA == -1):
            print("912")
            controldriver.tomare(self)