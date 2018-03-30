#!/usr/bin/env python
# -*- coding: utf-8 -*-
from Adafruit_PWM_Servo_Driver import PWM
import RPi.GPIO as GPIO
import thread
import time
import sys
import serial
import tty
import select
import termios

#ser = serial.Serial("/dev/ttyAMA0",9600)  #串口波特率设置

# Initialise the PWM device using the default address
# bmp = PWM(0x40, debug=True)
pwm = PWM(0x40,debug = False)

servoMin = 150  # Min pulse length out of 4096
servoMax = 600  # Max pulse length out of 4096
#SERVO
myservo1 = 0
myservo2 = 1
myservo3 = 2
myservo4 = 3

SERVOS = 4  #舵机数4个

#定义舵机的角度值初始化
#手爪
ms1MIN = 13
ms1MAX = 50
ms1INITANGLE = 30
ms1currentAngle = 0

#上臂电机
ms2MIN = 90
ms2MAX = 170
ms2INITANGLE =100
ms2currentAngle = 0

#下臂电机
ms3MIN = 40
ms3MAX = 170
ms3INITANGLE = 70
ms3currentAngle = 0

#底座
ms4MIN = 0
ms4MAX = 170
ms4INITANGLE = 90
ms4currentAngle = 0

ServoDelayTime = 0.05 #舵机响应时间
delta = 5        #舵机转动幅度
delta_bottom = 2 #底座舵机转动幅度

#car 
GPIO_M1 = 27
GPIO_M11 = 5
GPIO_M2 = 6
GPIO_M22 = 12
GPIO_M3 = 13
GPIO_M33 = 19
GPIO_M4 = 17
GPIO_M44 = 18
GPIO_S = 21 
GPIO_R1 = 20
#GPIO_R2 = 21
GPIO_R2 = 22
GPIO_STEP1 = 23
GPIO_STEP2 = 24
GPIO_STEP3 = 25
GPIO_STEP4 = 26
car_sleep = 0.05
var1 = 0
var2 = 0
signal = 0

def setServoPulse(channel, pulse):
  pulseLength = 1000000.0                   # 1,000,000 us per second
  pulseLength /= 50.0                       # 60 Hz
  print "%d us per period" % pulseLength
  pulseLength /= 4096.0                     # 12 bits of resolution
  print "%d us per bit" % pulseLength
  pulse *= 1000.0
  pulse /= (pulseLength*1.0)
# pwmV=int(pluse)
  print "pluse: %f  " % (pulse)
  pwm.setPWM(channel, 0, int(pulse))

#Angle to PWM
def write(servonum,x):
  y=x/90.0+0.5
  y=max(y,0.5)
  y=min(y,2.5)
  setServoPulse(servonum,y)

def setStep(w1, w2, w3, w4):  
	GPIO.output(GPIO_STEP1, w1)  
	time.sleep(0.01)
	GPIO.output(GPIO_STEP2, w2)  
	time.sleep(0.01)
	GPIO.output(GPIO_STEP3, w3)  
	time.sleep(0.01)	
	GPIO.output(GPIO_STEP4, w4)
	time.sleep(0.01)

def b_up():
	for i in range(5):
		setStep(1, 0, 0, 0)
		setStep(0, 1, 0, 0)
		setStep(0, 0, 1, 0)
		setStep(0, 0, 0, 1)

def b_down():
	i = 0
	while(i < 5):
        	setStep(0, 0, 0, 1)        
        	setStep(0, 0, 1, 0)
        	setStep(0, 1, 0, 0)
        	setStep(1, 0, 0, 0)
		i+=1

def t_down():
        GPIO.output(GPIO_M1,GPIO.LOW)
	GPIO.output(GPIO_M11,GPIO.HIGH)
	GPIO.output(GPIO_M2,GPIO.LOW)
	GPIO.output(GPIO_M22,GPIO.HIGH)
	GPIO.output(GPIO_M3,GPIO.LOW)
	GPIO.output(GPIO_M33,GPIO.HIGH)
	GPIO.output(GPIO_M4,GPIO.LOW)
	GPIO.output(GPIO_M44,GPIO.HIGH)        

def t_stop():
	GPIO.output(GPIO_M1,GPIO.LOW)
        GPIO.output(GPIO_M11,GPIO.LOW)
        GPIO.output(GPIO_M2,GPIO.LOW)
        GPIO.output(GPIO_M22,GPIO.LOW)
        GPIO.output(GPIO_M3,GPIO.LOW)
        GPIO.output(GPIO_M33,GPIO.LOW)
        GPIO.output(GPIO_M4,GPIO.LOW)
        GPIO.output(GPIO_M44,GPIO.LOW)
        
def t_up():
	GPIO.output(GPIO_M1,GPIO.HIGH)
        GPIO.output(GPIO_M11,GPIO.LOW)
        GPIO.output(GPIO_M2,GPIO.HIGH)
        GPIO.output(GPIO_M22,GPIO.LOW)
        GPIO.output(GPIO_M3,GPIO.HIGH)
        GPIO.output(GPIO_M33,GPIO.LOW)
        GPIO.output(GPIO_M4,GPIO.HIGH)
        GPIO.output(GPIO_M44,GPIO.LOW)

def t_left():
	GPIO.output(GPIO_M1,GPIO.HIGH)
        GPIO.output(GPIO_M11,GPIO.LOW)
        GPIO.output(GPIO_M2,GPIO.LOW)
        GPIO.output(GPIO_M22,GPIO.HIGH)
        GPIO.output(GPIO_M3,GPIO.HIGH)
        GPIO.output(GPIO_M33,GPIO.LOW)
        GPIO.output(GPIO_M4,GPIO.LOW)
        GPIO.output(GPIO_M44,GPIO.HIGH)

def t_right():
	GPIO.output(GPIO_M1,GPIO.LOW)
        GPIO.output(GPIO_M11,GPIO.HIGH)
        GPIO.output(GPIO_M2,GPIO.HIGH)
        GPIO.output(GPIO_M22,GPIO.LOW)
        GPIO.output(GPIO_M3,GPIO.LOW)
        GPIO.output(GPIO_M33,GPIO.HIGH)
        GPIO.output(GPIO_M4,GPIO.HIGH)
        GPIO.output(GPIO_M44,GPIO.LOW)

def checkdist(GPIO_R,var):
	global signal
	while True:
		if signal == 1:
			continue
		else:
			signal = 1
		print GPIO_R
		#发出发信号
        	GPIO.output(GPIO_S,GPIO.HIGH)
        	#保持15us
        	time.sleep(0.000015)
        	GPIO.output(GPIO_S,GPIO.LOW)
        	while not GPIO.input(GPIO_R):
                	pass
       		#发现高电平时开时计时
        	t1 = time.time()
        	while GPIO.input(GPIO_R):
                	pass
        	#高电平结束停止计时
        	t2 = time.time()
        	#返回距离，单位为米
        	dist = (t2-t1)*340/2
 		if dist < 0.05:
			t_stop()
			var = 1
			time.sleep(1)
		else:
			var = 0
		print dist
		signal = 0
		time.sleep(0.01)
           
def setup():
        GPIO.setwarnings(False)
        GPIO.setmode(GPIO.BCM)

	GPIO.setup(GPIO_M1,GPIO.OUT)
	GPIO.setup(GPIO_M11,GPIO.OUT)
	GPIO.setup(GPIO_M1,GPIO.OUT)
        GPIO.setup(GPIO_M2,GPIO.OUT)
	GPIO.setup(GPIO_M22,GPIO.OUT)
        GPIO.setup(GPIO_M3,GPIO.OUT)
	GPIO.setup(GPIO_M33,GPIO.OUT)
        GPIO.setup(GPIO_M4,GPIO.OUT)
	GPIO.setup(GPIO_M44,GPIO.OUT)
        GPIO.setup(GPIO_S,GPIO.OUT)
	GPIO.setup(GPIO_STEP1,GPIO.OUT)
        GPIO.setup(GPIO_STEP2,GPIO.OUT)
	GPIO.setup(GPIO_STEP3,GPIO.OUT)
        GPIO.setup(GPIO_STEP4,GPIO.OUT)
	GPIO.setup(GPIO_R1,GPIO.IN)
        GPIO.setup(GPIO_R2,GPIO.IN)
        #GPIO.setup(GPIO_R3,GPIO.IN)
        
def Re_Servo():
  global  ms1currentAngle
  global  ms2currentAngle
  global  ms3currentAngle
  global  ms4currentAngle  
  write(myservo1,ms1INITANGLE)   #手爪  
  write(myservo2,ms2INITANGLE)   #上臂  
  write(myservo3,ms3INITANGLE)   #下臂  
  write(myservo4,ms4INITANGLE)   #底座
  
  ms1currentAngle = ms1INITANGLE
  ms2currentAngle = ms2INITANGLE
  ms3currentAngle = ms3INITANGLE
  ms4currentAngle = ms4INITANGLE  
  time.sleep(1)
#-------------------机械臂运动函数定义----------------
def ClampOpen():  #手爪打开
  write(myservo1,ms1MAX)
  time.sleep(0.3)

def ClampClose(): #手臂闭合
  write(myservo1,ms1MIN)
  time.sleep(0.3)

def BottomLeft(): #底座左转
  global ms4currentAngle
  if(ms4currentAngle+delta_bottom) < ms4MAX:
    ms4currentAngle+=delta_bottom
  write(myservo4,ms4currentAngle)    

def BottomRight(): #底座右转
  global ms4currentAngle
  if(ms4currentAngle-delta_bottom) > ms4MIN:
    ms4currentAngle-=delta_bottom
  write(myservo4,ms4currentAngle)

def Arm_A_Up(): #上臂舵机向上
  global ms2currentAngle
  if(ms2currentAngle + delta) < ms2MAX:
    ms2currentAngle += delta
  write(myservo2,ms2currentAngle)

def Arm_A_Down():#上臂舵机向下
  global ms2currentAngle
  if(ms2currentAngle - delta) > ms2MIN:
    ms2currentAngle -= delta
  write(myservo2,ms2currentAngle)
  
def Arm_B_Up():#下臂舵机向上
  global ms3currentAngle
  if(ms3currentAngle - delta) > ms3MIN:
    ms3currentAngle -= delta
  write(myservo3,ms3currentAngle)

def Arm_B_Down(): #下臂舵机向下
  global ms3currentAngle
  if(ms3currentAngle + delta) < ms3MAX:
    ms3currentAngle += delta
  write(myservo3,ms3currentAngle) 

def Servo_stop(): #停止所有舵机
  write(myservo1,ms1currentAngle)
  write(myservo2,ms2currentAngle)
  write(myservo3,ms3currentAngle) 
  write(myservo4,ms4currentAngle) 
 
def isData():
	return select.select([sys.stdin],[],[],0)!=([sys.stdin],[],[]) 

def loop():
	i=1
	try:
		thread.start_new_thread(checkdist,(GPIO_R1,var1,))
		thread.start_new_thread(checkdist,(GPIO_R2,var2,))
	except:
		print "Error:unable to start thread"
        while True:
	  #command=ser.read()
          #ser.write(command +'\n')#回显
		time.sleep(0.001)
		if isData():
			print i
			i+=1
			command=sys.stdin.read(1)
			print command
			if command == 'w':
				t_up()    #前进
			elif command == 's':
				t_down()  #后退
			elif command == 'd' and var2 == 0:
				t_right() #右转
			elif command == 'a' and var1 == 0:
				t_left()  #左转
			elif command == 'q':
				b_up()
			elif command == 'e':
				b_down()
			#time.sleep(car_sleep)
			elif command == 'f':
				t_stop()
			if command == '0':
			#ser.write("Servo all stop\n")
				Servo_stop()
				time.sleep(ServoDelayTime)
			elif command =='1':  #底座左转
			#ser.write("MeArm turn Left\n")
				BottomLeft()
			#time.sleep(ServoDelayTime)
			elif command =='2':  #底座右转
			#ser.write("MeArm turn Right\n")
				BottomRight()
			#time.sleep(ServoDelayTime)
			elif command =='3':  #上臂舵机向上
			#ser.write("Arm A Up\n")
				Arm_A_Up()
			#time.sleep(ServoDelayTime)
			elif command =='4':  #上臂舵机向下
			#ser.write("Arm A Down\n")
				Arm_A_Down()
			#time.sleep(ServoDelayTime)         
			elif command =='5':   #下臂舵机向上
			#ser.write("Arm B Up\n")
				Arm_B_Up()
			#time.sleep(ServoDelayTime) 
			elif command =='6':    #下臂舵机向下
			#ser.write("Arm B Down\n")
				Arm_B_Down()
			#time.sleep(ServoDelayTime)            
			elif command =='7':  #打开手爪 （加速）
			#ser.write("Clamp Open\n")
				ClampOpen()
			elif command =='8':  #闭合手爪 （减速）
			#ser.write("Clamp Close\n")
				ClampClose()
			elif command =='\x1b':break
def destroy():
	GPIO.cleanup()

if __name__ == "__main__":
        setup()
        Re_Servo()

	old_settings=termios.tcgetattr(sys.stdin)
        try:

		tty.setcbreak(sys.stdin.fileno())
		loop()
        #except KeyboardInterrupt:
                destroy()
		print "end"
	except KeyboardInterrupt:
		destory()
		print "end"
	finally:
		termios.tcsetattr(sys.stdin,termios.TCSADRAIN,old_settings)
