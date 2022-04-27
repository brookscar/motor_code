import time
from time import sleep
import pygame

#Open motor
baudRate = 115200
port = "/dev/ttyACM0"
import sys
sys.path.append("../src/open_motor/")
from open_motor_serial import open_motor
comms = open_motor()
comms.init_serial_port(port,baudRate,0.5)

#Controller
done = False
pygame.init()
pygame.joystick.init()

#Stepper Motor
import RPi.GPIO as GPIO
import stepper_motor

def stopped():
    comms.send_pwm_goal(0,0,0,0)
    print("Response:" + comms.get_response())

def forward():
    comms.send_pwm_goal(0,200,200, 0)
    print("Response:" + comms.get_response())
    #speed = 200
    #for increment in range(0,speed):
        #comms.send_pwm_goal(0,increment,-increment, 0)
    print("Response:" + comms.get_response())
    
def backward():
    comms.send_pwm_goal(0,-200,-200, 0)
    print("Response:" + comms.get_response())

def left():
    comms.send_pwm_goal(0,-200,200,0)
    print("Response:" + comms.get_response())

def right():
    comms.send_pwm_goal(0,200,-200,0)
    print("Response:" + comms.get_response())
def leftW():
    comms.send_pwm_goal(0,200,0,0)
    print("Response:" + comms.get_response())

def rightW():
    comms.send_pwm_goal(0,0,200,0)
    print("Response:" + comms.get_response())
    
while not done:

    joystick = pygame.joystick.Joystick(0)  # assigns my remote controller
    joystick.init()  # initializes controller

    axis0 = joystick.get_axis(0)  # axis0 is left and right gets value from my axis(-1,1)
    axis1 = joystick.get_axis(1)  # axis1 is for/backwards
    button0 = joystick.get_button(0)
    button1 = joystick.get_button(1)
    button2 = joystick.get_button(2)
    button3 = joystick.get_button(3)
    button4 = joystick.get_button(4)
    button5 = joystick.get_button(5)
    
    forwardSpeed = abs(axis1)  # math to take axis value it use it to control the PWM and speed
    sideSpeed = abs(axis0)

    if (forwardSpeed >= .96):
        forwardSpeed = forwardSpeed - .04

    if (sideSpeed >= .96):
        sideSpeed = sideSpeed - .04

    if (axis1 < -0.1 and sideSpeed < .6):
        print("forward")
        forward()
        
    elif (axis1 > 0.1 and sideSpeed < .6):
        print("back")
        backward()

    elif (axis0 > .1 and forwardSpeed < .6):
        print("right")
        right()

    elif (axis0 < -.1 and forwardSpeed < .6):
        print("left")
        left()
    
    elif(button0 > 0.6):
        print("verticalMotorDown")
        stepper_motor.verticalMotorUp()
        
    elif(button1 > 0.6):
        print("left turn")
        left()
        
    elif(button2 > 0.6):
        print("verticalMotorUp")
        stepper_motor.verticalMotorDown()
        
    elif(button3 > 0.6):
        print("right turn")
        right()
    
    elif(button4 > 0.6):
        print("left wheel")
        leftW()
        
    elif(button5 > 0.6):
        print("right wheel")
        rightW()
    
    else:
        stopped()
        print("stopped")

    pygame.event.pump()
pygame.quit()
if __name__ == "__main__":
    main() 