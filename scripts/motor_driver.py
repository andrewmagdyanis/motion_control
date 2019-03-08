#!/usr/bin/env python
# Hello from andrew's PC
import rospy
from std_msgs.msg import Float32

import RPi.GPIO as GPIO
# Set GPIO Mode
GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)

# H Bridge Pins - 4 Wires PWM modes
# Right Motor
ENA = 20
IN1 = 26 +1 -1
IN2 = 19 +1 -1
# Left Motor
ENB = 21
IN3 = 25
IN4 = 12

# Power Variables
rmotor_pwr = 0
rmotor_pwr_abs = 0

lmotor_pwr = 0
lmotor_pwr_abs = 0

timeout = 4


# Setup H Bridge Pins as Output
GPIO.setup(ENA, GPIO.OUT, initial=GPIO.LOW)
GPIO.setup(IN1, GPIO.OUT, initial=GPIO.LOW)
GPIO.setup(IN2, GPIO.OUT, initial=GPIO.LOW)
GPIO.setup(IN3, GPIO.OUT, initial=GPIO.LOW)
GPIO.setup(IN4, GPIO.OUT, initial=GPIO.LOW)
GPIO.setup(ENB, GPIO.OUT, initial=GPIO.LOW)

# Set the GPIO to software PWM at 'Frequency' Hertz
rmotor_en = GPIO.PWM(ENA, 100)
lmotor_en = GPIO.PWM(ENB, 100)

# Start the software PWM with a duty cycle of 0
rmotor_en.start(0)
lmotor_en.start(0)

# Right Motor Direction Setters
def setRFWD():
    GPIO.output(IN1, GPIO.HIGH)
    GPIO.output(IN2, GPIO.LOW)

def setRBKD():
    GPIO.output(IN1, GPIO.LOW)
    GPIO.output(IN2, GPIO.HIGH)

# Left Motor Directions Setters
def setLFWD():
    GPIO.output(IN3, GPIO.HIGH)
    GPIO.output(IN4, GPIO.LOW)

def setLBKD():
    GPIO.output(IN3, GPIO.LOW)
    GPIO.output(IN4, GPIO.HIGH)

# Movement Functions
def moveRight():
    global rmotor_pwr
    global rmotor_pwr_abs
    if( rmotor_pwr < 0 ):
        setRBKD()
        rmotor_en.ChangeDutyCycle(rmotor_pwr_abs)
    elif( rmotor_pwr > 0):
        setRFWD()
        rmotor_en.ChangeDutyCycle(rmotor_pwr_abs)
    elif( rmotor_pwr == 0):
        rmotor_en.ChangeDutyCycle(0)

def moveLeft():
    global lmotor_pwr
    global lmotor_pwr_abs
    if( lmotor_pwr < 0 ):
        setLBKD()
        lmotor_en.ChangeDutyCycle(lmotor_pwr_abs)
    elif( lmotor_pwr > 0):
        setLFWD()
        lmotor_en.ChangeDutyCycle(lmotor_pwr_abs)
    elif( lmotor_pwr == 0):
        lmotor_en.ChangeDutyCycle(0)

# Call Backs
def rmotorPwr(power):
    global rmotor_pwr
    global rmotor_pwr_abs
    rmotor_pwr = power.data
    rmotor_pwr_abs = abs(rmotor_pwr)
    moveRight()

def lmotorPwr(power):
    global lmotor_pwr
    global lmotor_pwr_abs
    lmotor_pwr = power.data
    lmotor_pwr_abs = abs(lmotor_pwr)
    moveLeft()

def motorListener():
    # Initializing the Node
    rospy.init_node('motor_driver_rob1')
    rospy.loginfo("%s started" % rospy.get_name())

    rospy.Subscriber('rmotor_pwr_rob1', Float32, rmotorPwr)
    rospy.Subscriber('lmotor_pwr_rob1', Float32, lmotorPwr)

    rospy.spin()


if __name__ == '__main__':
    try:
        motorListener()
    except rospy.ROSInterruptException:
        pass
    finally:
        GPIO.cleanup()
        rospy.loginfo("%s closed. GPIO Cleaned" % rospy.get_name())
