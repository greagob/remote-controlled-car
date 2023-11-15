# *****************************************************************************
# ***************************  Python Source Code  ****************************
# *****************************************************************************
#
#   DESIGNER NAME:  Zach Herring & Ethan Battaglia
#
#       FILE NAME:  final_project.py
#
# DESCRIPTION
#   This program runs a raspberry pi controlled car that can enter manual or
#   line following mode with the push of a button.
#
#   When the program runs a GUI is created where the car can be controlled 
#   with stop, start, throttle control, steering, reverse, and a toggle button
#   to enter line following mode.
#
#   The line following mode uses infrared sensors attached to the bottom of the
#   car to detect when it goes over a black surface or is over a lighter
#   surface, which in this case is going over white. This program allows the 
#   car to switch between both cases, white tape on a blackbackground or black
#   tape on a black background.  
#
# *****************************************************************************
import RPi.GPIO as GPIO
import time
import tkinter as TK
import threading
#---------------------------------------------------
# Constants to be used in program
#---------------------------------------------------

# GPIO number based on BCM GPIO numbering scheme
R_MOTOR_OUTPUT_PIN1 = 27
R_MOTOR_OUTPUT_PIN2 = 17
R_MOTOR_HBRIDGE_PIN = 22

L_MOTOR_OUTPUT_PIN1 = 25
L_MOTOR_OUTPUT_PIN2 = 24
L_MOTOR_HBRIDGE_PIN = 23

PHOTO_SENSOR_PIN1  = 6
PHOTO_SENSOR_PIN2  = 5

# constant to set the motor speed initially off
MOTOR_OFF = GPIO.LOW

# constants to change the motor speed
MOTOR_POSITIVE    = GPIO.HIGH
MOTOR_NEGATIVE    = GPIO.LOW
PWM_FREQUENCY     = 1000
MOTOR_FULL_POWER  = 100
MOTOR_NO_POWER    = 0
TURN_SPEED_ADJUST = 100
FOLLOW_SPEED      = 30

# define the size of the window and position; note these are strings
WINDOW_SIZE     = "550x170"
X_OFFSET        = "+100"
Y_OFFSET        = "+100"
WINDOW_OFFSET   = X_OFFSET + Y_OFFSET
WINDOW_GEOMETRY = WINDOW_SIZE + WINDOW_OFFSET

# delay/sleep values
DELAY_40MS = 0.04
DELAY_1MS  = 0.001

# width of the section of slider that makes the car go straight
POS_SLIDER_BOUND = 8
NEG_SLIDER_BOUND = -8

# global variables
g_goingForward = True   #(boolean) car is configured to move forwards by default
g_isMoving     = False  #(boolean) car does not move on startup
g_followLine   = False  #(boolean) car is manual by default
g_speed        = 100.0  #(float)   car moves at full speed by default
g_direction    = 0      #(int)     car moves forward by default
g_stop_thread  = False  #(boolean) controls line detecting thread


# -----------------------------------------------------------------------------
# DESCRIPTION
#   This function sets up the GPIO interface pins.
#
# INPUT PARAMETERS:
#   none
#
# OUTPUT PARAMETERS:
#   none
#
# RETURN:
#   r_motor_pwm - PWM object that controls the right motor speed and direction
#   l_motor_pwm - PWM object that controls the left motor speed and direction
# -----------------------------------------------------------------------------
def setup_gpio():
    # gpio pin setup
    GPIO.setmode(GPIO.BCM)
    
    # photo sensor pin setup
    GPIO.setup(PHOTO_SENSOR_PIN1, GPIO.IN)
    GPIO.setup(PHOTO_SENSOR_PIN2, GPIO.IN)
    
    # right motor pin setup
    GPIO.setup(R_MOTOR_OUTPUT_PIN1, GPIO.OUT)
    GPIO.setup(R_MOTOR_OUTPUT_PIN2, GPIO.OUT)
    GPIO.setup(R_MOTOR_HBRIDGE_PIN, GPIO.OUT)
    
    # left motor pin setup
    GPIO.setup(L_MOTOR_OUTPUT_PIN1, GPIO.OUT)
    GPIO.setup(L_MOTOR_OUTPUT_PIN2, GPIO.OUT)
    GPIO.setup(L_MOTOR_HBRIDGE_PIN, GPIO.OUT)
    
    # start pwm for each motor
    r_motor_pwm = GPIO.PWM(R_MOTOR_HBRIDGE_PIN, PWM_FREQUENCY)
    l_motor_pwm = GPIO.PWM(L_MOTOR_HBRIDGE_PIN, PWM_FREQUENCY)
    
    # configure right wheel to go forwards
    GPIO.output(R_MOTOR_OUTPUT_PIN1, MOTOR_NEGATIVE)
    GPIO.output(R_MOTOR_OUTPUT_PIN2, MOTOR_POSITIVE)

    # configure left wheel to go forwards
    GPIO.output(L_MOTOR_OUTPUT_PIN1, MOTOR_POSITIVE)
    GPIO.output(L_MOTOR_OUTPUT_PIN2, MOTOR_NEGATIVE)
    
    return (r_motor_pwm, l_motor_pwm)


# -----------------------------------------------------------------------------
# DESCRIPTION
#   This function makes the gui to control the car.
#
# INPUT PARAMETERS:
#   r_motor_pwm - PWM object that controls the right motor speed and direction
#   l_motor_pwm - PWM object that controls the left motor speed and direction
#
# OUTPUT PARAMETERS:
#   none
#
# RETURN:
#   none
# -----------------------------------------------------------------------------
def create_gui(r_motor_pwm, l_motor_pwm):
    global window
    
    window = TK.Tk()
    window.title("RPC Car")
    
    #----------------------- INSTRUCTIONS START -----------------------
    frame0 = TK.Frame(window, bg="black",
                      highlightbackground="gray", highlightthickness=3)
    frame0.pack(side=TK.TOP)
    
    TK.Label(frame0, text="DIRECTIONS:",
             bg="black", fg="white").grid(row=0, column=0)
    TK.Label(frame0, text="Move sliders around to adjust the rotation angle",
             bg="black", fg="white").grid(row=0, column=1)
    
    TK.Label(frame0, text="-50 = fully left; 50 = fully right",
             bg="black", fg="white").grid(sticky = "w", row=1, column=1)
    #----------------------- INSTRUCTIONS END -----------------------

    #----------------------- SCALES START -----------------------
    frame1 = TK.Frame(window)
    frame1.pack(side=TK.BOTTOM, pady=2)
    

    TK.Label(frame1, text="Speed").grid(row=0, column=0, sticky="s")
    
    scaleSpeed = TK.Scale(frame1, from_=35, to=100, length=250, 
                          orient=TK.HORIZONTAL, command=updateMotorSpeed)
    scaleSpeed.grid(row=0, column=1)
    scaleSpeed.set(100)
    
    TK.Label(frame1, text="Angle").grid(row=1, column=0, sticky="s")
    
    scaleDirection = TK.Scale(frame1, from_=-50, to=50, length=250,
                              orient=TK.HORIZONTAL,
                              command=updateMotorDirection)
    scaleDirection.grid(row=1, column=1)
    #----------------------- SCALES END -----------------------
    
    #----------------------- BUTTONS START -----------------------
    lineFollowButton = TK.Button(frame1, text = "Follow Line",
                                 command = followLineToggle)
    lineFollowButton.grid(row=0, column=3, sticky="e")

    window.follow_label = TK.Label(frame1, text = "Mode: Manual")
    window.follow_label.grid(row=0, column=2)
    
    startButton = TK.Button(frame1, text = "Start", command = startMoving,
                            bg = "green")
    startButton.grid(row=2, column=0)
    
    stopButton = TK.Button(frame1, text = "Stop", command = stopMoving,
                           bg = "red")
    stopButton.grid(row=2, column=1)

    reverseButton = TK.Button(frame1, text = "Reverse", command = reverse,
                              bg = "orange")
    reverseButton.grid(row=2, column=2)

    window.reverse_label = TK.Label(frame1, text = "Going forwards")
    window.reverse_label.grid(row=1, column=2, sticky="s")

    quitButton = TK.Button(frame1, text = "Quit", command = window.destroy,
                           bg = "white", highlightbackground="black")
    quitButton.grid(row=2, column=3)
    #----------------------- BUTTONS END -----------------------

    window.r_motor_pwm = r_motor_pwm
    window.l_motor_pwm = l_motor_pwm
    
    window.geometry(WINDOW_GEOMETRY)
    window.mainloop()


# -----------------------------------------------------------------------------
# DESCRIPTION
#   This function is called when the follow line button is pressed. It sets a
#   global variable to say that the car is now in line following mode.
#
# INPUT PARAMETERS:
#   none
#
# OUTPUT PARAMETERS:
#   none
#
# RETURN:
#   none
# -----------------------------------------------------------------------------
def followLineToggle():
    # create scope for global variables so we can modify them
    global g_speed, g_direction, g_followLine
    # invert what g_followLine currently is
    g_followLine = not g_followLine

    if(g_followLine):
        # set globals to 70% speed and straight forward direction
        g_speed = FOLLOW_SPEED
        g_direction = 0
        #local variables
        leftSensor = False
        rightSensor = False
        window.follow_label.configure(text="Mode: Follow")
    else:
        window.follow_label.configure(text="Mode: Manual")
    

# -----------------------------------------------------------------------------
# DESCRIPTION
#   This function is called as a thread in main and is "activated" after the 
#   global variable for following the line is true. When the IR Sensors are
#   triggered, the car will change direction based on which one was triggered.
#
# INPUT PARAMETERS:
#   none
#
# OUTPUT PARAMETERS:
#   none
#
# RETURN:
#   none
# -----------------------------------------------------------------------------
def followLine():
    color_invert = False
    while(not g_stop_thread):
        while(g_followLine):
            leftSensor = GPIO.input(PHOTO_SENSOR_PIN1)
            rightSensor = GPIO.input(PHOTO_SENSOR_PIN2)
                        
            print(leftSensor)
            print(rightSensor)
            
            if (not(leftSensor) and not(rightSensor)):
                color_invert = True
            elif (leftSensor and rightSensor):
                color_invert = False
                
            if(not(color_invert)):
                #left sensor detect black (logic low)
                if(not(leftSensor) and rightSensor):

                    #turn left! (lower left motor speed)
                    window.r_motor_pwm.start(MOTOR_FULL_POWER * g_speed/100)
                    window.l_motor_pwm.start(
                        (MOTOR_FULL_POWER - TURN_SPEED_ADJUST) * g_speed/100)
                    
                    time.sleep(DELAY_40MS)
                    
                #right sensor detect black (logic low)
                if(not(rightSensor) and leftSensor):
                    
                    #turn right! (lower right motor speed)
                    window.r_motor_pwm.start(
                        (MOTOR_FULL_POWER - TURN_SPEED_ADJUST) * g_speed/100)
                    window.l_motor_pwm.start(MOTOR_FULL_POWER * g_speed/100)
                    
                    time.sleep(DELAY_40MS)

                # both sensors are detecting white (logic high)    
                if(leftSensor and rightSensor):
                    window.r_motor_pwm.start(MOTOR_FULL_POWER * g_speed/100)
                    window.l_motor_pwm.start(MOTOR_FULL_POWER * g_speed/100)
                
                    time.sleep(DELAY_40MS)
                time.sleep(DELAY_1MS)
                
            if (color_invert):
                #left sensor detect white (logic high)
                if(leftSensor and not(rightSensor)):
                    
                    #turn left! (lower left motor speed)
                    window.r_motor_pwm.start(MOTOR_FULL_POWER * g_speed/100)
                    window.l_motor_pwm.start(
                        (MOTOR_FULL_POWER - TURN_SPEED_ADJUST) * g_speed/100)
                    
                    time.sleep(DELAY_40MS)
                    
                #right sensor detect white (logic high)
                if(rightSensor and not(leftSensor)):
                    
                    #turn right! (lower right motor speed)
                    window.r_motor_pwm.start(
                        (MOTOR_FULL_POWER - TURN_SPEED_ADJUST) * g_speed/100)
                    window.l_motor_pwm.start(MOTOR_FULL_POWER * g_speed/100)
                    
                    time.sleep(DELAY_40MS)

                # both sensors are detecting black (logic low)    
                if(not(leftSensor) and not(rightSensor)):
                    window.r_motor_pwm.start(MOTOR_FULL_POWER * g_speed/100)
                    window.l_motor_pwm.start(MOTOR_FULL_POWER * g_speed/100)
                
                    time.sleep(DELAY_40MS)
                time.sleep(DELAY_1MS)    


# -----------------------------------------------------------------------------
# DESCRIPTION
#   This function is updates the motor speed while it is running.
#
# INPUT PARAMETERS:
#   speed - this is new desired speed
#
# OUTPUT PARAMETERS:
#   none
#
# RETURN:
#   none
# -----------------------------------------------------------------------------
def updateMotorSpeed(speed):
    global g_speed

    speed = float(speed)
    g_speed = speed
    
    updateMotorDirection(g_direction)
    

# -----------------------------------------------------------------------------
# DESCRIPTION
#   This  function updates the the direction of the car by contorlling how much
#   power each motor is recieving from the raspberry pi.
#
# INPUT PARAMETERS:
#   direction - integer from -50 to +50 that determines if car goes left/right
#
# OUTPUT PARAMETERS:
#   none
#
# RETURN:
#   none
# -----------------------------------------------------------------------------
def updateMotorDirection(direction):
    global g_direction
    direction = int(direction)
    g_direction = direction

    # if we are moving and in manual mode
    if(g_isMoving and not g_followLine):
        #turning right, left motor more power!
        if(direction > POS_SLIDER_BOUND):
            window.r_motor_pwm.start((MOTOR_FULL_POWER - direction*2) * g_speed/100)
            window.l_motor_pwm.start(MOTOR_FULL_POWER * g_speed/100)
            
        #turning left, right motor more power!
        elif(direction < NEG_SLIDER_BOUND):
            window.r_motor_pwm.start(MOTOR_FULL_POWER * g_speed/100)
            window.l_motor_pwm.start((MOTOR_FULL_POWER - abs(direction)*2) * g_speed/100)
            
        #going straight, both at full power!!
        else:
            window.r_motor_pwm.start(MOTOR_FULL_POWER * g_speed/100)
            window.l_motor_pwm.start(MOTOR_FULL_POWER * g_speed/100)   


# -----------------------------------------------------------------------------
# DESCRIPTION
#   This function is called when the start button is pressed on the gui. It
#   toggles a global variable to start the car.
#
# INPUT PARAMETERS:
#   none
#
# OUTPUT PARAMETERS:
#   none
#
# RETURN:
#   none
# -----------------------------------------------------------------------------
def startMoving():
    # create scope for global variable so we can modify it
    global g_isMoving
    g_isMoving = True


# -----------------------------------------------------------------------------
# DESCRIPTION
#   This function is called when the stop button is pressed on the gui. It 
#   stops the car immediatly and sets the global moving variable to false.
#
# INPUT PARAMETERS:
#   none
#
# OUTPUT PARAMETERS:
#   none
#
# RETURN:
#   none
# -----------------------------------------------------------------------------
def stopMoving():
    # create scope for global variable so we can modify it
    global g_isMoving
    global g_followLine
    
    window.r_motor_pwm.start(MOTOR_NO_POWER)
    window.l_motor_pwm.start(MOTOR_NO_POWER)
    
    g_isMoving = False
    g_followLine = False 


# -----------------------------------------------------------------------------
# DESCRIPTION
#   This function is called when the reverse button is pressed on the gui. It 
#   switches the polarity on the motors so that they spin in opposite direction
#   which will make the car go straight backwards.
#
# INPUT PARAMETERS:
#   none
#
# OUTPUT PARAMETERS:
#   none
#
# RETURN:
#   none
# -----------------------------------------------------------------------------
def reverse():
    # create scope for global variable so we can modify it
    global g_goingForward

    # If car is already going forwards, then go backwards
    if(g_goingForward):

        # configure right wheel to go backwards
        GPIO.output(R_MOTOR_OUTPUT_PIN1, MOTOR_POSITIVE)
        GPIO.output(R_MOTOR_OUTPUT_PIN2, MOTOR_NEGATIVE)

        # configure left wheel to go backwards
        GPIO.output(L_MOTOR_OUTPUT_PIN1, MOTOR_NEGATIVE)
        GPIO.output(L_MOTOR_OUTPUT_PIN2, MOTOR_POSITIVE)
        
        window.reverse_label.configure(text="Going backwards")

        # toggle global variable
        g_goingForward = False

    # If car is already going backwards, then go forwards
    else:

        # configure right wheel to go forwards
        GPIO.output(R_MOTOR_OUTPUT_PIN1, MOTOR_NEGATIVE)
        GPIO.output(R_MOTOR_OUTPUT_PIN2, MOTOR_POSITIVE)

        # configure left wheel to go forwards
        GPIO.output(L_MOTOR_OUTPUT_PIN1, MOTOR_POSITIVE)
        GPIO.output(L_MOTOR_OUTPUT_PIN2, MOTOR_NEGATIVE)
        
        window.reverse_label.configure(text="Going forwards")

        # toggle global variable
        g_goingForward = True 


# -----------------------------------------------------------------------------
# DESCRIPTION
#   This function contains the clean-up code to place the GPIO in a safe state
#   before exiting the program.
#
# INPUT PARAMETERS:
#   r_motor_pwm - PWM object that controls the right motor speed and direction
#   l_motor_pwm - PWM object that controls the left motor speed and direction
#
# OUTPUT PARAMETERS:
#   none
#
# RETURN:
#   none
# -----------------------------------------------------------------------------
def destroy(r_motor_pwm, l_motor_pwm):
    # stop motors
    r_motor_pwm.stop()
    l_motor_pwm.stop()
    GPIO.cleanup()
    
    try:
        window.destroy()
        print("Window destroyed")
    except:
        print("Window already destroyed")

#---------------------------------------------------------------------
#  main() function
#---------------------------------------------------------------------
def main():
    global g_stop_thread
    
    r_motor_pwm = 0
    l_motor_pwm = 0
    
    print()
    print("****************  PROGRAM IS RUNNING  ****************")
    print()
    print("Press CTRL-c to end the program.")
    print()
    
    try:
        
        r_motor_pwm, l_motor_pwm = setup_gpio()
        
        task = threading.Thread(target=followLine)
        task.start()
        
        create_gui(r_motor_pwm, l_motor_pwm)
                    
    except KeyboardInterrupt:
        print()
        print("CTRL-c detected.")
        print()
        
    finally:
        destroy(r_motor_pwm, l_motor_pwm)
        g_stop_thread = True
        task.join()
        
        print("GPIO Port has been cleaned up.")
        print()
        print("****************  PROGRAM TERMINATED  ****************")
        print()
 
# if file execute standalone then call the main function.
if __name__ == '__main__':
    main()
