# ---------------------------------------------------------------------------- #
#                                                                              #
# 	Module:       main.py                                                      #
# 	Author:                                                                    #
# 	Created:                                                                   #
# 	Description:  V5 project                                                   #
#                                                                              #
# ---------------------------------------------------------------------------- #

# Library imports
from math import pi, fmod, sin, cos, atan2, hypot
from vex import *

#---------------------------------------------------------------------------#
#                            Robot Configuration                            #
#---------------------------------------------------------------------------#

# A global instance of brain used for printing to the V5 Brain screen
brain = Brain()

# VEXcode device constructors
controller = Controller(PRIMARY)

# Add motors here
drive_LF = Motor(Ports.PORT1, GearSetting.RATIO_6_1, False)
drive_LB = Motor(Ports.PORT2, GearSetting.RATIO_6_1, False)
drive_RF = Motor(Ports.PORT3, GearSetting.RATIO_6_1, True)
drive_RB = Motor(Ports.PORT4, GearSetting.RATIO_6_1, True)
placeholder_motor_5 = Motor(Ports.PORT5, GearSetting.RATIO_6_1, True)

# Add sensors here
inertial = Inertial(Ports.PORT5)

# Add motor groups/drivetrains here
drive_L = MotorGroup(drive_LF, drive_LB)
drive_R = MotorGroup(drive_RF, drive_RB)
drive = MotorGroup(drive_LF, drive_LB, drive_RF, drive_RB)

#---------------------------------------------------------------------------#
#                             Utility Functions                             #
#---------------------------------------------------------------------------#

def reduce_angle(min, max, angle):
    range = max - min
    while not (min <= angle < max):
        if angle < min: 
            angle += range
        if angle >= max: 
            angle -= range
    return angle

def to_rad(angle_deg):
    return angle_deg/(180/pi)

def to_deg(angle_rad):
    return angle_rad*(180/pi)

def clamp(input, max_limit, min_limit):
    input = max(min(input, max_limit), min_limit)
    return input

def is_reversed(input):
    if input < 0:
        return True
    return False

def to_volt(percent):
    return percent * 0.12

def deadband(input, width):
    if input < width:
        return 0
    return input

#---------------------------------------------------------------------------#
#                          Pre-Autonomous Functions                         #
#                                                                           #
#  You may want to perform some actions before the competition starts.      #
#  Do them in the following function.  You must return from this function   #
#  or the autonomous and usercontrol tasks will not be started.  This       #
#  function is only called once after the V5 has been powered on and        #
#  not every time that the robot is disabled.                               #
#---------------------------------------------------------------------------#

def pre_auton():
    # All activities that occur before the competition starts
    # Example: clearing encoders, setting servo positions, ...
    if inertial.installed():
        inertial.reset_heading()
        inertial.reset_rotation()

#---------------------------------------------------------------------------#
#                               PID Functions                               #
#---------------------------------------------------------------------------#
        
def set_constants(maxVoltage, kP, kI, kD, timeout_msec):
    global dt, kp, ki, kd, max_voltage, output, prev_error, timeout
    global tolerance

    max_voltage = maxVoltage
    kp = kP
    ki = kI
    kd = kD
    timeout = timeout_msec 
    dt = 20
    tolerance = 0.5
        
def compute(target, sensor):
    global error, integral, derivative, dt, kp, ki, kd, max_voltage, output
    global prev_error, prev_deriv, acceleration, tolerance, timeout

    error = target - sensor
    
    integral += error * dt
    integral = clamp(integral, 5, -5)

    derivative = (error - prev_error) / dt
    if abs(derivative) > abs(acceleration + prev_deriv) and prev_deriv != 0:
            derivative = acceleration + prev_deriv 

    output = kp * error + ki * integral + kd * derivative
    output = clamp(output, 12, -12)

    acceleration = derivative - prev_deriv
    prev_deriv = derivative
    prev_error = error
    
    return output

# ---------------------------------------------------------------------------#
#                                                                            #
#                               Autonomous Task                              #
#                                                                            #
#   This task is used to control your robot during the autonomous phase of   #
#   a VEX Competition.                                                       #
#                                                                            #
#   You must modify the code to add your own robot specific commands here.   #
# ---------------------------------------------------------------------------#

def auto_offense_side():
    pass

def auto_defense_side():
    pass

def auto_skills():
    pass

#---------------------------------------------------------------------------#
#                        Autonomous Selector Functions                      #
#---------------------------------------------------------------------------#

autons = ["Skills", "Offense Side", "Defense Side", "Test", "Calibration"]

debounce = True
def debounce_true():
    global debounce
    wait(500,MSEC)
    debounce = True

def autonomous_menu():
    global debounce
    position = 0
    while 1:
        controller.screen.clear_screen()
        controller.screen.set_cursor(1, 1)
        controller.screen.print(f"Selecting: {str(position + 1)}. {autons[position]}")
        controller.screen.next_row()
        controller.screen.print("A to select auton")
        
        if controller.buttonRight.pressing() and debounce:
            debounce = False
            if position + 1 != len(autons):
                position += 1
        Thread(debounce_true)
        
        if controller.buttonLeft.pressing() and debounce:
            debounce = False
            if position != 0:
                position -= 1
        Thread(debounce_true)
        
        if controller.buttonA.pressing():
            return autons[position]
        
        wait(100, MSEC)

def select_autonomous():
    global at_competition

    driver_control = False
    controller.screen.clear_screen()
    controller.screen.set_cursor(1, 1)
    controller.screen.print("Are you at a comp?")
    controller.screen.next_row()
    controller.screen.print("A = Yes")
    controller.screen.next_row()
    controller.screen.print("B = No, Y = Driver Control")

    while 1:
        if controller.buttonA.pressing():
            at_competition = True
            break

        if controller.buttonB.pressing():
            at_competition = False
            break

        if controller.buttonY.pressing():
            driver_control = True
            break

    if driver_control:
        user_control()

    wait(500, MSEC)
    auton = autonomous_menu()

    # controller.screen.clear_screen()
    # controller.screen.set_cursor(1,1)
    # controller.screen.print("Run \"" + str(auton) + "\"?")
    # controller.screen.next_row()
    # controller.screen.print("A = Confirm")
    # controller.screen.next_row()
    # controller.screen.print("B = Choose again")
   
    # while 1:
    #     if controller.buttonB.pressing():
    #         select_autonomous()

            
    #     if controller_1.buttonA.pressing():
            
    #         return auton
    #     wait(50,MSEC)

def autonomous():
    # ..........................................................................
    # Insert autonomous user code here.
    # ..........................................................................
    auton = 1
    if auton == 0:
        auto_offense_side()
    elif auton == 1:
        auto_defense_side()
    elif auton == 2:
        auto_skills()
    else:
        pass # Does not run auton

#---------------------------------------------------------------------------#
#                             Drive Functions                               #
#---------------------------------------------------------------------------#

def drive_with_voltage(left_voltage, right_voltage):
    drive_L.spin(FORWARD, left_voltage, VOLT)
    drive_R.spin(FORWARD, right_voltage, VOLT)

def drive_distance(distance):
    global error, integral, derivative, dt, kp, ki, kd, max_voltage, output
    global prev_error, prev_deriv, acceleration, tolerance, timeout
    
    drive.reset_position()
    position = (drive_L.position() + drive_R.position()) / 2
    error = distance - position
    integral, acceleration, prev_deriv, time_spent_running = 0, 0, 0, 0
    prev_error = error

    # Tuning parameters: maxVoltage, kP, kI, kD, timeout (msec)
    set_constants(12, 0, 0, 0, 5000)
    
    while True:
        drive_output = compute(distance, position)
        drive_with_voltage(drive_output, drive_output)
        if abs(error) < tolerance:
            integral = 0
            output = 0
            break
        wait(dt, MSEC)
        time_spent_running += dt
        if time_spent_running > timeout:
            break
    drive.stop()

def turn_to_angle(angle):
    global error, integral, derivative, dt, kp, ki, kd, max_voltage, output
    global prev_error, prev_deriv, acceleration, tolerance, timeout

    rotation = inertial.rotation()
    error = angle - rotation
    integral, acceleration, prev_deriv, time_spent_running = 0, 0, 0, 0
    prev_error = error

    # Tuning parameters: maxVoltage, kP, kI, kD, timeout (msec)
    set_constants(12, 0, 0, 0, 3000)

    while True:
        turn_output = compute(angle, rotation)
        drive_with_voltage(turn_output, -turn_output)
        if abs(error) < tolerance:
            integral = 0
            output = 0
            break
        wait(dt, MSEC)
        time_spent_running += dt
        if time_spent_running > timeout:
            break
    drive.stop()

def left_swing_to_angle(angle):

    global error, integral, derivative, dt, kp, ki, kd, max_voltage, output
    global prev_error, prev_deriv, acceleration, tolerance, timeout

    rotation = reduce_angle(-180, 180, inertial.rotation())
    error = angle - rotation
    integral, acceleration, prev_deriv, time_spent_running = 0, 0, 0, 0
    prev_error = error

    # Tuning parameters: maxVoltage, kP, kI, kD, timeout (msec)
    set_constants(12, 0, 0, 0, 3000)

    while True:
        swing_turn_output = compute(angle, rotation)
        # Only the left side of the drive turns, so this is a "left swing".
        drive_with_voltage(swing_turn_output, 0)

        if abs(error) < tolerance:
            integral = 0
            output = 0
            break
        wait(dt, MSEC)
        time_spent_running += dt
        if time_spent_running > timeout:
            break
    drive.stop()

def right_swing_to_angle(angle):

    global error, integral, derivative, dt, kp, ki, kd, max_voltage, output
    global prev_error, prev_deriv, acceleration, tolerance, timeout

    rotation = reduce_angle(-180, 180, inertial.rotation())
    error = angle - rotation
    integral, acceleration, prev_deriv, time_spent_running = 0, 0, 0, 0
    prev_error = error

    # Tuning parameters: maxVoltage, kP, kI, kD, timeout (msec)
    set_constants(12, 0, 0, 0, 3000)

    while True:
        swing_turn_output = compute(angle, rotation)
        # Only the right side of the drive turns, so this is a "right swing".
        drive_with_voltage(0, swing_turn_output) 
        
        if abs(error) < tolerance:
            integral = 0
            output = 0
            break
        wait(dt, MSEC)
        time_spent_running += dt
        if time_spent_running > timeout:
            break
    drive.stop()

# COMING FUNCTIONS
# drive_to_point()
# turn_to_point()
# turn_along_arc()

def control_arcade():
    throttle = deadband(controller.axis3.value(), 5)
    turn = deadband(controller.axis1.value(), 5)
    drive_L.spin(FORWARD, to_volt(throttle + turn), VOLT)
    drive_R.spin(FORWARD, to_volt(throttle - turn), VOLT)

def control_tank():
    left_throttle = deadband(controller.axis3.value(), 5)
    right_throttle = deadband(controller.axis2.value(), 5)
    drive_L.spin(FORWARD, to_volt(left_throttle), VOLT)
    drive_R.spin(FORWARD, to_volt(right_throttle), VOLT)

def control_holonomic():
    throttle = deadband(controller.axis3.value(), 5)
    turn = deadband(controller.axis1.value(), 5)
    strafe = deadband(controller.axis4.value(), 5)
    drive_LF.spin(FORWARD, to_volt(throttle + turn + strafe), VOLT)
    drive_LB.spin(FORWARD, to_volt(throttle + turn - strafe), VOLT)
    drive_RF.spin(FORWARD, to_volt(throttle - turn - strafe), VOLT)
    drive_RB.spin(FORWARD, to_volt(throttle - turn + strafe), VOLT)

#---------------------------------------------------------------------------#
#                                                                           #
#                              User Control Task                            #
#                                                                           #
#  This task is used to control your robot during the user control phase of #
#  a VEX Competition.                                                       #
#                                                                           #
#  You must modify the code to add your own robot specific commands here.   #
#---------------------------------------------------------------------------#

def user_control():
    # User control code here, inside the loop
    while 1:
        # This is the main execution loop for the user control program.
        # Each time through the loop your program should update motor + servo
        # values based on feedback from the joysticks.

        # ........................................................................
        # Insert user code here. This is where you use the joystick values to
        # update your motors, etc.
        # ........................................................................

        # Replace this line with control_tank() for tank drive or
        # control_holonomic() for holo drive.
        control_arcade()

        wait(20, MSEC) # Sleep the task for a short amount of time to
                       # prevent wasted resources.

#---------------------------------------------------------------------------#
#                               Button Macros                               #
#---------------------------------------------------------------------------#

def macro_buttonL1():
    # Insert code here and remove the 'pass' keyword
    pass
    
def macro_buttonL2():
    # Insert code here and remove the 'pass' keyword
    pass

def macro_buttonR1():
    # Insert code here and remove the 'pass' keyword
    pass
    
def macro_buttonR2():
    # Insert code here and remove the 'pass' keyword
    pass

# Set up callbacks for autonomous and driver control periods.
controller.buttonL1.pressed(macro_buttonL1)
controller.buttonL2.pressed(macro_buttonL2)

controller.buttonR1.pressed(macro_buttonR1)
controller.buttonR2.pressed(macro_buttonR2)

# A global instance of competition
competition = Competition(user_control, autonomous)

# Run the pre-autonomous function.
pre_auton()
