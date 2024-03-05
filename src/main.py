# ---------------------------------------------------------------------------- #
#                                                                              #
# 	Module:       main.py                                                      #
# 	Author:                                                                    #
# 	Created:                                                                   #
# 	Description:  V5 project                                                   #
#                                                                              #
# ---------------------------------------------------------------------------- #

# Library imports
from math import pi
from vex import *

#---------------------------------------------------------------------------#
#                            Robot Configuration                            #
#---------------------------------------------------------------------------#

# A global instance of brain used for printing to the V5 Brain screen
brain = Brain()

# VEXcode device constructors
controller = Controller(PRIMARY)

# Add motors here
drive_LF = Motor(Ports.PORT1, GearSetting.RATIO_6_1, True)
drive_LB = Motor(Ports.PORT2, GearSetting.RATIO_6_1, True)
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

def clamp(input, max, min):
    if input > max:
        return max
    if input < min: 
        return min
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

def drive_distance(distance):

    drive.reset_position()

    # Tuning parameters
    dt = 20
    kp, ki, kd = 0, 0, 0
    tolerance = 0.5
    timeout = abs(distance) / 2

    error = distance - drive.position()
    integral, acceleration, prev_deriv, time_spent_running = 0, 0, 0, 0
    prev_error = error

    while abs(error) > tolerance:

        error = distance - drive.position()

        integral += error * dt
        integral = clamp(integral, 5, -5) 
        # This statement restricts the integral between -5 and 5
        # to prevent integral windup.

        derivative = (error - prev_error) * dt
        if abs(derivative) > abs(acceleration + prev_deriv) and prev_deriv != 0:
            derivative = acceleration + prev_deriv 
            # This statement allows smooth derivative calculations.

        output = kp * error + ki * integral + kd * derivative
        output = clamp(output, 12, -12) 
        drive.spin(FORWARD, output, VOLT)

        wait(dt)
        time_spent_running += dt
        if time_spent_running > timeout:
            break

        acceleration = derivative - prev_deriv
        prev_deriv = derivative
        prev_error = error

    drive.stop()

def turn_to_angle(angle):

    drive.reset_position()

    # Tuning parameters
    dt = 20
    kp, ki, kd = 0, 0, 0
    tolerance = 0.5
    timeout = 1000

    error = angle - inertial.rotation()
    integral, acceleration, prev_deriv, time_spent_running = 0, 0, 0, 0
    prev_error = error

    while abs(error) > tolerance:

        error = angle - inertial.rotation()

        integral += error * dt
        integral = clamp(integral, 5, -5) 
        # This statement restricts the integral between -5 and 5
        # to prevent integral windup.

        derivative = (error - prev_error) * dt
        if abs(derivative) > abs(acceleration + prev_deriv) and prev_deriv != 0:
            derivative = acceleration + prev_deriv 
            # This statement allows smooth derivative calculations.

        output = kp * error + ki * integral + kd * derivative
        output = clamp(output, 12, -12) 
        drive_L.spin(FORWARD, output, VOLT)
        drive_R.spin(REVERSE, output, VOLT)

        wait(dt)
        time_spent_running += dt
        if time_spent_running > timeout:
            break

        acceleration = derivative - prev_deriv
        prev_deriv = derivative
        prev_error = error

    drive.stop()

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

# COMING FUNCTIONS
# drive_distance()
# turn_to_angle()
# left_swing_to_angle()
# right_swing_to_angle()
# drive_to_point()
# turn_to_point()
# arcturn()

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

        # Replace this line with control_tank() for tank drive 
        # or control_holonomic() for holomonic drive
        control_arcade()

        wait(20, MSEC) # Sleep the task for a short amount of time to
                       # prevent wasted resources.
        
#---------------------------------------------------------------------------#
#                               Button Macros                               #
#---------------------------------------------------------------------------#

def macro_button_L1():
    # Insert code here and remove the 'pass' keyword
    pass
    
def macro_button_L2():
    # Insert code here and remove the 'pass' keyword
    pass

# Set up callbacks for autonomous and driver control periods.
controller.buttonL1.pressed(macro_motor_5_fwd)
controller.buttonL2.pressed(macro_motor_5_rev)

# A global instance of competition
competition = Competition(user_control, autonomous)

# Run the pre-autonomous function.
pre_auton()
