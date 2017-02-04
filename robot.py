#!/usr/bin/env python3

#library includes
import wpilib
from wpilib import RobotDrive
#from networktables import NetworkTable

#User Includes
from Utilities import UtilityFunctions
#from AutonomousStates import AutoStates


#This robot uses Mechanum Drive
class MyRobot(wpilib.IterativeRobot):

    def autonomousInit(self):
        '''Called only at the beginning of autonomous mode'''
        pass
    
    def robotInit(self):
        """
        This function is called upon program startup and
        should be used for any initialization code.
        """
        #TODO: Update networktables implementation
        #self.sd = NetworkTable.getTable('SmartDashboard')
        
        # Joysticks
        self.stick1 = wpilib.Joystick(1)#left
        self.stick2 = wpilib.Joystick(2)#right
        self.game_pad = wpilib.Joystick(3)
        
        # Drive Train Motors
        self.leftFrontMotor = wpilib.VictorSP(0) #front left 
        self.leftBackMotor = wpilib.VictorSP(1) #back left 
        self.rightFrontMotor = wpilib.VictorSP(2) #front right 
        self.rightBackMotor = wpilib.VictorSP(3) #back right
        
        # Shooter Motors
        self.shooterMotorOne = wpilib.VictorSP(4)
        self.shooterMotorTwo = wpilib.VictorSP(5)
        
        # Climber Motor(s)
        self.climberMotor = wpilib.VictorSP(6)
        
        # Hopper Agitator motor(s)
        self.hopperAgitatorMotor = wpilib.VIctorSP(7)
        
        # Conveyor Motor(s)
        self.conveyorMotor = wpilib.VictorSP(8)
        
        # Gear Box Servos. Connected on the extension board
        self.portServo = wpilib.Servo(10) #Looking from inside the robot towards the front, this is the left servo
        self.starboardServo = wpilib.Servo(11) #Looking from inside the robot towards the front, this is the right servo
        
        
        #robot drive
        self.robotDrive = wpilib.RobotDrive(self.leftFrontMotor,
                                            self.leftBackMotor,
                                            self.rightFrontMotor,
                                            self.rightBackMotor)
                                            
        
        
        ## invert the front motors
        #self.robotDrive.setInvertedMotor(RobotDrive.MotorType.kFrontLeft, True)
        #self.robotDrive.setInvertedMotor(RobotDrive.MotorType.kFrontRight, True)
        #
        ## you may need to change or remove this to match your robot
        #self.robotDrive.setInvertedMotor(RobotDrive.MotorType.kRearLeft, True)
        #self.robotDrive.setInvertedMotor(RobotDrive.MotorType.kRearRight, True) 
        
        # Sensors
        
        
        # initialize the gyro (ANALOG INPUT)
        self.gyro = wpilib.AnalogGyro(0)
        
        #Timer
        self.timer = wpilib.Timer()
        self.timer.start()
        self.startTime = 0
            
        #autonomous
        self.direction = "straight"
        self.obstacle_type = "moat"
        self.autonomous_state = "begin"
        self.initialTime = 0
        self.initialHeading = 0
        self.autoSafeToGetHeading = True
        self.autoSafeToResetEncoder = True
        self.autoSafeToGetTime = True
        
        self.acceptable_heading_error = 5 # the range (+- degrees) that the heading can be off of desired that we still consider good
        self.slower_speed_band = 20  # When +-20 degrees from desired heading, turn slower to avoid overshooting
        
        self.autoSlowTurnSpeed = 0.4 #slow speed
        self.autoNormalTurnSpeed = 0.8 #normal speed
        
        #AutoStates variables
        self.choose_direction_state = "begin"
        self.handle_obstacle_state = "begin"
        self.shooting_state = "begin"
        
        #Utilities variables
        self.goalSafeToGetTime = True
        self.goalInitialTimeStamp = 0   #use to store an initial timestamp
        self.imageSearchRate = 5 #how many times to query for an updated network table a second
        self.noGoalFoundCount = 0 #How many times we have been searched without having a goal
        self.lastCOG_X = 0
        self.lastCOG_Y = 0
        
        
        self.ERROR = 0
        self.GO_LEFT = 1
        self.GO_RIGHT = 2
        self.ON_TARGET = 3
        
        
    def autonomousPeriodic(self):
        try:
            """This function is called periodically during autonomous."""
            self.robotDrive.setSafetyEnabled(False) #IMPORTANT! DO NOT REMOVE!
            
        except:
            pass
            
    def disabledInit(self):
        '''Calledonly ar the beginning of disabled mode'''
        pass
       
    def disabledPeriodic(self):
        '''Called every 20ms in disabled mode'''
        pass

    def teleopInit(self):
        '''Called only at the beginning of teleoperated mode'''
        pass

    def teleopPeriodic(self):
        try:
            """This function is called periodically during operator control."""
            self.robotDrive.setSafetyEnabled(True) #IMPORTANT! DO NOT REMOVE!
            
            stick1_X = self.stick1.getX()#left
            stick2_Y = self.stick2.getY()#right
            stick2_X = self.stick2.getX()#right
            
            """ dead-band """
            if stick1_X > -0.05 and stick1_X < 0.05:
                stick1_X = 0
            if stick2_Y > -0.05 and stick2_Y < 0.05:
                stick2_Y = 0
            if stick2_X > -0.05 and stick2_X < 0.05:
                stick2_X = 0
           
        
            # Use the joystick X axis for lateral movement, Y axis for forward movement, and stick 2 X axis for rotation.
            # This sample does not use field-oriented drive, so the gyro input is set to zero.
            self.robotDrive.mecanumDrive_Cartesian(stick1_X,
                                                   stick1_Y,
                                                   stick2_X, 0);
               
            # Other Controls Below
            
            # Shooter Controls:
            
            #Spin up shooter
            if self.game_pad.gerRawButton(7) or self.stick1.getRawButton(1):
                self.shooterMotorOne.set(0.8)
                self.shooterMotorTwo.set(0.8)
            else:
                self.shooterMotorOne.set(0)
                self.shooterMotorTwo.set(0)
            
            # Conveyor Controls:
            if self.game_pad.getRawButton(6) or self.stick1.getRawButton(3):
                self.conveyorMotor.set(1)
            else:
                self.conveyorMotor.set(0)
                
            if self.game_pad.getRawButton(3) or self.stick1.getRawButton(2):
                self.conveyorMotor.set(-0.6)
            else:
                self.conveyorMotor.set(0)
            
            # Climbing Controls
            if self.game_pad.getRawButton(4) or self.stick2.getRawButton(3):
                self.climberMotor.set(0.7)
            else:
                self.climberMotor.set(0)
                
            if self.game_pad.getRawButton(1) or self.stick2.getRawButton(2):
                self.climberMotor.set(-0.5)
            else:
                self.climberMotor.set(0)
         
            
            #if self.stick2.getRawButton(2):
            #    self.defenceArmRotate.set(-.5)
            #
            #elif self.stick2.getRawButton(3):
            #    self.defenceArmRotate.set(1)
            #else:
            #    self.defenceArmRotate.set(0)
        except:
            pass
           

if __name__ == '__main__':
    wpilib.run(MyRobot)