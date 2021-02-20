/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import java.io.PrintWriter;
import java.io.StringWriter;
import java.util.ArrayList;
import java.util.logging.Level;
import java.util.logging.Logger;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.drive.MecanumDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.SerialPort;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystem.DisplayManager;
import frc.robot.subsystem.PortMan;
import frc.robot.subsystem.SubsystemFactory;
import frc.robot.subsystem.controlpanel.ControlPanel;
import frc.robot.util.OzoneLogger;

//2910 auton stuff
import frc.common.auton.AutonomousSelector;
import frc.common.auton.AutonomousTrajectories;
import frc.common.commands.FollowTrajectoryCommand;
import frc.robot.subsystem.swerve.DrivetrainSubsystem2910;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.Scheduler;


import frc.robot.subsystem.SBInterface;
import frc.robot.subsystem.controlpanel.ControlPanelSBTab;

import java.time.Instant;
import java.time.Duration;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {

  AHRS ahrs;
  MecanumDrive myRobot;
  Joystick stick;

  public Robot() {
    //stick = new Joystick(0);
    try {
  /***********************************************************************
   * navX-MXP:
   * - Communication via RoboRIO MXP (SPI, I2C, TTL UART) and USB.            
   * - See http://navx-mxp.kauailabs.com/guidance/selecting-an-interface.
   * 
   * navX-Micro:
   * - Communication via I2C (RoboRIO MXP or Onboard) and USB.
   * - See http://navx-micro.kauailabs.com/guidance/selecting-an-interface.
   * 
   * Multiple navX-model devices on a single robot are supported.
   ************************************************************************/
        ahrs = new AHRS(SerialPort.Port.kUSB1);
        //ahrs = new AHRS(SerialPort.Port.kMXP, SerialDataType.kProcessedData, (byte)50);
        ahrs.enableLogging(true);
    } catch (RuntimeException ex ) {
        DriverStation.reportError("Error instantiating navX MXP:  " + ex.getMessage(), true);
    }
    Timer.delay(1.0);
  //UsbCamera cam = CameraServer.getInstance().startAutomaticCapture();
  //cam.setResolution(640, 480);        
}

  static Logger logger = Logger.getLogger(Robot.class.getName());
  ControlPanel controlPanel;
  private static SubsystemFactory subsystemFactory;

  private Instant initTime;
  private Instant currentTime;

  private DisplayManager dManager;
  private ShuffleboardTab tab;

  private AutonomousTrajectories autonomousTrajectories = new AutonomousTrajectories(DrivetrainSubsystem2910.CONSTRAINTS);
  private AutonomousSelector autonomousSelector = new AutonomousSelector(autonomousTrajectories);

  private Command autonomousCommand = null;


  // Channels for the wheels
  final static int frontLeftChannel = 2;
  final static int rearLeftChannel = 3;
  final static int frontRightChannel = 1;
  final static int rearRightChannel = 0;

  Spark frontLeft;
  Spark rearLeft;
  Spark frontRight;
  Spark rearRight;


  /**
   * This function is run when the robot is first started up and should be
   * used for any initialization code.
   */
  @Override
  public void robotInit() {
    initTime = Instant.now();

    subsystemFactory = SubsystemFactory.getInstance();

    tab = Shuffleboard.getTab("Auton");

    OzoneLogger.getInstance().init(Level.ALL);
    logger.log(Level.INFO, "robot init");

    dManager = new DisplayManager();

    try {
      subsystemFactory.init(dManager, PortMan.getInstance());

    } catch (Exception e) {
      StringWriter writer = new StringWriter();
      PrintWriter pw  = new PrintWriter(writer);
      e.printStackTrace(pw);
      logger.severe(writer.toString());
    }

    frontLeft = new Spark(frontLeftChannel);
    rearLeft = new Spark(rearLeftChannel);
    frontRight = new Spark(frontRightChannel);
    rearRight = new Spark(rearRightChannel);
    myRobot = new MecanumDrive(frontLeft, rearLeft, frontRight, rearRight);
    myRobot.setExpiration(0.1);
    stick = new Joystick(0);
    try {
      /***********************************************************************
       * navX-MXP: - Communication via RoboRIO MXP (SPI, I2C) and USB. - See
       * http://navx-mxp.kauailabs.com/guidance/selecting-an-interface.
       * 
       * navX-Micro: - Communication via I2C (RoboRIO MXP or Onboard) and USB. - See
       * http://navx-micro.kauailabs.com/guidance/selecting-an-interface.
       * 
       * VMX-pi: - Communication via USB. - See
       * https://vmx-pi.kauailabs.com/installation/roborio-installation/
       * 
       * Multiple navX-model devices on a single robot are supported.
       ************************************************************************/
      ahrs = new AHRS(SPI.Port.kMXP);
    } catch (RuntimeException ex) {
      DriverStation.reportError("Error instantiating navX MXP:  " + ex.getMessage(), true);
    }

  }

  /**
   * This function is called every robot packet, no matter the mode. Use
   * this for items like diagnostics that you want ran during disabled,
   * autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before
   * LiveWindow and SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
      CommandScheduler.getInstance().run();
      Scheduler.getInstance().run();
      dManager.update();
      // need to double check if default Drive command is being called too.
      // this looks realy weird.
      currentTime = Instant.now();
      SubsystemFactory.getInstance().getDriveTrain().updateKinematics(Duration.between(initTime, currentTime).toMillis());
       
  }

  /**
   * This autonomous (along with the chooser code above) shows how to select
   * between different autonomous modes using the dashboard. The sendable
   * chooser code works with the Java SmartDashboard. If you prefer the
   * LabVIEW Dashboard, remove all of the chooser code and uncomment the
   * getString line to get the auto name from the text box below the Gyro
   *
   * <p>You can add additional auto modes by adding additional comparisons to
   * the switch structure below with additional strings. If using the
   * SendableChooser make sure to add them to the chooser code above as well.
   */
  @Override
  public void autonomousInit() {
    if (autonomousCommand != null) {
      autonomousCommand.cancel();
    }

    autonomousCommand = autonomousSelector.getCommand();
    autonomousCommand.start();
  }

  /**
   * This function is called periodically during autonomous.
   */
  @Override
  public void autonomousPeriodic() {
    CommandScheduler.getInstance().run();
    Scheduler.getInstance().run();

    boolean motionDetected = ahrs.isMoving();
    SmartDashboard.putBoolean("MotionDetected", motionDetected);

    try {
      myRobot.driveCartesian(stick.getX(), stick.getY(), stick.getTwist(), 0);
    } catch (RuntimeException ex) {
      String err_string = "Drive system error:  " + ex.getMessage();
      DriverStation.reportError(err_string, true);
    }

    //test.execute();
  }

  /**
   * This function is called periodically during operator control.
   */
  @Override
  public void teleopPeriodic() {
  }

   /**
     * Display navX MXP Sensor Data on Smart Dashboard
     */
    public void operatorControl() {
      while (isOperatorControl() && isEnabled()) {
          
          Timer.delay(0.020);		/* wait for one motor update time period (50Hz)     */
          
          boolean zero_yaw_pressed = false; //stick.getTrigger();
          if ( zero_yaw_pressed ) {
              ahrs.zeroYaw();
          }

          /* Display 6-axis Processed Angle Data                                      */
          SmartDashboard.putBoolean(  "IMU_Connected",        ahrs.isConnected());
          SmartDashboard.putBoolean(  "IMU_IsCalibrating",    ahrs.isCalibrating());
          SmartDashboard.putNumber(   "IMU_Yaw",              ahrs.getYaw());
          SmartDashboard.putNumber(   "IMU_Pitch",            ahrs.getPitch());
          SmartDashboard.putNumber(   "IMU_Roll",             ahrs.getRoll());
          
          /* Display tilt-corrected, Magnetometer-based heading (requires             */
          /* magnetometer calibration to be useful)                                   */
          
          SmartDashboard.putNumber(   "IMU_CompassHeading",   ahrs.getCompassHeading());
          
          /* Display 9-axis Heading (requires magnetometer calibration to be useful)  */
          SmartDashboard.putNumber(   "IMU_FusedHeading",     ahrs.getFusedHeading());

          /* These functions are compatible w/the WPI Gyro Class, providing a simple  */
          /* path for upgrading from the Kit-of-Parts gyro to the navx MXP            */
          
          SmartDashboard.putNumber(   "IMU_TotalYaw",         ahrs.getAngle());
          SmartDashboard.putNumber(   "IMU_YawRateDPS",       ahrs.getRate());

          /* Display Processed Acceleration Data (Linear Acceleration, Motion Detect) */
          
          SmartDashboard.putNumber(   "IMU_Accel_X",          ahrs.getWorldLinearAccelX());
          SmartDashboard.putNumber(   "IMU_Accel_Y",          ahrs.getWorldLinearAccelY());
          SmartDashboard.putBoolean(  "IMU_IsMoving",         ahrs.isMoving());
          SmartDashboard.putBoolean(  "IMU_IsRotating",       ahrs.isRotating());

          /* Display estimates of velocity/displacement.  Note that these values are  */
          /* not expected to be accurate enough for estimating robot position on a    */
          /* FIRST FRC Robotics Field, due to accelerometer noise and the compounding */
          /* of these errors due to single (velocity) integration and especially      */
          /* double (displacement) integration.                                       */
          
          SmartDashboard.putNumber(   "Velocity_X",           ahrs.getVelocityX());
          SmartDashboard.putNumber(   "Velocity_Y",           ahrs.getVelocityY());
          SmartDashboard.putNumber(   "Displacement_X",       ahrs.getDisplacementX());
          SmartDashboard.putNumber(   "Displacement_Y",       ahrs.getDisplacementY());
          
          /* Display Raw Gyro/Accelerometer/Magnetometer Values                       */
          /* NOTE:  These values are not normally necessary, but are made available   */
          /* for advanced users.  Before using this data, please consider whether     */
          /* the processed data (see above) will suit your needs.                     */
          
          SmartDashboard.putNumber(   "RawGyro_X",            ahrs.getRawGyroX());
          SmartDashboard.putNumber(   "RawGyro_Y",            ahrs.getRawGyroY());
          SmartDashboard.putNumber(   "RawGyro_Z",            ahrs.getRawGyroZ());
          SmartDashboard.putNumber(   "RawAccel_X",           ahrs.getRawAccelX());
          SmartDashboard.putNumber(   "RawAccel_Y",           ahrs.getRawAccelY());
          SmartDashboard.putNumber(   "RawAccel_Z",           ahrs.getRawAccelZ());
          SmartDashboard.putNumber(   "RawMag_X",             ahrs.getRawMagX());
          SmartDashboard.putNumber(   "RawMag_Y",             ahrs.getRawMagY());
          SmartDashboard.putNumber(   "RawMag_Z",             ahrs.getRawMagZ());
          SmartDashboard.putNumber(   "IMU_Temp_C",           ahrs.getTempC());
          SmartDashboard.putNumber(   "IMU_Timestamp",        ahrs.getLastSensorTimestamp());
          
          /* Omnimount Yaw Axis Information                                           */
          /* For more info, see http://navx-mxp.kauailabs.com/installation/omnimount  */
          AHRS.BoardYawAxis yaw_axis = ahrs.getBoardYawAxis();
          SmartDashboard.putString(   "YawAxisDirection",     yaw_axis.up ? "Up" : "Down" );
          SmartDashboard.putNumber(   "YawAxis",              yaw_axis.board_axis.getValue() );
          
          /* Sensor Board Information                                                 */
          SmartDashboard.putString(   "FirmwareVersion",      ahrs.getFirmwareVersion());
          
          /* Quaternion Data                                                          */
          /* Quaternions are fascinating, and are the most compact representation of  */
          /* orientation data.  All of the Yaw, Pitch and Roll Values can be derived  */
          /* from the Quaternions.  If interested in motion processing, knowledge of  */
          /* Quaternions is highly recommended.                                       */
          SmartDashboard.putNumber(   "QuaternionW",          ahrs.getQuaternionW());
          SmartDashboard.putNumber(   "QuaternionX",          ahrs.getQuaternionX());
          SmartDashboard.putNumber(   "QuaternionY",          ahrs.getQuaternionY());
          SmartDashboard.putNumber(   "QuaternionZ",          ahrs.getQuaternionZ());
          
          /* Connectivity Debugging Support                                           */
          SmartDashboard.putNumber(   "IMU_Byte_Count",       ahrs.getByteCount());
          SmartDashboard.putNumber(   "IMU_Update_Count",     ahrs.getUpdateCount());

          SmartDashboard.putNumber(   "Gyroscope Angle",      ahrs.getAngle());
      }
  }


  /**
   * This function is called periodically during test mode.
   */
  @Override
  public void testPeriodic() {
  }
}
