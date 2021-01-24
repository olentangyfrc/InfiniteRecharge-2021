/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystem.SubsystemFactory;

import java.util.logging.Logger;


public class Auton extends CommandBase {

  private final Logger logger = Logger.getLogger(Auton.class.getName());
  /**
   * Creates a new TestAuton.
   */
  private boolean firstTime;
  private double stopTime;
  private boolean stop;
  private double speed;
  public Auton(double s) {
    firstTime = true;
    stop = false;
    speed = s;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    firstTime = true;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(firstTime){
      stopTime = Timer.getFPGATimestamp() + 3;
      firstTime = false;
    }
    if(Timer.getFPGATimestamp() < stopTime)
      SubsystemFactory.getInstance().getDriveTrain().drive(new Translation2d(speed, 0), 0, true);
    else 
      stop = true;
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return stop;
  }
}
