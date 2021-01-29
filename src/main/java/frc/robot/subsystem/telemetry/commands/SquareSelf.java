/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystem.telemetry.commands;

import java.util.logging.Logger;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystem.telemetry.Telemetry;

public class SquareSelf extends CommandBase {
  /**
   * Creates a new SquareSelf.
   */
  private Telemetry telemetry;
  private boolean stop;
  private double targetDistance;
  private static Logger logger = Logger.getLogger(SquareSelf.class.getName());

  private int direction = 0;

  private double rotSpeed = 0.3;

  public SquareSelf(Telemetry sqs, double td) {
    // Use addRequirements() here to declare subsystem dependencies.
    telemetry = sqs;
    targetDistance = td;
    addRequirements(sqs);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  stop = false;

  //stop = true; why is there stop = true?
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    direction = telemetry.whereAmI();
    SubsystemFactory.getInstance().getDriveTrain().drive(new Translation2d(0, 0), speed * direction, true);
    if(telemetry.whereAmI() == 0)
      stop = true;
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    stop = true;
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(telemetry.whereAmI() == 0)
      return stop;
  }
}
