package frc.robot.subsystem.telemetry.commands;

import java.util.logging.Logger;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystem.telemetry.Telemetry;
import frc.robot.subsystem.SubsystemFactory;

import edu.wpi.first.wpilibj.geometry.Translation2d;

public class GoToVerticalDistance extends CommandBase {
    
  /**
   * Creates a new SquareSelf.
   */
  
  private Telemetry telemetry;
  private boolean stop;
  private double lidarTolerance;
  private static Logger logger = Logger.getLogger(GoToHorizontalDistance.class.getName());

  private int directionGoToVerticalDistance = 0;

  public GoToVerticalDistance(Telemetry sqs, double td) {
    // Use addRequirements() here to declare subsystem dependencies.
    telemetry = sqs;
    lidarTolerance = td;
    addRequirements(sqs);
    logger.info("creates goToVerticalDistance");
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    logger.info("starts goToVerticalDistance");
  stop = false;

  //stop = true; why is there stop = true?
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    directionGoToVerticalDistance = telemetry.verticalDirectionToGo();
    SubsystemFactory.getInstance().getDriveTrain().drive(new Translation2d(telemetry.getTranslationalSpeed() * directionGoToVerticalDistance, 0), 0, true);
    logger.info("moving vertical");
    if(telemetry.verticalDirectionToGo() == 0)
      stop = true;
      logger.info("checking if there yet");
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    stop = true;
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    logger.info("checking if there yet");
    if(telemetry.verticalDirectionToGo() == 0)
      return stop;
    else{
      return false;
    }
  }
}