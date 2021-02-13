package frc.robot.subsystem.telemetry.commands;

import java.util.logging.Logger;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystem.telemetry.Telemetry;
import frc.robot.subsystem.SubsystemFactory;

import edu.wpi.first.wpilibj.geometry.Translation2d;

public class RotateTowardsBall extends CommandBase {
    private Telemetry telemetry;
    private boolean stop;
    private static Logger logger = Logger.getLogger(SquareSelf.class.getName());

    private int direction = 0;

     public RotateTowardsBall(Telemetry sqs) {
    // Use addRequirements() here to declare subsystem dependencies.
        telemetry = sqs;
        addRequirements(sqs);
        logger.info("creates RotateTowardsBall");
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    logger.info("starts RotateTowardsBall");
  stop = false;

  //stop = true; why is there stop = true?
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    direction = telemetry.getBallDirection();
    logger.info("" + direction);
    SubsystemFactory.getInstance().getDriveTrain().drive(new Translation2d(0, 0), telemetry.getRotationalSpeed() * direction, true);
    logger.info("rotating");
    if(telemetry.getBallDirection() == 0)
      stop = true;
      logger.info("checking if centered with ball");
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    stop = true;
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    logger.info("checking if centered with ball");
    if(telemetry.getBallDirection() == 0)
      return stop;
    else{
      return false;
    }
  }
}
