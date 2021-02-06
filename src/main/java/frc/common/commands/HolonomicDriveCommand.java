package frc.common.commands;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.subsystem.swerve.DrivetrainSubsystem2910;
import frc.common.math.Rotation2;
import frc.common.math.Vector2;
import frc.robot.OI;
import java.util.logging.Level;
import java.util.logging.Logger;

public class HolonomicDriveCommand extends Command {

    static Logger logger = Logger.getLogger(HolonomicDriveCommand.class.getName());

    public HolonomicDriveCommand() {
        requires(DrivetrainSubsystem2910.getInstance());
    }
    @Override
    protected void execute() {
        double forward = - OI.getInstance().getLeftJoystickYValue();
        double strafe = - OI.getInstance().getLeftJoystickXValue();
        double rotation = - OI.getInstance().getRightJoystickXValue();

        Vector2 translation = new Vector2(forward, strafe);

        DrivetrainSubsystem2910.getInstance().holonomicDrive(translation, rotation, true);
    }

    @Override
    protected boolean isFinished() {
        return false;
    }
}
