package frc.common.commands;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.subsystem.SubsystemFactory;
import frc.robot.subsystem.swerve.DrivetrainSubsystem2910;
import frc.common.math.Rotation2;
import frc.common.math.Vector2;
import frc.robot.OI;
import java.util.logging.Level;
import java.util.logging.Logger;
import frc.common.drivers.NavX.Axis;

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
        //String output = String.format("Forward[%f], Strafe[%f], Rotation[%f], Gyro[%f]", forward, strafe, rotation, DrivetrainSubsystem2910.getInstance().getGyroscope().getAxis(Axis.YAW));
        //logger.log(Level.INFO, output);
        DrivetrainSubsystem2910.getInstance().holonomicDrive(translation, rotation, false);
        String output = String.format("Translation: (%f,%f), Rotation: %f, FieldOriented: %b", translation.x, translation.y, rotation, false);
        logger.log(Level.INFO, output);
    }

    @Override
    protected boolean isFinished() {
        return false;
    }
}
