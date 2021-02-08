package frc.robot.subsystem.swerve.commands;

import frc.robot.OI;
import frc.robot.subsystem.SubsystemFactory;
import frc.robot.subsystem.swerve.DrivetrainSubsystem;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import java.util.logging.Logger;
import java.util.logging.Level;
import frc.common.drivers.NavX.Axis;

public class DriveCommand extends CommandBase {

    private DrivetrainSubsystem driveTrain;
    private static Logger logger = Logger.getLogger(DriveCommand.class.getName());

    public DriveCommand(DrivetrainSubsystem dt) {
        driveTrain = dt;
        addRequirements(driveTrain);
    }

    @Override
    public void execute() {
        double forward = - OI.getInstance().getLeftJoystickYValue();
        // Square the forward stick
        forward = Math.copySign(Math.pow(forward, 2.0), forward);

        double strafe = - OI.getInstance().getLeftJoystickXValue();
        // Square the strafe stick
        strafe = Math.copySign(Math.pow(strafe, 2.0), strafe);

        double rotation = - OI.getInstance().getRightJoystickXValue();
        // Square the rotation stick
        rotation = Math.copySign(Math.pow(rotation, 2.0), rotation);
        String output = String.format("Forward[%f], Strafe[%f], Rotation[%f], Gyro[%f]", forward, strafe, rotation, SubsystemFactory.getInstance().getGyro().getAxis(Axis.YAW));
        logger.log(Level.INFO, output);
        driveTrain.drive(new Translation2d(forward, strafe), rotation, true);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
