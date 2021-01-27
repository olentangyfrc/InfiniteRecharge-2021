package frc.robot.subsystem.swerve.commands;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.subsystem.swerve.DrivetrainSubsystem2910;
import frc.common.math.Vector2;

public class DriveCommand2910 extends Command {
    private Vector2 translation;
    private double rotation;
    private boolean fieldOriented;

    public DriveCommand2910(Vector2 translation, double rotation, boolean fieldOriented) {
        this.translation = translation;
        this.rotation = rotation;
        this.fieldOriented = fieldOriented;

        requires(DrivetrainSubsystem2910.getInstance());

        this.setRunWhenDisabled(true);
    }

    @Override
    protected void initialize() {
        DrivetrainSubsystem2910.getInstance().holonomicDrive(translation, rotation, fieldOriented);
    }

    @Override
    protected void end() {
        DrivetrainSubsystem2910.getInstance().holonomicDrive(Vector2.ZERO, 0.0);
    }

    @Override
    protected boolean isFinished() {
        return false;
    }
}
