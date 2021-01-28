package frc.common.commands;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Command;
import frc.robot.subsystem.swerve.DrivetrainSubsystem2910;
import frc.common.control.Trajectory;
import frc.common.math.Vector2;

import java.util.function.Supplier;

public class FollowTrajectoryCommand extends Command {
    private final Supplier<Trajectory> trajectorySupplier;

    private Trajectory trajectory;

    public FollowTrajectoryCommand(Trajectory trajectory) {
        this(() -> trajectory);
    }

    public FollowTrajectoryCommand(Supplier<Trajectory> trajectorySupplier) {
        this.trajectorySupplier = trajectorySupplier;

        requires(DrivetrainSubsystem2910.getInstance());
        this.setRunWhenDisabled(true);
    }

    @Override
    protected void initialize() {
        trajectory = trajectorySupplier.get();
        DrivetrainSubsystem2910.getInstance().resetKinematics(Vector2.ZERO, Timer.getFPGATimestamp());
        DrivetrainSubsystem2910.getInstance().getFollower().follow(trajectory);
    }

    @Override
    protected void end() {
        DrivetrainSubsystem2910.getInstance().setSnapRotation(trajectory.calculateSegment(trajectory.getDuration()).rotation.toRadians());
    }

    @Override
    protected void interrupted() {
        end();
        DrivetrainSubsystem2910.getInstance().getFollower().cancel();
    }

    @Override
    protected boolean isFinished() {
        // Only finish when the trajectory is completed
        return DrivetrainSubsystem2910.getInstance().getFollower().getCurrentTrajectory().isEmpty();
    }
}
