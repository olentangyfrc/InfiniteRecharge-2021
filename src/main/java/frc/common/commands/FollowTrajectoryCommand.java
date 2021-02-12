package frc.common.commands;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Command;
import frc.robot.subsystem.SubsystemFactory;
import frc.robot.subsystem.swerve.DrivetrainSubsystem2910;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import frc.common.math.Vector2;
import frc.common.util.HolonomicDriveSignal;
import frc.common.math.RigidTransform2;
import frc.common.math.Rotation2;
import frc.robot.subsystem.telemetry.Pigeon;
import frc.common.drivers.NavX.Axis;

import java.util.function.Supplier;
import java.util.Optional;

import java.util.logging.Level;
import java.util.logging.Logger;

public class FollowTrajectoryCommand extends Command {
    private final Supplier<Trajectory> trajectorySupplier;
    private DrivetrainSubsystem2910 driveTrain;
    private Pigeon pigeon;

    private Trajectory trajectory;

    private double previousUpdate;

    static Logger logger = Logger.getLogger(FollowTrajectoryCommand.class.getName());

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
        driveTrain = DrivetrainSubsystem2910.getInstance();
        pigeon = SubsystemFactory.getInstance().getGyro();
        driveTrain.resetKinematics(Vector2.ZERO, Timer.getFPGATimestamp());
        driveTrain.getFollower().follow(trajectory);
    }

    @Override
    protected void execute() {
    }

    @Override
    protected void end() {
        DrivetrainSubsystem2910.getInstance().setSnapRotation(trajectory.sample(trajectory.getTotalTimeSeconds()).poseMeters.getRotation().getDegrees());
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
