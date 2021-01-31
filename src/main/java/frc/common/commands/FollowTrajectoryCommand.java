package frc.common.commands;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Command;
import frc.robot.subsystem.SubsystemFactory;
import frc.robot.subsystem.swerve.DrivetrainSubsystem2910;
import frc.common.control.Trajectory;
import frc.common.math.Vector2;
import frc.common.util.HolonomicDriveSignal;
import frc.common.math.RigidTransform2;
import frc.common.math.Rotation2;
import frc.robot.subsystem.telemetry.Pigeon;
import frc.common.drivers.NavX.Axis;

import java.util.function.Supplier;
import java.util.Optional;

import java.time.Instant;
import java.time.Duration;

public class FollowTrajectoryCommand extends Command {
    private final Supplier<Trajectory> trajectorySupplier;
    private DrivetrainSubsystem2910 driveTrain;
    private Pigeon pigeon;

    private Trajectory trajectory;

    private double previousUpdate;

    private Instant initTime;
    private Instant lastExecute;

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
        
        initTime = Instant.now();
        lastExecute = Instant.now();
    }

    @Override
    protected void execute() {
        Instant currentTime = Instant.now();
        RigidTransform2 pos = new RigidTransform2(driveTrain.getKinematicPosition(),Rotation2.fromDegrees(pigeon.getAxis(Axis.YAW)));
        Optional<HolonomicDriveSignal> sig = driveTrain.getFollower().update(pos, driveTrain.getKinematicVelocity(), pigeon.getAngularVelocity(), Duration.between(initTime, currentTime).toMillis(), Duration.between(lastExecute, currentTime).toMillis());
        if(sig.isPresent()) {
            driveTrain.holonomicDrive(sig.get());
        }
        lastExecute = Instant.now();
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
