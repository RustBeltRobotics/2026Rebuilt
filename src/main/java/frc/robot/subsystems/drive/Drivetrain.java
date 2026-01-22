package frc.robot.subsystems.drive;

import java.util.function.Supplier;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants;
import frc.robot.generated.TunerConstants;

public class Drivetrain extends CommandSwerveDrivetrain {

    private final SwerveRequest.FieldCentric teleopRequest = new SwerveRequest.FieldCentric()
        .withDeadband(DriveConstants.maxSpeed * ControlBoardConstants.stickDeadband)
        .withRotationalDeadband(DriveConstants.maxAngularRate * ControlBoardConstants.stickDeadband) // Add a 10% deadband
        .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors

    private final SwerveRequest.ApplyRobotSpeeds m_pathApplyRobotSpeeds = new SwerveRequest.ApplyRobotSpeeds();

    public Drivetrain() {
        super(TunerConstants.DrivetrainConstants, TunerConstants.FrontLeft, TunerConstants.FrontRight, TunerConstants.BackLeft, TunerConstants.BackRight);
    }

    public Command teleopDrive(CommandXboxController controller) {
        return applyRequest(() ->
                teleopRequest.withVelocityX(-controller.getLeftY() * DriveConstants.maxSpeed) // Drive forward with negative Y (forward)
                    .withVelocityY(-controller.getLeftX() * DriveConstants.maxSpeed) // Drive left with negative X (left)
                    .withRotationalRate(-controller.getRightX() * DriveConstants.maxAngularRate) // Drive counterclockwise with negative X (left)
            );
    }

    public Command alignDrive(CommandXboxController controller, Supplier<Pose2d> targetPoseSupplier) {
        return applyRequest(() -> {
            double controllerVelX = -controller.getLeftY();
            double controllerVelY = -controller.getLeftX();

            Pose2d drivePose = getState().Pose;
            Pose2d targetPose = targetPoseSupplier.get();
            double shooterOffset = -DriveConstants.shooterSideOffset.in(Units.Meters);
            double targetDistance = drivePose.getTranslation().getDistance(targetPose.getTranslation());
            double shooterAngleRads = Math.acos(shooterOffset / targetDistance); 
            Rotation2d shooterAngle = Rotation2d.fromRadians(shooterAngleRads);
            Rotation2d offsetAngle = Rotation2d.kCCW_90deg.minus(shooterAngle);
            Rotation2d desiredAngle = offsetAngle.plus(drivePose.relativeTo(targetPose).getTranslation().getAngle()).plus(Rotation2d.k180deg);
            desiredAngle = desiredAngle.plus(Rotation2d.k180deg);
            Rotation2d currentAngle = drivePose.getRotation();
            Rotation2d deltaAngle = currentAngle.minus(desiredAngle);
            double wrappedAngleDeg = MathUtil.inputModulus(deltaAngle.getDegrees(), -180.0, 180.0);

            if (
                (Math.abs(wrappedAngleDeg) < DriveConstants.epsilonAngleToGoal.in(Units.Degrees)) // if facing goal already
                && Math.hypot(controllerVelX, controllerVelY) < ControlBoardConstants.stickDeadband) {
                    return new SwerveRequest.SwerveDriveBrake();
                } else {
                double rotationalRate = DriveConstants.rotationController.calculate(currentAngle.getRadians(), desiredAngle.getRadians());

                return alignRequest.withVelocityX(controllerVelX * DriveConstants.maxSpeed) // Drive forward with negative Y (forward)
                    .withVelocityY(-controller.getLeftX() * DriveConstants.maxSpeed) // Drive left with negative X (left)
                    .withRotationalRate(rotationalRate * DriveConstants.maxAngularRate); // Use angular rate for rotation
            }
        });
    }

    public Distance getShotDistance(Translation2d targetPose) {
        Pose2d drivePose = getState().Pose;
        double centerToTargetMeters = drivePose.getTranslation().getDistance(targetPose);
        double centerToShooterMeters = DriveConstants.shooterSideOffset.in(Units.Meters);
        double shooterToTargetMeters = Math.sqrt(Math.pow(centerToTargetMeters, 2.0) - Math.pow(centerToShooterMeters, 2.0));
        return Units.Meters.of(shooterToTargetMeters);
    }

    public Distance getShotDistance() {
        return getShotDistance(Constants.Game.getHubPose().toPose2d().getTranslation());
    }
}
