package frc.robot.subsystems.drive;

import java.util.function.Supplier;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants;
import frc.robot.generated.TunerConstants;
import frc.robot.vision.VisionEstimateConsumer;

public class Drivetrain extends CommandSwerveDrivetrain implements VisionEstimateConsumer {

    private final SwerveRequest.FieldCentric teleopRequest = new SwerveRequest.FieldCentric()
        .withDeadband(Constants.Kinematics.MAX_VELOCITY_METERS_PER_SECOND * Constants.Controls.CONTROLLER_DEADBAND)
        .withRotationalDeadband(Constants.Kinematics.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND * Constants.Controls.CONTROLLER_DEADBAND)
        .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors

    private final SwerveRequest.FieldCentric alignToTargetDriveRequest = new SwerveRequest.FieldCentric()
        .withDeadband(Constants.Kinematics.MAX_VELOCITY_METERS_PER_SECOND * Constants.Controls.CONTROLLER_DEADBAND)
        .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors

    private final SwerveRequest.ApplyRobotSpeeds pathApplyRobotSpeeds = new SwerveRequest.ApplyRobotSpeeds();

    public Drivetrain() {
        super(TunerConstants.DrivetrainConstants, TunerConstants.FrontLeft, TunerConstants.FrontRight, TunerConstants.BackLeft, TunerConstants.BackRight);
    }

    public Command teleopDrive(CommandXboxController controller) {
        return applyRequest(() ->
                teleopRequest.withVelocityX(-controller.getLeftY() * Constants.Kinematics.MAX_VELOCITY_METERS_PER_SECOND) // Drive forward with negative Y (forward)
                    .withVelocityY(-controller.getLeftX() * Constants.Kinematics.MAX_VELOCITY_METERS_PER_SECOND) // Drive left with negative X (left)
                    .withRotationalRate(-controller.getRightX() * Constants.Kinematics.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND) // Drive counterclockwise with negative X (left)
            );
    }

    public Command alignToTargetDrive(CommandXboxController controller, Supplier<Pose2d> targetPoseSupplier) {
        return applyRequest(() -> {
            double controllerVelX = -controller.getLeftY();  //forward/back
            double controllerVelY = -controller.getLeftX();  //left/right strafe

            //change this to false if our shooter is centered on the robot
            boolean shooterIsOffsetFromCenter = true;
            Rotation2d desiredAngle;
            Pose2d drivePose = getState().Pose;
            Pose2d targetPose = targetPoseSupplier.get();
            Rotation2d currentAngle = drivePose.getRotation();

            if (shooterIsOffsetFromCenter) {
                double targetDistance = drivePose.getTranslation().getDistance(targetPose.getTranslation());
                double shooterOffset = -DriveConstants.shooterSideOffset.in(Units.Meters);  //lateral offset from center of robot to shooter (negative is left)
                double shooterAngleRads = Math.acos(shooterOffset / targetDistance); 
                Rotation2d shooterAngle = Rotation2d.fromRadians(shooterAngleRads);
                Rotation2d offsetAngle = Rotation2d.kCCW_90deg.minus(shooterAngle);
                desiredAngle = offsetAngle.plus(drivePose.relativeTo(targetPose).getTranslation().getAngle()).plus(Rotation2d.k180deg);
                desiredAngle = desiredAngle.plus(Rotation2d.k180deg);
            } else {
                desiredAngle = targetPose.getTranslation().minus(drivePose.getTranslation()).getAngle();
                //TODO: do we need to add 180 degrees here?
            }

            Rotation2d deltaAngle = currentAngle.minus(desiredAngle);
            double wrappedAngleDeg = MathUtil.inputModulus(deltaAngle.getDegrees(), -180.0, 180.0);

            boolean notTryingToDrive = Math.hypot(controllerVelX, controllerVelY) < Constants.Controls.CONTROLLER_DEADBAND;
            boolean alreadyFacingGoal = Math.abs(wrappedAngleDeg) < Constants.Kinematics.EPSILON_ANGLE_TO_GOAL.in(Units.Degrees);

            if (alreadyFacingGoal && notTryingToDrive) {
                return new SwerveRequest.SwerveDriveBrake();
            } else {
                double rotationalRate = Constants.Kinematics.ROTATE_TO_POSE_PID_CONTROLLER.calculate(currentAngle.getRadians(), desiredAngle.getRadians());

                return alignToTargetDriveRequest.withVelocityX(controllerVelX * Constants.Kinematics.MAX_VELOCITY_METERS_PER_SECOND) // Drive forward with negative Y (forward)
                    .withVelocityY(-controller.getLeftX() * Constants.Kinematics.MAX_VELOCITY_METERS_PER_SECOND) // Drive left with negative X (left)
                    .withRotationalRate(rotationalRate * Constants.Kinematics.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND); // Use angular rate for rotation
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

    @Override
    public void consumeVisionPoseEstimate(Pose2d pose, double timestamp, Matrix<N3, N1> estimationStdDevs) {
        this.addVisionMeasurement(pose, timestamp, estimationStdDevs);
    }
}
