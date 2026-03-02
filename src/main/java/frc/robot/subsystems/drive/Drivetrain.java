package frc.robot.subsystems.drive;

import java.util.function.Supplier;

import org.photonvision.PhotonUtils;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants;
import frc.robot.generated.TunerConstants;
import frc.robot.util.AlertManager;
import frc.robot.vision.VisionEstimateConsumer;

public class Drivetrain extends CommandSwerveDrivetrain implements VisionEstimateConsumer {

    private StructPublisher<Rotation2d> targetTurnAnglePublisher = NetworkTableInstance.getDefault().getStructTopic("/RBR/AutoTarget/Angle", Rotation2d.struct).publish();
    private BooleanPublisher isAutoTargetingPublisher = NetworkTableInstance.getDefault().getBooleanTopic("/RBR/AutoTarget/Activated").publish();

    private final SwerveRequest.FieldCentric teleopRequest = new SwerveRequest.FieldCentric()
        .withDeadband(Constants.Kinematics.MAX_VELOCITY_METERS_PER_SECOND * Constants.Controls.CONTROLLER_DEADBAND)
        .withRotationalDeadband(Constants.Kinematics.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND * Constants.Controls.CONTROLLER_DEADBAND)
        .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors

    private final SwerveRequest.FieldCentric alignToTargetDriveRequest = new SwerveRequest.FieldCentric()
        .withDeadband(Constants.Kinematics.MAX_VELOCITY_METERS_PER_SECOND * Constants.Controls.CONTROLLER_DEADBAND)
        .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors

    private final SwerveRequest.ApplyRobotSpeeds pathApplyRobotSpeeds = new SwerveRequest.ApplyRobotSpeeds();

    private boolean initialPoseSetViaVision = false;
    private boolean rampOrTipDetected = false;
    private boolean needToCorrectOdometryUsingVision = false;
    private boolean isAutoTargeting = false;
    private Pose2d latestVisionPose;
    private Rotation2d targetTurnAngle = new Rotation2d();

    public Drivetrain() {
        super(TunerConstants.DrivetrainConstants, TunerConstants.FrontLeft, TunerConstants.FrontRight, TunerConstants.BackLeft, TunerConstants.BackRight);
        configureAutoBuilder();
    }

    @Override
    public void periodic() {
        super.periodic();
        Pigeon2 pigeon2 = getPigeon2();
        double robotPitch = Math.abs(pigeon2.getPitch().getValueAsDouble());
        double robotRoll = Math.abs(pigeon2.getRoll().getValueAsDouble());
        //periodically check for ramp/tip condition
        if (!rampOrTipDetected && (robotPitch > Constants.Kinematics.TIP_THRESHOLD_DEGREES || robotRoll > Constants.Kinematics.TIP_THRESHOLD_DEGREES)) {
            rampOrTipDetected = true;
            AlertManager.addAlert("Ramp/Tip", "Ramp/Tip detected! Pitch: " + robotPitch + " Roll: " + robotRoll, AlertType.kInfo);
        } else if (rampOrTipDetected && robotPitch <= Constants.Kinematics.ROLL_PITCH_ERROR && robotRoll <= Constants.Kinematics.ROLL_PITCH_ERROR) {
            //reset condition when back within safe limits
            rampOrTipDetected = false;
            needToCorrectOdometryUsingVision = true;
            AlertManager.addAlert("Ramp/Tip", "Ramp/Tip cleared. Pitch: " + robotPitch + " Roll: " + robotRoll, AlertType.kInfo);
        }
    }

    public void updateTelemetry() {
        targetTurnAnglePublisher.accept(targetTurnAngle);
        isAutoTargetingPublisher.accept(isAutoTargeting);
    }

    public Command teleopDrive(CommandXboxController controller) {
        return applyRequest(() -> {
            isAutoTargeting = false;
            double xControllerValue = modifyDriverControllerInput(controller.getLeftY());  //forward/back
            double yControllerValue = modifyDriverControllerInput(controller.getLeftX());  //left/right
            double rotationalControllerValue = modifyDriverControllerInput(controller.getRightX());  //rotation
            return teleopRequest.withVelocityX(-xControllerValue * Constants.Kinematics.MAX_VELOCITY_METERS_PER_SECOND) // Drive forward with negative Y (forward)
                    .withVelocityY(-yControllerValue * Constants.Kinematics.MAX_VELOCITY_METERS_PER_SECOND) // Drive left with negative X (left)
                    .withRotationalRate(-rotationalControllerValue * Constants.Kinematics.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND); // Drive counterclockwise with negative X (left)
        }).withName("Teleop Drive");
    }

    private double modifyDriverControllerInput(double input) {
        // scale the controller input - See https://www.desmos.com/calculator/bnqnldev69 for function graph
        return (Math.signum(input) * Math.pow(Math.abs(input), 3.7) + (input * 0.43)) / (1 + 0.42);
    }

    public Command alignToTargetDrive(CommandXboxController controller, Supplier<Pose2d> targetPoseSupplier) {
        return applyRequest(() -> {
            isAutoTargeting = true;
            //TODO: update this to use the modified controller inputs once we verify it works well for teleop driving
            double controllerVelX = -controller.getLeftY();  //forward/back
            double controllerVelY = -controller.getLeftX();  //left/right strafe

            Pose2d drivePose = getState().Pose;
            Pose2d targetPose = targetPoseSupplier.get();
            Rotation2d currentAngle = drivePose.getRotation();

            targetTurnAngle = PhotonUtils.getYawToPose(drivePose, targetPose);

            Rotation2d deltaAngle = currentAngle.minus(targetTurnAngle);
            double wrappedAngleDeg = MathUtil.inputModulus(deltaAngle.getDegrees(), -180.0, 180.0);

            boolean notTryingToDrive = Math.hypot(controllerVelX, controllerVelY) < Constants.Controls.CONTROLLER_DEADBAND;
            boolean alreadyFacingGoal = Math.abs(wrappedAngleDeg) < Constants.Kinematics.EPSILON_ANGLE_TO_GOAL.in(Units.Degrees);

            if (alreadyFacingGoal && notTryingToDrive) {
                return new SwerveRequest.SwerveDriveBrake();
            } else {
                double rotationalRate = Constants.Kinematics.getRotateToPoseController().calculate(currentAngle.getRadians(), targetTurnAngle.getRadians());
                //TODO: switch to the following form once we have the PID fully tuned to avoid network table reads every loop
                //double rotationalRate = Constants.Kinematics.ROTATE_TO_POSE_PID_CONTROLLER.calculate(currentAngle.getRadians(), targetTurnAngle.getRadians());

                return alignToTargetDriveRequest.withVelocityX(controllerVelX * Constants.Kinematics.MAX_VELOCITY_METERS_PER_SECOND) // Drive forward with negative Y (forward)
                    .withVelocityY(-controller.getLeftX() * Constants.Kinematics.MAX_VELOCITY_METERS_PER_SECOND) // Drive left with negative X (left)
                    .withRotationalRate(rotationalRate * Constants.Kinematics.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND); // Use angular rate for rotation
            }
        });
    }

    public void resetPoseForAutoStart(Pose2d pose) {
        Pose2d targetPose = pose;
        //if this is autonomous and we have valid vision data, override the provided pose with vision data
        if (DriverStation.isAutonomous() && Constants.Vision.VISION_ENABLED && !initialPoseSetViaVision && this.latestVisionPose != null) {
            targetPose = this.latestVisionPose;
            initialPoseSetViaVision = true;
            AlertManager.addAlert("AutoVision", "Initial AutoPose [" + pose + "] overriden with vision to [" + this.latestVisionPose + "]", AlertType.kInfo);
        }

        resetPose(targetPose);
    }

    private void configureAutoBuilder() {
        RobotConfig ppRobotConfig = Constants.PathPlanner.ROBOT_CONFIG;

        AutoBuilder.configure(
                () -> getState().Pose,   // Supplier of current robot pose
                this::resetPoseForAutoStart,   // Consumer for seeding robot pose against auto
                () -> getState().Speeds, // Supplier of current robot speeds
                // Consumer of ChassisSpeeds and feedforwards to drive the robot
                (speeds, feedforwards) -> setControl(
                    pathApplyRobotSpeeds.withSpeeds(ChassisSpeeds.discretize(speeds, 0.020))
                        .withWheelForceFeedforwardsX(feedforwards.robotRelativeForcesXNewtons())
                        .withWheelForceFeedforwardsY(feedforwards.robotRelativeForcesYNewtons())
                ),
                new PPHolonomicDriveController( // HolonomicPathFollowerConfig, this should likely live in your Constants class
                        new PIDConstants(Constants.PathPlanner.translation_P, Constants.PathPlanner.translation_I, Constants.PathPlanner.translation_D), // Translation PID constants
                        new PIDConstants(Constants.PathPlanner.rotation_P, Constants.PathPlanner.rotation_I, Constants.PathPlanner.rotation_D) // Rotation PID constants
                ),
                ppRobotConfig,
                // Assume the path needs to be flipped for Red vs Blue, this is normally the case
                () -> DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red,
                this // Reference to this subsystem to set requirements
        );
    }

    public Distance getShotDistance(Translation2d targetPose) {
        Pose2d robotPose = getState().Pose;
        double centerToTargetMeters = robotPose.getTranslation().getDistance(targetPose);
        // Constants.Kinematics.SHOOTER_TRANSLATION_FROM_ROBOT_CENTER.
        double centerToShooterMeters = 123; //TODO: fix this reference
        double shooterToTargetMeters = Math.sqrt(Math.pow(centerToTargetMeters, 2.0) - Math.pow(centerToShooterMeters, 2.0));

        return Units.Meters.of(shooterToTargetMeters);
    }

    public Distance getShotDistance() {
        return getShotDistance(Constants.Game.getHubPose().toPose2d().getTranslation());
    }

    @Override
    public void consumeVisionPoseEstimate(Pose2d pose, double timestamp, Matrix<N3, N1> estimationStdDevs) {
        if (needToCorrectOdometryUsingVision) {
            //boost vision odometry via lowering standard deviations to compensate for error in wheel odometry after ramp/tip
            //TODO: test this and tune boost factor
            estimationStdDevs = estimationStdDevs.times(0.25); //boost by factor of 4
            needToCorrectOdometryUsingVision = false;
            AlertManager.addAlert("Vision", "Ramp/Tip compensated using new pose: " + pose, AlertType.kInfo);
        }
        this.addVisionMeasurement(pose, timestamp, estimationStdDevs);
        this.latestVisionPose = pose;
    }

}
