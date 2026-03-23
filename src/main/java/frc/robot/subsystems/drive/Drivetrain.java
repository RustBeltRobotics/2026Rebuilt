package frc.robot.subsystems.drive;

import java.util.function.Supplier;

import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.swerve.SwerveModule;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.util.FlippingUtil;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.DoubleEntry;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants;
import frc.robot.generated.TunerConstants;
import frc.robot.util.AlertManager;
import frc.robot.vision.VisionEstimateConsumer;

public class Drivetrain extends CommandSwerveDrivetrain implements VisionEstimateConsumer {

    private final BooleanPublisher isAutoTargetingPublisher = NetworkTableInstance.getDefault().getBooleanTopic("/RBR/AutoTarget/Activated").publish();
    private final DoublePublisher driveForwardSpeedPublisher = NetworkTableInstance.getDefault().getDoubleTopic("/RBR/Drivetrain/ChassisSpeeds/vxMetersPerSecond").publish();
    private final DoublePublisher frontLeftDriveCurrentPublisher = NetworkTableInstance.getDefault().getDoubleTopic("/RBR/Drivetrain/Drive/Current/FL").publish();
    private final DoublePublisher frontRightDriveCurrentPublisher = NetworkTableInstance.getDefault().getDoubleTopic("/RBR/Drivetrain/Drive/Current/FR").publish();
    private final DoublePublisher backLeftDriveCurrentPublisher = NetworkTableInstance.getDefault().getDoubleTopic("/RBR/Drivetrain/Drive/Current/BL").publish();
    private final DoublePublisher backRightDriveCurrentPublisher = NetworkTableInstance.getDefault().getDoubleTopic("/RBR/Drivetrain/Drive/Current/BR").publish();
    private final DoublePublisher frontLeftSteerCurrentPublisher = NetworkTableInstance.getDefault().getDoubleTopic("/RBR/Drivetrain/Steer/Current/FL").publish();
    private final DoublePublisher frontRightSteerCurrentPublisher = NetworkTableInstance.getDefault().getDoubleTopic("/RBR/Drivetrain/Steer/Current/FR").publish();
    private final DoublePublisher backLeftSteerCurrentPublisher = NetworkTableInstance.getDefault().getDoubleTopic("/RBR/Drivetrain/Steer/Current/BL").publish();
    private final DoublePublisher backRightSteerCurrentPublisher = NetworkTableInstance.getDefault().getDoubleTopic("/RBR/Drivetrain/Steer/Current/BR").publish();

    private final StructPublisher<Pose2d> targetHubPose = NetworkTableInstance.getDefault().getStructTopic("/RBR/Drivetrain/AutoAlign/Hub/Pose", Pose2d.struct).publish();
    private final StructPublisher<Rotation2d> alignDriveTargetRotationPublisher = NetworkTableInstance.getDefault().getStructTopic("/RBR/Drivetrain/AutoAlign/Rotation/Target", Rotation2d.struct).publish();
    private final StructPublisher<Rotation2d> alignDriveDeltaRotationNewPublisher = NetworkTableInstance.getDefault().getStructTopic("/RBR/Drivetrain/AutoAlign/Rotation/DeltaNew", Rotation2d.struct).publish();
    private final DoublePublisher alignDriveRadiansResultPublisher = NetworkTableInstance.getDefault().getDoubleTopic("/RBR/Drivetrain/AutoAlign/Result/Radians").publish();
    private final BooleanPublisher alreadyAtGoalPublisher = NetworkTableInstance.getDefault().getBooleanTopic("/RBR/Drivetrain/AutoAlign/AlreadyAtGoal").publish();
    private final BooleanPublisher notTryingToDrivePublisher = NetworkTableInstance.getDefault().getBooleanTopic("/RBR/Drivetrain/AutoAlign/NotTryingToDrive").publish();

    private final DoubleEntry kPEntry = NetworkTableInstance.getDefault().getTable("Tuning")
        .getDoubleTopic("HeadingController/kP")
        .getEntry(0.14); // 0.14 is the default

    private final DoubleEntry kDEntry = NetworkTableInstance.getDefault().getTable("Tuning")
        .getDoubleTopic("HeadingController/kD")
        .getEntry(0.0);

    private int priorHeadingControllerKp = 0.14;
    private int priorHeadingControllerKd = 0;

    private final SwerveRequest.FieldCentric teleopRequest = new SwerveRequest.FieldCentric()
        .withDeadband(Constants.Kinematics.MAX_VELOCITY_METERS_PER_SECOND * Constants.Controls.CONTROLLER_DEADBAND)
        .withRotationalDeadband(Constants.Kinematics.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND * Constants.Controls.CONTROLLER_DEADBAND)
        .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors

    private final SwerveRequest.FieldCentric alignToTargetDriveRequest = new SwerveRequest.FieldCentric()
        .withDeadband(Constants.Kinematics.MAX_VELOCITY_METERS_PER_SECOND * Constants.Controls.CONTROLLER_DEADBAND)
        .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors

    private final SwerveRequest.FieldCentricFacingAngle autoAlignRequest = new SwerveRequest.FieldCentricFacingAngle()
        .withHeadingPID(0.14, 0, 0);

    private final SwerveRequest.ApplyRobotSpeeds pathApplyRobotSpeeds = new SwerveRequest.ApplyRobotSpeeds();

    private boolean initialPoseSetViaVision = false;
    private boolean rampOrTipDetected = false;
    private boolean needToCorrectOdometryUsingVision = false;
    private boolean isAutoTargeting = false;
    private Pose2d latestVisionPose;

    private Supplier<Pose2d> currentPoseSupplier = () -> getState().Pose;
    //TODO: test these for auto-lowering the hood when approaching the trench 
    private Trigger nearingUpperTrenchLeftTrigger = nearFieldPositionAutoFlipped(new Translation2d(4.0, 6.772), 2.0, currentPoseSupplier);
    private Trigger nearingUpperTrenchRightTrigger = nearFieldPositionAutoFlipped(new Translation2d(5.223, 6.597), 2.0, currentPoseSupplier);
    private Trigger nearingLowerTrenchLeftTrigger = nearFieldPositionAutoFlipped(new Translation2d(4.0, 1.314), 2.0, currentPoseSupplier);
    private Trigger nearingLowerTrenchRightTrigger = nearFieldPositionAutoFlipped(new Translation2d(5.223, 1.330), 2.0, currentPoseSupplier);

    public Drivetrain() {
        super(TunerConstants.DrivetrainConstants, TunerConstants.FrontLeft, TunerConstants.FrontRight, TunerConstants.BackLeft, TunerConstants.BackRight);
        configureAutoBuilder();
    }

    @Override
    public void periodic() {
        super.periodic();

        double kP = kPEntry.get();
        double kD = kDEntry.get();

        if (kP != priorHeadingControllerKp || kD != priorHeadingControllerKd) {
            headingController.setPID(kP, 0.0, kD);
            priorHeadingControllerKp = kP;
            priorHeadingControllerKd = kD;
        }

        SwerveDriveState swerveDriveState = getState();
        driveForwardSpeedPublisher.set(swerveDriveState.Speeds.vxMetersPerSecond);

        SwerveModule<?, ?, ?> frontLeftModule = getModule(0);
        SwerveModule<?, ?, ?> frontRightModule = getModule(1);
        SwerveModule<?, ?, ?> backLeftModule = getModule(2);
        SwerveModule<?, ?, ?> backRightModule = getModule(3);
        frontLeftDriveCurrentPublisher.set(frontLeftModule.getDriveMotor().getStatorCurrent().getValueAsDouble());
        frontLeftSteerCurrentPublisher.set(frontLeftModule.getSteerMotor().getStatorCurrent().getValueAsDouble());
        frontRightDriveCurrentPublisher.set(frontRightModule.getDriveMotor().getStatorCurrent().getValueAsDouble());
        frontRightSteerCurrentPublisher.set(frontRightModule.getSteerMotor().getStatorCurrent().getValueAsDouble());
        backLeftDriveCurrentPublisher.set(backLeftModule.getDriveMotor().getStatorCurrent().getValueAsDouble());
        backLeftSteerCurrentPublisher.set(backLeftModule.getSteerMotor().getStatorCurrent().getValueAsDouble());
        backRightDriveCurrentPublisher.set(backRightModule.getDriveMotor().getStatorCurrent().getValueAsDouble());
        backRightSteerCurrentPublisher.set(backRightModule.getSteerMotor().getStatorCurrent().getValueAsDouble());

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

    public Command teleOpDriveWithAutoAimToTarget(CommandXboxController controller, Supplier<Pose2d> targetPoseSupplier) {
        return applyRequest(() -> {
            isAutoTargeting = true;

            double xControllerValue = -modifyDriverControllerInput(controller.getLeftY());  //forward/back
            double yControllerValue = -modifyDriverControllerInput(controller.getLeftX());  //left/right

            Pose2d drivePose = getState().Pose;
            Pose2d targetPose = targetPoseSupplier.get();
            Translation2d toTarget = targetPose.getTranslation().minus(drivePose.getTranslation());
            Rotation2d headingToTarget = toTarget.getAngle();

            return autoAlignRequest.withVelocityX(xControllerValue * Constants.Kinematics.MAX_VELOCITY_METERS_PER_SECOND)
                .withVelocityY(yControllerValue * Constants.Kinematics.MAX_VELOCITY_METERS_PER_SECOND)
                .withTargetDirection(headingToTarget);
        });
    }

    public Command alignToTargetDrive(CommandXboxController controller, Supplier<Pose2d> targetPoseSupplier) {
        return applyRequest(() -> {
            isAutoTargeting = true;
            //TODO: update this to use the modified controller inputs once we verify it works well for teleop driving
            // double controllerVelX = -controller.getLeftY();  //forward/back
            // double controllerVelY = -controller.getLeftX();  //left/right strafe

            double xControllerValue = -modifyDriverControllerInput(controller.getLeftY());  //forward/back
            double yControllerValue = -modifyDriverControllerInput(controller.getLeftX());  //left/right

            Pose2d drivePose = getState().Pose;
            Pose2d targetPose = targetPoseSupplier.get();
            targetHubPose.set(targetPose);
            Rotation2d currentAngle = drivePose.getRotation();

            Translation2d difference = targetPose.getTranslation().minus(drivePose.getTranslation());
            Rotation2d angleToTarget = difference.getAngle();
            Rotation2d deltaAngle = angleToTarget.minus(drivePose.getRotation());
            alignDriveTargetRotationPublisher.set(angleToTarget);
            alignDriveDeltaRotationNewPublisher.set(deltaAngle);
            double wrappedAngleDeg = MathUtil.inputModulus(deltaAngle.getDegrees(), -180.0, 180.0);

            boolean notTryingToDrive = Math.hypot(xControllerValue, yControllerValue) < Constants.Controls.CONTROLLER_DEADBAND;
            notTryingToDrivePublisher.set(notTryingToDrive);
            boolean alreadyFacingGoal = Math.abs(wrappedAngleDeg) < Constants.Kinematics.EPSILON_ANGLE_TO_GOAL.in(Units.Degrees);
            alreadyAtGoalPublisher.set(alreadyFacingGoal);

            if (alreadyFacingGoal && notTryingToDrive) {
                return new SwerveRequest.SwerveDriveBrake();
            } else {
                double rotationalRate = Constants.Kinematics.ROTATE_TO_POSE_PID_CONTROLLER.calculate(currentAngle.getRadians(), angleToTarget.getRadians());
                double calcRotationRadians = rotationalRate * Constants.Kinematics.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND;
                alignDriveRadiansResultPublisher.set(calcRotationRadians);

                return alignToTargetDriveRequest.withVelocityX(xControllerValue * Constants.Kinematics.MAX_VELOCITY_METERS_PER_SECOND) // Drive forward with negative Y (forward)
                    .withVelocityY(yControllerValue * Constants.Kinematics.MAX_VELOCITY_METERS_PER_SECOND) // Drive left with negative X (left)
                    .withRotationalRate(rotationalRate * Constants.Kinematics.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND); // Use angular rate for rotation
            }
        });
    }

    public void resetPoseForAutoStart(Pose2d pose) {
        Pose2d targetPose = pose;
        //if this is autonomous and we have valid vision data, override the provided pose with vision data
        if (DriverStation.isAutonomous() && Constants.Vision.VISION_ENABLED && Constants.Vision.TAKE_INITIAL_AUTO_POSE_FROM_VISION
                && !initialPoseSetViaVision && this.latestVisionPose != null) {
            targetPose = this.latestVisionPose;
            initialPoseSetViaVision = true;
            AlertManager.addAlert("AutoVision", "Initial AutoPose [" + pose + "] overriden with vision to [" + this.latestVisionPose + "]", AlertType.kInfo);
        } else {
            AlertManager.addAlert("AutoVision", "Initial AutoPose [" + pose + "] from PathPlanner accepted as is", AlertType.kInfo);
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
        if (Constants.Vision.TAKE_POSE_ESTIMATES_FROM_VISION) {
            this.addVisionMeasurement(pose, timestamp, estimationStdDevs);
        }
        this.latestVisionPose = pose;
    }

    //Taken from PathPlanner's AutoBuilder class for use outside the context of autnomous
    private Trigger nearFieldPositionAutoFlipped(Translation2d blueFieldPosition, double toleranceMeters, Supplier<Pose2d> currentPoseSupplier) {
        return new Trigger(
            () -> {
                boolean shouldFlipProvidedPosition = DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red;
                if (shouldFlipProvidedPosition) {
                    Translation2d redFieldPosition = FlippingUtil.flipFieldPosition(blueFieldPosition);
                    return currentPoseSupplier.get().getTranslation().getDistance(redFieldPosition) <= toleranceMeters;
                } else {
                    return currentPoseSupplier.get().getTranslation().getDistance(blueFieldPosition) <= toleranceMeters;
                }
            }
        );
    }

    public Supplier<Pose2d> getCurrentPoseSupplier() {
        return currentPoseSupplier;
    }

}
