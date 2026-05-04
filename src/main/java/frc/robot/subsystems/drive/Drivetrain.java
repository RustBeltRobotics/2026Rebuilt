package frc.robot.subsystems.drive;

import java.util.function.Supplier;

import org.photonvision.EstimatedRobotPose;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
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
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.generated.TunerConstants;
import frc.robot.util.AlertManager;
import frc.robot.vision.VisionEstimateConsumer;
import frc.robot.vision.VisionPoseEstimationResult;

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
    private final DoublePublisher alignDriveTargetRotationDegreesPublisher = NetworkTableInstance.getDefault().getDoubleTopic("/RBR/Drivetrain/AutoAlign/Rotation/Target/degrees").publish();
    private final DoublePublisher alignDriveCurrentRotationDegreesPublisher = NetworkTableInstance.getDefault().getDoubleTopic("/RBR/Drivetrain/AutoAlign/Rotation/Current/degrees").publish();

    private static final Pose2d testShortDefensePose = new Pose2d(2.231, 4.003, new Rotation2d());
    private static final Pose2d testLongDefensePose = new Pose2d(1.181, 4.003, new Rotation2d());

    private final DoubleEntry kPEntry = NetworkTableInstance.getDefault().getTable("Tuning")
        .getDoubleTopic("HeadingController/kP")
        .getEntry(Constants.Kinematics.RotateToPosePID.K_P); // 5.0 is the default

    private final DoubleEntry kDEntry = NetworkTableInstance.getDefault().getTable("Tuning")
        .getDoubleTopic("HeadingController/kD")
        .getEntry(Constants.Kinematics.RotateToPosePID.K_D);

    private double priorHeadingControllerKp = Constants.Kinematics.RotateToPosePID.K_P;
    private double priorHeadingControllerKd = Constants.Kinematics.RotateToPosePID.K_D;

    private final SwerveRequest.FieldCentric teleopRequest = new SwerveRequest.FieldCentric()
        .withDeadband(Constants.Kinematics.MAX_VELOCITY_METERS_PER_SECOND * Constants.Controls.CONTROLLER_DEADBAND)
        .withRotationalDeadband(Constants.Kinematics.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND * Constants.Controls.CONTROLLER_DEADBAND)
        .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors

    private final SwerveRequest.FieldCentric alignToTargetDriveRequest = new SwerveRequest.FieldCentric()
        .withDeadband(Constants.Kinematics.MAX_VELOCITY_METERS_PER_SECOND * Constants.Controls.CONTROLLER_DEADBAND)
        .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors

    private final SwerveRequest.FieldCentricFacingAngle autoAlignRequest = new SwerveRequest.FieldCentricFacingAngle()
        .withHeadingPID(Constants.Kinematics.RotateToPosePID.K_P, Constants.Kinematics.RotateToPosePID.K_I, Constants.Kinematics.RotateToPosePID.K_D);

    private final SwerveRequest.ApplyRobotSpeeds pathApplyRobotSpeeds = new SwerveRequest.ApplyRobotSpeeds();

    private final SwerveRequest.SwerveDriveBrake applyBrakeRequest = new SwerveRequest.SwerveDriveBrake();

    private boolean initialPoseSetViaVision = false;
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
        //This is required to get the entry to show in NT for the first time
        kPEntry.setDefault(Constants.Kinematics.RotateToPosePID.K_P);
        kDEntry.setDefault(Constants.Kinematics.RotateToPosePID.K_D);
    }

    public void applyAutoCurrentLimits() {
        for (int i = 0; i < 4; i++) {
            var talonFXConfigurator = this.getModule(i).getDriveMotor().getConfigurator();
            var talonFXConfigs = new TalonFXConfiguration();
            talonFXConfigurator.refresh(talonFXConfigs);  //read current config from motor controller into talonFXConfigs
            var currentLimitConfig = talonFXConfigs.CurrentLimits;
            currentLimitConfig.SupplyCurrentLimit = Constants.Kinematics.DRIVE_MOTOR_SUPPLY_CURRENT_LIMIT_AUTO; //set new supply current limit for auto
            talonFXConfigurator.apply(talonFXConfigs);  //apply the updated configuration to the motor controller
        }
    }

    public void applyTeleopCurrentLimits(double newCurrentLimit) {
        for (int i = 0; i < 4; i++) {
            var talonFXConfigurator = this.getModule(i).getDriveMotor().getConfigurator();
            var talonFXConfigs = new TalonFXConfiguration();
            talonFXConfigurator.refresh(talonFXConfigs);  //read current config from motor controller into talonFXConfigs
            var currentLimitConfig = talonFXConfigs.CurrentLimits;
            currentLimitConfig.SupplyCurrentLimit = newCurrentLimit; //set new supply current limit for teleop
            talonFXConfigurator.apply(talonFXConfigs);  //apply the updated configuration to the motor controller
        }
    }

    @Override
    public void periodic() {
        super.periodic();

        if (Constants.Game.ENABLE_LIVE_PID_VALUE_TUNING) {
            double kP = kPEntry.get();
            double kD = kDEntry.get();

            if (kP != priorHeadingControllerKp || kD != priorHeadingControllerKd) {
                autoAlignRequest.HeadingController.setP(kP);
                autoAlignRequest.HeadingController.setD(kD);
                priorHeadingControllerKp = kP;
                priorHeadingControllerKd = kD;
                AlertManager.addAlert("DriveAutoAlign", "DriveAutoAlign PID changed! kP: " + kP + " kD: " + kD, AlertType.kInfo);
            }
        }

        if (Constants.Game.ENABLE_DEBUG_NT_LOGGING) {
            SwerveDriveState swerveDriveState = getState();
            driveForwardSpeedPublisher.set(swerveDriveState.Speeds.vxMetersPerSecond);

            SwerveModule<?, ?, ?> frontLeftModule = getModule(0);
            SwerveModule<?, ?, ?> frontRightModule = getModule(1);
            SwerveModule<?, ?, ?> backLeftModule = getModule(2);
            SwerveModule<?, ?, ?> backRightModule = getModule(3);
            frontLeftDriveCurrentPublisher.set(frontLeftModule.getDriveMotor().getSupplyCurrent().getValueAsDouble());
            frontLeftSteerCurrentPublisher.set(frontLeftModule.getSteerMotor().getSupplyCurrent().getValueAsDouble());
            frontRightDriveCurrentPublisher.set(frontRightModule.getDriveMotor().getSupplyCurrent().getValueAsDouble());
            frontRightSteerCurrentPublisher.set(frontRightModule.getSteerMotor().getSupplyCurrent().getValueAsDouble());
            backLeftDriveCurrentPublisher.set(backLeftModule.getDriveMotor().getSupplyCurrent().getValueAsDouble());
            backLeftSteerCurrentPublisher.set(backLeftModule.getSteerMotor().getSupplyCurrent().getValueAsDouble());
            backRightDriveCurrentPublisher.set(backRightModule.getDriveMotor().getSupplyCurrent().getValueAsDouble());
            backRightSteerCurrentPublisher.set(backRightModule.getSteerMotor().getSupplyCurrent().getValueAsDouble());
        }
    }

    public void updateTelemetry() {
        isAutoTargetingPublisher.accept(isAutoTargeting);
    }

    public Command teleopDrive(CommandXboxController controller) {
        return applyRequest(() -> {
            isAutoTargeting = false;
            double xControllerValue = modifyDriverControllerInput(controller.getLeftY());  //forward/back
            xControllerValue = xControllerValue * RobotContainer.MAX_SPEED_FACTOR;
            double yControllerValue = modifyDriverControllerInput(controller.getLeftX());  //left/right
            yControllerValue = yControllerValue * RobotContainer.MAX_SPEED_FACTOR;
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

    public Command brakeAndLockWheels() {
        return applyRequest(() -> applyBrakeRequest);
    }

    public Command teleOpDriveWithAutoAimToTarget(CommandXboxController controller, Supplier<Pose2d> targetPoseSupplier) {
        return applyRequest(() -> {
            isAutoTargeting = true;

            double xControllerValue = -modifyDriverControllerInput(controller.getLeftY());  //forward/back
            xControllerValue = xControllerValue * RobotContainer.MAX_SPEED_FACTOR;
            double yControllerValue = -modifyDriverControllerInput(controller.getLeftX());  //left/right
            yControllerValue = yControllerValue * RobotContainer.MAX_SPEED_FACTOR;

            Pose2d drivePose = getState().Pose;
            alignDriveCurrentRotationDegreesPublisher.set(drivePose.getRotation().getDegrees());
            Pose2d targetPose = targetPoseSupplier.get();
            targetHubPose.set(targetPose);
            Translation2d toTarget = targetPose.getTranslation().minus(drivePose.getTranslation());
            Rotation2d headingToTarget = toTarget.getAngle();
            headingToTarget = headingToTarget.plus(Rotation2d.fromDegrees(180)); //raw headingToTarget results in back of robot facing the hub, so flip it
            alignDriveTargetRotationPublisher.set(headingToTarget);
            alignDriveTargetRotationDegreesPublisher.set(headingToTarget.getDegrees());

            return autoAlignRequest.withVelocityX(xControllerValue * Constants.Kinematics.MAX_VELOCITY_METERS_PER_SECOND)
                .withVelocityY(yControllerValue * Constants.Kinematics.MAX_VELOCITY_METERS_PER_SECOND)
                .withTargetDirection(headingToTarget);
        });
    }
/* 
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
*/
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

        return Units.Meters.of(centerToTargetMeters);
    }

    public Distance getShotDistance() {
        return getShotDistance(Constants.Game.getHubPose().toPose2d().getTranslation());
    }

    @Override
    public void consumeVisionPoseEstimate(VisionPoseEstimationResult visionPoseEstimationResult) {
        EstimatedRobotPose estimatedRobotPose = visionPoseEstimationResult.getEstimatedRobotPose();
        Pose2d pose = estimatedRobotPose.estimatedPose.toPose2d();
        double timestamp = estimatedRobotPose.timestampSeconds;
        Matrix<N3, N1> estimationStdDevs = visionPoseEstimationResult.getVisionMeasurementStdDevs();

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

    public Command resetPoseUsingVision() {
        if (latestVisionPose != null) {
            Pose2d newPose = new Pose2d(latestVisionPose.getTranslation(), currentPoseSupplier.get().getRotation());

            return runOnce(() -> resetPose(newPose));
        } else {
            return runOnce(() -> {
                ;
            });
        }
    }

    public Command resetState() {
        return Commands.runOnce(() -> {
            initialPoseSetViaVision = false;
            isAutoTargeting = false;
            latestVisionPose = null;
        }, this);
    }

}
