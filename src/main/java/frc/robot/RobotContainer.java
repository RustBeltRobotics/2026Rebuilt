// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.function.Supplier;

import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveDriveState;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.FollowPathCommand;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.commands.DefaultLedCommand;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.LED;
import frc.robot.subsystems.ShooterFeederYams;
import frc.robot.subsystems.ShooterYams;
import frc.robot.subsystems.Spindexer;
import frc.robot.subsystems.drive.Drivetrain;
import frc.robot.util.HubStateTracker;
import frc.robot.util.SwerveTelemetryCTRE;
import frc.robot.vision.VisionSystem;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...

   // For limiting maximum speed (1.0 = 100% = full speed)
  private static double MAX_SPEED_FACTOR = Constants.Kinematics.INITIAL_DRIVE_MAX_SPEED_FACTOR;

  private final CommandXboxController driverController = new CommandXboxController(Constants.Controls.CONTROLLER_PORT_DRIVER);
  private final CommandXboxController operatorController = new CommandXboxController(Constants.Controls.CONTROLLER_PORT_OPERATOR);

  private final SwerveTelemetryCTRE swerveTelemetryCTRE = new SwerveTelemetryCTRE(MAX_SPEED_FACTOR * Constants.Kinematics.MAX_VELOCITY_METERS_PER_SECOND);
  private final Drivetrain drivetrain = new Drivetrain();
  private final ShooterYams shooter = new ShooterYams();
  private final ShooterFeederYams shooterFeeder = new ShooterFeederYams();
  private final Spindexer spindexer = new Spindexer();
  private final Intake intake = new Intake();
  private final Climber climber = new Climber();
  private final LED led = new LED();
  private final VisionSystem visionSystem;

  private final SendableChooser<Command> autoChooser;
  private final SendableChooser<Double> driveTrainSpeedChooser = new SendableChooser<>();
  private DoublePublisher maxSpeedFactorPublisher = NetworkTableInstance.getDefault().getDoubleTopic("/RBR/MaxSpeed").publish();

  public static double getMaxSpeed() {
    return MAX_SPEED_FACTOR * Constants.Kinematics.MAX_VELOCITY_METERS_PER_SECOND;
  }

  /** The container for the robot. Contains subsystems, I/O devices, and commands. */
  public RobotContainer() {
    registerPathPlannerNamedCommands();
    
    autoChooser = AutoBuilder.buildAutoChooser();

    driveTrainSpeedChooser.setDefaultOption(MAX_SPEED_FACTOR + "%", MAX_SPEED_FACTOR);
    driveTrainSpeedChooser.addOption("75%", 0.75);
    driveTrainSpeedChooser.addOption("50%", 0.5);
    driveTrainSpeedChooser.addOption("25%", 0.25);
    driveTrainSpeedChooser.onChange((newValue) -> {
      MAX_SPEED_FACTOR = newValue;
      swerveTelemetryCTRE.setMaxSpeed(getMaxSpeed());
    });
    Constants.Shuffleboard.COMPETITION_TAB.add("Drive Speed Selector", driveTrainSpeedChooser).withPosition(0, 2).withSize(2, 1);

    drivetrain.registerTelemetry(swerveTelemetryCTRE::telemeterize);

    if (Constants.Vision.VISION_ENABLED) {
      visionSystem = new VisionSystem(drivetrain::consumeVisionPoseEstimate);
    } else {
      visionSystem = null;
    }

    setDefaultCommands();
    configureBindings();
    configureAutos();

    // Speed up initial run of Pathplanner GUI commands
    FollowPathCommand.warmupCommand().schedule();
  }

  private void registerPathPlannerNamedCommands() {
    //TODO: implement
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
        // Idle while the robot is disabled. This ensures the configured
        // neutral mode is applied to the drive motors while disabled.
        final var idle = new SwerveRequest.Idle();
        RobotModeTriggers.disabled().whileTrue(drivetrain.applyRequest(() -> idle).ignoringDisable(true));

        // Run SysId routines when holding back/start and X/Y.
        // Note that each routine should be run exactly once in a single log.
        // Note: to switch between translation, steer and rotation routines, you need to reassign the value of
        //  CommandSwerveDrivetrain.m_sysIdRoutineToApply to m_sysIdRoutineTranslation, m_sysIdRoutineSteer or m_sysIdRoutineRotation respectively
        driverController.back().and(driverController.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        driverController.back().and(driverController.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        driverController.start().and(driverController.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        driverController.start().and(driverController.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

        // Reset the field-centric heading (set robot forward direction) on left bumper press.
        driverController.leftBumper().onTrue(drivetrain.runOnce(drivetrain::seedFieldCentric));

        driverController.rightBumper().whileTrue(intake.intakeFuel());
        driverController.leftTrigger().whileTrue(shooterFeeder.runAtAngularVelocity(Constants.ShooterFeeder.FEEDER_RPM));
        driverController.rightTrigger().whileTrue(spindexer.runAtDutyCycle(0.5));
        driverController.x().whileTrue(shooter.runAtAngularVelocity(Constants.Shooter.SHOOTER_TEST_RPM));

        Supplier<Pose2d> hubTargetPoseSupplier = () -> Constants.Game.getHubPose().toPose2d();
        /* 
        // Auto-align drive to hub while holding right bumper.
        driverController.rightBumper().whileTrue(Commands.parallel(
          shooter.prepVariableDistanceShot(() -> drivetrain.getShotDistance(hubTargetPoseSupplier.get().getTranslation())),
          drivetrain.alignToTargetDrive(driverController, hubTargetPoseSupplier)
        ));
        */
  }

  private void setDefaultCommands() {
    drivetrain.setDefaultCommand(drivetrain.teleopDrive(driverController).withName("Teleop Drive"));
    shooterFeeder.setDefaultCommand(shooterFeeder.stop());
    spindexer.setDefaultCommand(spindexer.stop());
    intake.setDefaultCommand(intake.stopIntakeWheelRotation());
    climber.setDefaultCommand(climber.stopCommand());
    //Default LED command removes all color output (sets to black)
    led.setDefaultCommand(new DefaultLedCommand(led));
  }

  public void configureAutos() {
    Constants.Shuffleboard.COMPETITION_TAB.add("Auto Selector", autoChooser).withPosition(0, 0).withSize(2, 1);
  }

  public void rumbleControllers(boolean rumble, boolean rumbleLeft, boolean rumbleRight) {
    double rumbleValue = rumble ? 0.25 : 0.0;
    XboxController driver = driverController.getHID();
    XboxController operator = operatorController.getHID();
    
    if (rumbleLeft) {
      driver.setRumble(RumbleType.kLeftRumble, rumbleValue);
      operator.setRumble(RumbleType.kLeftRumble, rumbleValue);
    }
    if (rumbleRight) {
      driver.setRumble(RumbleType.kRightRumble, rumbleValue);
      operator.setRumble(RumbleType.kRightRumble, rumbleValue);
    }
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
     Command chosenAutoCommand = autoChooser.getSelected();

    return chosenAutoCommand;
  }

  public void updatePeriodic() {
    visionSystem.periodic();
    //control LEDs based on robot state
    //TODO: use a color when there are no vision tags visible? i.e. !visionSystem.isReceivedNewVisionData
    if (DriverStation.isAutonomous()) {
      led.changeColor(Color.kOrange);
    } else {
      if (HubStateTracker.isAllianceHubActive()) {
        led.changeColor(Color.kGreen);
      } else {
        led.changeColor(Color.kRed);
      }
    }

    if (visionSystem != null) {
      //blink LED if no valid vision data received
      led.changeBlink(!visionSystem.isReceivedNewVisionData());
    }
  }

  public void updateTelemetry() {
    maxSpeedFactorPublisher.set(MAX_SPEED_FACTOR);
  }

  public void updateSimPeriodic() {
    //Note: CommandSwerveDrivetrain already handles state updates via startSimThread() logic
    if (visionSystem != null) {
      SwerveDriveState swerveState = drivetrain.getState();
      visionSystem.simulationPeriodic(swerveState.Pose);
    }
  }

  public static void setMaxSpeedFactor(double newSpeedFactor) {
    MAX_SPEED_FACTOR = newSpeedFactor;
  }
}
