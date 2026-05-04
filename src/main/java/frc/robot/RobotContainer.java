// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.function.Supplier;

import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveDriveState;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.FollowPathCommand;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.subsystems.IntakeArm;
import frc.robot.subsystems.IntakeRoller;
import frc.robot.subsystems.ShooterFeederYams;
import frc.robot.subsystems.ShooterYams;
import frc.robot.subsystems.RollingFloor;
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

   // For limiting maximum speed in teleop (1.0 = 100% = full speed)
  public static double MAX_SPEED_FACTOR = Constants.Kinematics.INITIAL_DRIVE_MAX_SPEED_FACTOR;

  private final CommandXboxController driverController = new CommandXboxController(Constants.Controls.CONTROLLER_PORT_DRIVER);
  private final CommandXboxController operatorController = new CommandXboxController(Constants.Controls.CONTROLLER_PORT_OPERATOR);

  private final SwerveTelemetryCTRE swerveTelemetryCTRE = new SwerveTelemetryCTRE(MAX_SPEED_FACTOR * Constants.Kinematics.MAX_VELOCITY_METERS_PER_SECOND);
  private final Drivetrain drivetrain = new Drivetrain();
  private final ShooterYams shooter = new ShooterYams();
  // private final ShooterSysId shooterSysId = new ShooterSysId();
  private final ShooterFeederYams shooterFeeder = new ShooterFeederYams();
  //Hood is no longer used in new design
  // private final ShooterHood shooterHood = new ShooterHood();
  private final RollingFloor rollingFloor = new RollingFloor();
  private final IntakeRoller intakeRoller = new IntakeRoller();
  private final IntakeArm intakeArm = new IntakeArm();
  // private final Climber climber = new Climber();
  private final VisionSystem visionSystem;

  private final SendableChooser<Command> autoChooser;
  private final SendableChooser<Double> driveTrainSpeedChooser = new SendableChooser<>();
  private final Trigger lastTenSecondsOfShiftTrigger = HubStateTracker.getLastTenSecondsShiftWarning();
  private final Trigger lastFiveSecondsOfShiftTrigger = HubStateTracker.getLastFiveSecondsShiftWarning();
  private DoublePublisher maxSpeedFactorPublisher = NetworkTableInstance.getDefault().getDoubleTopic("/RBR/MaxSpeed").publish();
  private DoublePublisher batteryVoltagePublisher = NetworkTableInstance.getDefault().getDoubleTopic("/RBR/Battery/Voltage").publish();
  private BooleanPublisher brownoutPublisher = NetworkTableInstance.getDefault().getBooleanTopic("/RBR/Battery/Brownout").publish();

  public static double getMaxSpeed() {
    return MAX_SPEED_FACTOR * Constants.Kinematics.MAX_VELOCITY_METERS_PER_SECOND;
  }

  /** The container for the robot. Contains subsystems, I/O devices, and commands. */
  public RobotContainer() {
    registerPathPlannerNamedCommands();
    
    autoChooser = AutoBuilder.buildAutoChooser();
    driveTrainSpeedChooser.setDefaultOption(MAX_SPEED_FACTOR + "%", MAX_SPEED_FACTOR);
    driveTrainSpeedChooser.addOption("100%", 1.0);

    if (Constants.Kinematics.INITIAL_DRIVE_MAX_SPEED_FACTOR != 0.90) {
      driveTrainSpeedChooser.addOption("90%", 0.90);
    }
    
    driveTrainSpeedChooser.addOption("85%", 0.85);
    driveTrainSpeedChooser.addOption("75%", 0.75);
    driveTrainSpeedChooser.addOption("50%", 0.5);
    driveTrainSpeedChooser.addOption("25%", 0.25);
    driveTrainSpeedChooser.onChange((newValue) -> {
      if (newValue != null) {
        MAX_SPEED_FACTOR = newValue;
        swerveTelemetryCTRE.setMaxSpeed(getMaxSpeed());
      }
    });
    Constants.Shuffleboard.COMPETITION_TAB.add("Drive Speed Selector", driveTrainSpeedChooser).withPosition(0, 2).withSize(2, 1);

    drivetrain.registerTelemetry(swerveTelemetryCTRE::telemeterize);

    if (Constants.Vision.VISION_ENABLED) {
      visionSystem = new VisionSystem(drivetrain::consumeVisionPoseEstimate, drivetrain.getCurrentPoseSupplier());
    } else {
      visionSystem = null;
    }

    lastTenSecondsOfShiftTrigger.whileTrue(Commands.startEnd(
        () -> driverController.getHID().setRumble(RumbleType.kBothRumble, 0.5),
        () -> driverController.getHID().setRumble(RumbleType.kBothRumble, 0.0)
    ));
    lastFiveSecondsOfShiftTrigger.whileTrue(Commands.startEnd(
        () -> driverController.getHID().setRumble(RumbleType.kBothRumble, 0.75),
        () -> driverController.getHID().setRumble(RumbleType.kBothRumble, 0.0)
    ));

    setDefaultCommands();
    configureBindings();
    configureAutos();

    // Speed up initial run of Pathplanner GUI commands
    FollowPathCommand.warmupCommand().schedule();
  }

  private void registerPathPlannerNamedCommands() { 
    Command runFullShootingSystem = Commands.parallel(
          shooterFeeder.runAtAngularVelocity(Constants.ShooterFeeder.FEEDER_RPM),
          rollingFloor.rollInwards(),
          Commands.sequence(Commands.waitSeconds(2.5), intakeArm.retract().withTimeout(0.5))  //TODO: graph and tune this delay based on time for shooter to get up to speed (adjust if we leave the shooter running at low RPM idle between shots)
        ).withTimeout(3.0);  //TODO: We may want to lower the timeout here, evaluate after testing

    Command runFullShootingSystemBump = Commands.parallel(
          shooterFeeder.runAtAngularVelocity(Constants.ShooterFeeder.FEEDER_RPM),
          rollingFloor.rollInwards(),
          Commands.sequence(Commands.waitSeconds(2.0), intakeArm.retract().withTimeout(0.5))  //TODO: graph and tune this delay based on time for shooter to get up to speed (adjust if we leave the shooter running at low RPM idle between shots)
        ).withTimeout(3.0);

    Command runFullShootingSystemLayup = Commands.parallel(
          shooterFeeder.runAtAngularVelocity(Constants.ShooterFeeder.FEEDER_RPM),
          rollingFloor.rollInwards(),
          Commands.sequence(Commands.waitSeconds(2.0), intakeArm.retract().withTimeout(0.5))  //TODO: graph and tune this delay based on time for shooter to get up to speed (adjust if we leave the shooter running at low RPM idle between shots)
        ).withTimeout(3.0);

    NamedCommands.registerCommand("start-shooter", shooter.runAtAngularVelocity(Constants.Shooter.SHOOTER_TRENCH_RPM));
    //TODO: test start-shooter-bump, may need to bump RPM a bit and save as a diff constant
    NamedCommands.registerCommand("start-shooter-bump", shooter.runAtAngularVelocity(Constants.Shooter.SHOOTER_BUMP_AUTO_RPM));
    NamedCommands.registerCommand("start-shooter-center", shooter.runAtAngularVelocity(Constants.Shooter.SHOOTER_CENTER_AUTO_RPM));
    NamedCommands.registerCommand("start-shooter-layup", shooter.runAtAngularVelocity(Constants.Shooter.SHOOTER_LAYUP_RPM));
    NamedCommands.registerCommand("bump-shoot", runFullShootingSystemBump);
    NamedCommands.registerCommand("extend-intake", intakeArm.extendForAutonomous().withTimeout(0.75));
    Command intakeFuelSequence = Commands.parallel(intakeRoller.intakeFuelForAuto(), intakeArm.extendForIntakeSequenceAuto());
    Command stopIntakeFuelSequence = Commands.parallel(intakeRoller.stopIntakeWheelRotation(), intakeArm.stopExtendRetract());
    NamedCommands.registerCommand("intake-fuel", intakeFuelSequence);
    //TODO: add an .andThen(intakeArm.retract()) to the end of this?
    NamedCommands.registerCommand("stop-intake", stopIntakeFuelSequence);
    NamedCommands.registerCommand("stop-shooter", Commands.parallel(shooter.stopImmediately(), shooterFeeder.stop(), rollingFloor.stop()).withTimeout(0.2));
    NamedCommands.registerCommand("outpost-wait", Commands.waitSeconds(1.5));
    NamedCommands.registerCommand("trench-shoot", runFullShootingSystem);
    
    NamedCommands.registerCommand("layup-shoot", runFullShootingSystemLayup);
    NamedCommands.registerCommand("outpost-shoot", runFullShootingSystem);
    NamedCommands.registerCommand("shooter-low-idle", shooter.idleAtLowRpm());
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
        // Idle while the robot is disabled. This ensures the configured neutral mode is applied to the drive motors while disabled.
        final var idle = new SwerveRequest.Idle();
        RobotModeTriggers.disabled().whileTrue(drivetrain.resetState().ignoringDisable(true).andThen(drivetrain.applyRequest(() -> idle).ignoringDisable(true)));

        //don't retain old state/vision info when starting autos when in the lab
        RobotModeTriggers.teleop().onFalse(drivetrain.resetState());

        //Apply different current limits in teleop vs auto - we can allow higher limits in auto since we're not concerned about brownouts as much and we want max performance
        RobotModeTriggers.teleop().onTrue(Commands.runOnce(() -> {
          shooter.setDefaultCommand(shooter.idleAtLowRpm());
          drivetrain.applyTeleopCurrentLimits(Constants.Kinematics.DRIVE_MOTOR_SUPPLY_CURRENT_LIMIT_TELEOP);
        }));
        
        RobotModeTriggers.autonomous().onTrue(Commands.runOnce(() -> {
          System.out.println("Running Auto init resets...");
          // drivetrain.applyAutoCurrentLimits();
          visionSystem.resetTelemetry();
        }));

        Supplier<Pose2d> hubTargetPoseSupplier = () -> Constants.Game.getHubPose().toPose2d();

        //Note: we no longer do this as it zeros out the robot heading - we want to retain heading, since PathPlanner sets it at the start of auto
        //run seedFieldCentric on start of tele-op mode
        // RobotModeTriggers.teleop().onTrue(drivetrain.runOnce(drivetrain::seedFieldCentric));

        Command intakeFuelSequence = Commands.parallel(intakeRoller.intakeFuel(), intakeArm.extendForIntakeSequence());

        //TODO: we can't run the intake roller in the shoot sequence commands and also allow left dpad fullRetractCommand to run since they both want to control the intake roller
        Command fullShootingSequence = new SequentialCommandGroup(
            shooter.resetShooterAtTargetRpm(),
            Commands.parallel(
              shooter.runAtAngularVelocity(Constants.Shooter.SHOOTER_TEST_RPM),
              Commands.waitUntil(shooter.getAtRpmTrigger())
                .andThen(Commands.parallel(
                    shooterFeeder.runAtAngularVelocity(Constants.ShooterFeeder.FEEDER_RPM), 
                    intakeRoller.intakeFuel(),
                    rollingFloor.rollInwardsFast()))
            )
        );

        Command fullShootingSequenceTunableRpm = new SequentialCommandGroup(
            shooter.resetShooterAtTargetRpm(),
            Commands.parallel(
              shooter.runAtTunableAngularVelocity(),
              Commands.waitUntil(shooter.getAtRpmTrigger())
                .andThen(Commands.parallel(
                    shooterFeeder.runAtAngularVelocity(Constants.ShooterFeeder.FEEDER_RPM), 
                    intakeRoller.intakeFuel(),
                    rollingFloor.rollInwardsFast()))
            )
        );

        //TODO: test integrating this into the runFullShootingSystem command to see if it helps with agitation
        Command runIntakeRollerAlternating = Commands.sequence(intakeRoller.intakeFuel().withTimeout(0.1), intakeRoller.outtakeFuel().withTimeout(0.1));

        //TODO: Add a waitSeconds before the retractForAutoAgitate call in the sequencce if we need time to empty some balls out of the hopper first during shot
        //TODO: integrate this into the full shooting sequence and test with teletop
        Command runIntakeArmAlternating = Commands.sequence(intakeArm.retractForAutoAgitate().withTimeout(0.2), 
          Commands.waitSeconds(0.4), 
          intakeArm.extendForAutoAgitate().withTimeout(0.25),
          Commands.waitSeconds(0.6))
        .repeatedly();

        Command runFullShootingSystemInReverse = Commands.parallel(
          // shooter.runAtAngularVelocity(Constants.Shooter.SHOOTER_TEST_RPM),
          shooterFeeder.runAtAngularVelocity(Constants.ShooterFeeder.FEEDER_RPM.unaryMinus()),
          rollingFloor.rollOutwards(),
          intakeRoller.outtakeFuel()
        ).finallyDo(() -> shooter.resetShooterAtTargetRpm());

        Command runFullShootingSystemLayup = new SequentialCommandGroup(
            shooter.resetShooterAtTargetRpm(),
            Commands.parallel(
              shooter.runAtAngularVelocity(Constants.Shooter.SHOOTER_LAYUP_RPM),
              Commands.waitUntil(shooter.getAtRpmTrigger())
                .andThen(Commands.parallel(
                    shooterFeeder.runAtAngularVelocity(Constants.ShooterFeeder.FEEDER_RPM), 
                    intakeRoller.intakeFuel(),
                    rollingFloor.rollInwardsFast()
                  )
                )
            )
        );

        Command runFullShootingSystemTowerShot = new SequentialCommandGroup(
            shooter.resetShooterAtTargetRpm(),
            Commands.parallel(
              shooter.runAtAngularVelocity(Constants.Shooter.SHOOTER_TOWER_RPM),
              Commands.waitUntil(shooter.getAtRpmTrigger())
                .andThen(Commands.parallel(
                    shooterFeeder.runAtAngularVelocity(Constants.ShooterFeeder.FEEDER_RPM), 
                    intakeRoller.intakeFuel(),
                    rollingFloor.rollInwardsFast()
                  )
                )
            )
        );

        Command runFullShootingAutoRpmAndAim = new SequentialCommandGroup(
            shooter.resetShooterAtTargetRpm(),
            Commands.parallel(
              shooter.variableDistanceShot(() -> drivetrain.getShotDistance(hubTargetPoseSupplier.get().getTranslation())),
              drivetrain.teleOpDriveWithAutoAimToTarget(driverController, hubTargetPoseSupplier),
              Commands.waitUntil(shooter.getAtRpmTrigger())
                .andThen(
                  Commands.parallel(
                    shooterFeeder.runAtAngularVelocity(Constants.ShooterFeeder.FEEDER_RPM), 
                    intakeRoller.intakeFuel(),
                    rollingFloor.rollInwardsFast())
                )
            )
        );

        Command passCommand = new SequentialCommandGroup(
            shooter.resetShooterAtTargetRpm(),
            Commands.parallel(
              shooter.runAtAngularVelocity(Constants.Shooter.SHOOTER_PASS_RPM),
              Commands.waitUntil(shooter.getAtRpmTrigger())
                .andThen(shooterFeeder.runAtAngularVelocity(Constants.ShooterFeeder.FEEDER_RPM)).alongWith(rollingFloor.rollInwards())
            )
        );

        Command toggleShooterLowIdleEnabledCommand = Commands.runOnce(() -> {
          boolean currentState = shooter.isDefaultCommandIsStop();
          boolean newState = !currentState;
          shooter.setDefaultCommand(newState ? shooter.stop() : shooter.idleAtLowRpm());
          shooter.setDefaultCommandIsStop(newState);
        }, shooter);

        // Reset the field-centric heading (set robot forward direction) on left bumper press.
        driverController.leftBumper().onTrue(drivetrain.runOnce(drivetrain::seedFieldCentric));  
        driverController.leftTrigger().whileTrue(runFullShootingSystemInReverse);

        driverController.rightBumper().whileTrue(intakeFuelSequence);
        driverController.rightBumper().onFalse(Commands.parallel(intakeRoller.stopIntakeWheelRotation(), intakeArm.stopExtendRetract()));
        driverController.y().whileTrue(runFullShootingSystemTowerShot);
        driverController.rightTrigger().whileTrue(runFullShootingAutoRpmAndAim);

        driverController.x().whileTrue(intakeRoller.outtakeFuel());
        // driverController.b().whileTrue(runFullShootingSystemLayup);
        //TODO: test/confirm this behavior
        driverController.b().toggleOnTrue(runFullShootingSystemLayup);
        
        driverController.back().onTrue(toggleShooterLowIdleEnabledCommand);
        driverController.start().whileTrue(passCommand);
        
        Command fullRetractCommand = Commands.parallel(intakeArm.retractForAgitate(), intakeRoller.intakeFuel());

        driverController.povLeft().whileTrue(intakeArm.retractForAgitate());
        driverController.povRight().whileTrue(intakeArm.extend());  //press and relese to manually 'agitate'

        // Auto-align drive to hub while holding y
        // driverController.y().whileTrue(
        //   Commands.parallel(
        //     shooter.variableDistanceShot(() -> drivetrain.getShotDistance(hubTargetPoseSupplier.get().getTranslation())),
        //     drivetrain.teleOpDriveWithAutoAimToTarget(driverController, hubTargetPoseSupplier)
        //     // drivetrain.alignToTargetDrive(driverController, hubTargetPoseSupplier)
        //   )
        // );

        driverController.a().whileTrue(drivetrain.brakeAndLockWheels());

        driverController.povUp().onTrue(drivetrain.resetPoseUsingVision());
        
        //Free driver buttons = povUp, povDown, a

        //For quick RPM tuning
        operatorController.rightTrigger().whileTrue(fullShootingSequenceTunableRpm);

        //Note: operator controller is for testing new commands and running SysId tests
        // operatorController.b().whileTrue(runIntakeArmAlternating);

        operatorController.rightBumper().whileTrue(rollingFloor.rollInwards());

        //Run this to determine value for kCoupleRatio for the CTRE swerve modules by observing how much the drive motor encoder moves when we rotate the azimuth (steer motor) a known number of rotations
        // operatorController.a().onTrue(new RotateAzimuthCommand(
        //     (TalonFX) drivetrain.getModule(0).getSteerMotor(),
        //     (TalonFX) drivetrain.getModule(0).getDriveMotor(),
        //     3.0
        // ));

        operatorController.a().whileTrue(
          drivetrain.teleOpDriveWithAutoAimToTarget(operatorController, hubTargetPoseSupplier)
        );

        //Run SysId tests using the operator controller
        operatorController.back().and(operatorController.y()).whileTrue(shooter.sysIdQuasistatic(Direction.kForward));
        operatorController.back().and(operatorController.x()).whileTrue(shooter.sysIdQuasistatic(Direction.kReverse));
        operatorController.start().and(operatorController.y()).whileTrue(shooter.sysIdDynamic(Direction.kForward));
        operatorController.start().and(operatorController.x()).whileTrue(shooter.sysIdDynamic(Direction.kReverse));
        operatorController.start().and(operatorController.a()).onTrue(shooter.stopSysIdLogging());
  }

  private void setDefaultCommands() {
    drivetrain.setDefaultCommand(drivetrain.teleopDrive(driverController).withName("Teleop Drive"));
    shooterFeeder.setDefaultCommand(shooterFeeder.stop());
    shooter.setDefaultCommand(shooter.stop());
    rollingFloor.setDefaultCommand(rollingFloor.stop());
    intakeRoller.setDefaultCommand(intakeRoller.stopIntakeWheelRotation());
    intakeArm.setDefaultCommand(intakeArm.stopExtendRetract());
    // shooterHood.setDefaultCommand(shooterHood.stop());
    // climber.setDefaultCommand(climber.stopCommand());
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
    if (visionSystem != null) {
      visionSystem.periodic();
    }
  }

  public void updateTelemetry() {
    maxSpeedFactorPublisher.set(MAX_SPEED_FACTOR);
    batteryVoltagePublisher.set(RobotController.getBatteryVoltage());
    brownoutPublisher.set(RobotController.isBrownedOut());
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
