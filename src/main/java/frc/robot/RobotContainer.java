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
      visionSystem = new VisionSystem(drivetrain::consumeVisionPoseEstimate, drivetrain.getCurrentPoseSupplier());
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

    Command runFullShootingSystem = Commands.parallel(
          shooterFeeder.runAtAngularVelocity(Constants.ShooterFeeder.FEEDER_RPM),
          rollingFloor.rollInwards(),
          Commands.sequence(Commands.waitSeconds(2.5), intakeArm.retract().withTimeout(0.5))  //TODO: graph and tune this delay based on time for shooter to get up to speed (adjust if we leave the shooter running at low RPM idle between shots)
        ).withTimeout(4.0);  //TODO: We may want to lower the timeout here, evaluate after testing

    NamedCommands.registerCommand("start-shooter", shooter.runAtAngularVelocity(Constants.Shooter.SHOOTER_TRENCH_RPM));
    NamedCommands.registerCommand("extend-intake", intakeArm.extend().withTimeout(0.75));
    Command intakeFuelSequence = Commands.parallel(intakeRoller.intakeFuelForAuto(), intakeArm.extendForIntakeSequenceAuto());
    Command stopIntakeFuelSequence = Commands.parallel(intakeRoller.stopIntakeWheelRotation(), intakeArm.stopExtendRetract());
    NamedCommands.registerCommand("intake-fuel", intakeFuelSequence);
    //TODO: add an .andThen(intakeArm.retract()) to the end of this?
    NamedCommands.registerCommand("stop-intake", stopIntakeFuelSequence);
    NamedCommands.registerCommand("outpost-wait", Commands.waitSeconds(1.5));
    NamedCommands.registerCommand("trench-shoot", runFullShootingSystem);
    Command runFullShootingSystemLayup = Commands.parallel(
      shooter.runAtAngularVelocity(Constants.Shooter.SHOOTER_LAYUP_RPM),
      Commands.sequence(Commands.waitSeconds(0.9), shooterFeeder.runAtAngularVelocity(Constants.ShooterFeeder.FEEDER_RPM)),
      Commands.sequence(Commands.waitSeconds(0.9), rollingFloor.rollInwards())
    ).withTimeout(3.0);
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
        RobotModeTriggers.disabled().whileTrue(drivetrain.applyRequest(() -> idle).ignoringDisable(true));

        Supplier<Pose2d> hubTargetPoseSupplier = () -> Constants.Game.getHubPose().toPose2d();

        //run seedFieldCentric on start of tele-op mode
        // RobotModeTriggers.teleop().onTrue(drivetrain.runOnce(drivetrain::seedFieldCentric));

        Command intakeFuelSequence = Commands.parallel(intakeRoller.intakeFuel(), intakeArm.extendForIntakeSequence());

        Command fullShootingSequence = new SequentialCommandGroup(
            shooter.resetShooterAtTargetRpm(),
            Commands.parallel(
              shooter.runAtAngularVelocity(Constants.Shooter.SHOOTER_TEST_RPM),
              Commands.waitUntil(shooter.getAtRpmTrigger())
                .andThen(Commands.parallel(shooterFeeder.runAtAngularVelocity(Constants.ShooterFeeder.FEEDER_RPM), rollingFloor.rollInwards()))
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
          shooter.runAtAngularVelocity(Constants.Shooter.SHOOTER_TEST_RPM),
          shooterFeeder.runAtAngularVelocity(Constants.ShooterFeeder.FEEDER_RPM.unaryMinus()),
          rollingFloor.rollOutwards()
        ).finallyDo(() -> shooter.resetShooterAtTargetRpm());

        Command runFullShootingSystemLayup = new SequentialCommandGroup(
            shooter.resetShooterAtTargetRpm(),
            Commands.parallel(
              shooter.runAtAngularVelocity(Constants.Shooter.SHOOTER_LAYUP_RPM),
              Commands.waitUntil(shooter.getAtRpmTrigger())
                .andThen(Commands.parallel(shooterFeeder.runAtAngularVelocity(Constants.ShooterFeeder.FEEDER_RPM), rollingFloor.rollInwards()))
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
        driverController.rightTrigger().whileTrue(fullShootingSequence);

        driverController.x().whileTrue(intakeRoller.outtakeFuel());
        driverController.b().whileTrue(runFullShootingSystemLayup); 
        
        driverController.back().onTrue(toggleShooterLowIdleEnabledCommand);
        driverController.start().whileTrue(passCommand);
        
        Command fullRetractCommand = Commands.parallel(intakeArm.retractForAgitate(), intakeRoller.intakeFuel());

        driverController.povLeft().whileTrue(fullRetractCommand);
        driverController.povRight().whileTrue(intakeArm.extend());  //press and relese to manually 'agitate'

        // Auto-align drive to hub while holding y
        driverController.y().whileTrue(
          Commands.parallel(
            shooter.prepVariableDistanceShot(() -> drivetrain.getShotDistance(hubTargetPoseSupplier.get().getTranslation())),
            drivetrain.teleOpDriveWithAutoAimToTarget(driverController, hubTargetPoseSupplier)
            // drivetrain.alignToTargetDrive(driverController, hubTargetPoseSupplier)
          )
        );
        
        //Free driver buttons = povUp, povDown, a


        //Note: operator controller is for testing new commands and running SysId tests
        operatorController.b().whileTrue(runIntakeArmAlternating);

        //Run SysId tests using the operator controller
        operatorController.back().and(operatorController.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        operatorController.back().and(operatorController.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));
        operatorController.start().and(operatorController.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        operatorController.start().and(operatorController.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        operatorController.start().and(operatorController.a()).onTrue(drivetrain.stopSysIdLogging());
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
