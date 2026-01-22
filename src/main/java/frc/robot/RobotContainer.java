// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathfindingCommand;

import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.drive.Drivetrain;
import frc.robot.util.SwerveTelemetryCTRE;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  //TODO: add subsystems

   // For limiting maximum speed (1.0 = 100% = full speed)
  private static double MAX_SPEED_FACTOR = Constants.Kinematics.INITIAL_DRIVE_MAX_SPEED_FACTOR;

  private final CommandXboxController driverController = new CommandXboxController(Constants.DriverStation.CONTROLLER_PORT_DRIVER);
  private final CommandXboxController operatorController = new CommandXboxController(Constants.DriverStation.CONTROLLER_PORT_OPERATOR);

  private final SwerveTelemetryCTRE swerveTelemetryCTRE = new SwerveTelemetryCTRE(MAX_SPEED_FACTOR * Constants.Kinematics.MAX_VELOCITY_METERS_PER_SECOND);
  private final Drivetrain drivetrain = new Drivetrain();

  private final SendableChooser<Command> autoChooser;
  private final SendableChooser<Double> driveTrainSpeedChooser = new SendableChooser<>();
  private DoublePublisher maxSpeedFactorPublisher = NetworkTableInstance.getDefault().getDoubleTopic("/RBR/MaxSpeed").publish();

  public static double getMaxSpeed() {
    return MAX_SPEED_FACTOR * Constants.Kinematics.MAX_VELOCITY_METERS_PER_SECOND;
  }

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
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

    setDefaultCommands();
    configureBindings();
    configureAutos();

    // Speed up initial run of Pathplanner commands
    PathfindingCommand.warmupCommand().schedule();
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
    //TODO: define controller bindings
  }

  private void setDefaultCommands() {
    //TODO: register default commands for subsystems
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

  public void updateTelemetry() {
    maxSpeedFactorPublisher.set(MAX_SPEED_FACTOR);
  }

  public static void setMaxSpeedFactor(double newSpeedFactor) {
    MAX_SPEED_FACTOR = newSpeedFactor;
  }
}
