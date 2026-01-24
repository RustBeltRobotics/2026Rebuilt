// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.Optional;

import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.RobotConfig;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

  public static final class BasicUnits {
    public static final double SECONDS_PER_MINUTE = 60.0;
    public static final double DEGREES_PER_REVOLUTION = 360.0;
  }

  public static final class Controls {
    public static final int CONTROLLER_PORT_DRIVER = 0;
    public static final int CONTROLLER_PORT_OPERATOR = 1;
    public static final double CONTROLLER_DEADBAND = 0.1;
  }

  public static final class Game {
    //These are buffers to accomodate for margin of error in Vision / PoseEstimator output
    public static final double FIELD_POSE_XY_ERROR_MARGIN_METERS = Units.Inches.of(1.0).in(Units.Meters);
    public static final double FIELD_POSE_THETA_ERROR_MARGIN_RADIANS = Units.Degrees.of(2.0).in(Units.Radians);

    //These values are from pathplanner - center of hub opening at 72 inches height
    public static final Pose3d BLUE_HUB_POSE = new Pose3d(Units.Meters.of(4.618), Units.Meters.of(4.039),  Units.Inches.of(72.0), new Rotation3d());
    public static final Pose3d RED_HUB_POSE = new Pose3d(Units.Meters.of(11.914), Units.Meters.of(4.039),  Units.Inches.of(72.0), new Rotation3d());

    public static final Pose3d getHubPose() {
        Pose3d pose = edu.wpi.first.wpilibj.DriverStation.getAlliance().equals(Optional.of(Alliance.Red)) ? RED_HUB_POSE : BLUE_HUB_POSE;

        return pose;
    }

    //Note: this should be set to false for testing the the RBR Lab
    //TODO: check this is set properly before every competition!!
    public static final boolean IS_COMPETITION = true;
  }

  /**
   * CAN bus IDs
   */
  public static final class CanID {
    //TODO: these values are all from last year and are not valid - update once available for Rebuilt game
    public static final int POWER_DISTRIBUTION = 1;
    public static final int CLIMBER_MOTOR = 19;
    public static final int PIGEON_GYRO = 10;
    public static final int ELEVATOR_LEFT_MOTOR = 14;
    public static final int ELEVATOR_RIGHT_MOTOR = 16;
    public static final int ELEVATOR_EXTEND_RETRACT_MOTOR = 12;
    public static final int REJECTOR_MOTOR = 20;
    public static final int SWERVE_MODULE_FRONT_LEFT_DRIVE_MOTOR = 6;
    public static final int SWERVE_MODULE_FRONT_LEFT_STEER_MOTOR = 13;
    public static final int SWERVE_MODULE_FRONT_LEFT_STEER_ENCODER = 2;
    public static final int SWERVE_MODULE_FRONT_RIGHT_DRIVE_MOTOR = 7;
    public static final int SWERVE_MODULE_FRONT_RIGHT_STEER_MOTOR = 17;
    public static final int SWERVE_MODULE_FRONT_RIGHT_STEER_ENCODER = 3;
    public static final int SWERVE_MODULE_BACK_RIGHT_DRIVE_MOTOR = 9;
    public static final int SWERVE_MODULE_BACK_RIGHT_STEER_MOTOR = 18;
    public static final int SWERVE_MODULE_BACK_RIGHT_STEER_ENCODER = 5;
    public static final int SWERVE_MODULE_BACK_LEFT_DRIVE_MOTOR = 8;
    public static final int SWERVE_MODULE_BACK_LEFT_STEER_MOTOR = 15;
    public static final int SWERVE_MODULE_BACK_LEFT_STEER_ENCODER = 4;
  }

  /**
   * Current limits - measure in Amps
   */
  public static final class CurrentLimit {
    public static final class SparkMax {
      public static final int SMART_STEER = 40;
      public static final int SMART_ELEVATOR = 40;
      public static final int SECONDARY_STEER = 80;
      public static final int SECONDARY_ELEVATOR = 80;
    }

    public static final class Neo {
      public static final int SMART = 60;
      public static final int SECONDARY = 80;
    }
  }

  //TODO: update / verify all of these values and/or integrate with CTRE Swerve generated constants (these are all from last year)

  /**
   * Robot physical constraints (max velocity, max angular velocity, SwerveDriveKinematics, etc.)
   */
  public static final class Kinematics {

    /* Initial / max speed multiplier for drivetrain - reduce to slow driving */
    public static final double INITIAL_DRIVE_MAX_SPEED_FACTOR = 1.00;

    /* Robot mass in Kg. */
    public static final double ROBOT_MASS = Units.Pounds.of(106.0).in(Units.Kilograms); //Note: this weight does NOT include the battery or bumpers

    public static final double LOADED_MASS = Units.Pounds.of(135.0).in(Units.Kilograms); //Note: this weight includes the battery and bumpers

    public static final double MOMENT_OF_INTERIA = 6.883; //TODO: get this value from CAD from Dillan - 6.883 is the default UI value

    /* Robot frame width in meters */
    public static final double WIDTH = Units.Inches.of(27.5).in(Units.Meters);  //from 2026 CAD

    /* Robot width in meters WITH bumpers on */
    public static final double WIDTH_WITH_BUMPERS = Units.Inches.of(31.5).in(Units.Meters); //TODO: update

    /* Robot frame length in meters */
    public static final double LENGTH = Units.Inches.of(27.0).in(Units.Meters); //from 2026 CAD

    /* Robot length in meters WITH bumpers on */
    public static final double LENGTH_WITH_BUMPERS = Units.Inches.of(31.0).in(Units.Meters); //TODO: update

    /* Robot wheel diameter in meters */
    public static final double WHEEL_DIAMETER = Units.Inches.of(4.0).in(Units.Meters);;

    /* Robot wheel circumference in meters */
    public static final double WHEEL_CIRCUMFERENCE = WHEEL_DIAMETER * Math.PI;

    /* Robot wheel radius in meters */
    public static final double WHEEL_RADIUS = WHEEL_DIAMETER / 2.0;

    /* COF of wheels */
    public static final double WHEEL_COEFFECIENT_OF_FRICTION = 2.2;  //MK5N Spiky Wheels - see https://www.chiefdelphi.com/t/orbit-1690-new-custom-swerve-reveal/508927/30
    
    /**
     * The left-to-right distance between the drivetrain wheels
     * <p>
     * Should be measured from center to center
     */
    public static final double DRIVETRAIN_TRACKWIDTH_METERS = Units.Inches.of(21.75).in(Units.Meters); //from 2026 CAD
    
    /**
     * The front-to-back distance between the drivetrain wheels
     * <p>
     * Should be measured from center to center
     */
    public static final double DRIVETRAIN_WHEELBASE_METERS = Units.Inches.of(21.75).in(Units.Meters); //from 2026 CAD

    /** Distance from center of robot to center of swerve module  **/
    public static final double DRIVETRAIN_BASE_RADIUS = Math.hypot(DRIVETRAIN_TRACKWIDTH_METERS, DRIVETRAIN_WHEELBASE_METERS) / 2.0;

    //SDS Mk4i L3 gear ratios - equivalent to 6.12:1 overall ratio
    public static final double DRIVE_GEAR_RATIO = (14.0 / 50.0) * (28.0 / 16.0) * (15.0 / 45.0); //Note: (14.0 / 50.0) * (28.0 / 16.0) * (15.0 / 45.0) == 1.0/6.12 = 0.1634

    public static final double STEER_GEAR_RATIO = 1.0 / (150.0 / 7.0); //150/7:1 gear ratio

    //TODO: Figure out why true is needed for correct wheel direction, when in theory it should be false
    public static final boolean DRIVE_MOTOR_INVERTED = true; //should be false for Mk4i, true for Mk4 (tested on Eclipse robot)

    public static final boolean STEER_MOTOR_INVERTED = true; //should be true for Mk4i, false for Mk4 (tested on Eclipse robot)

    /** Conversion between motor rotations and drive meters */
    public static final double DRIVE_POSITION_CONVERSION = WHEEL_CIRCUMFERENCE * DRIVE_GEAR_RATIO;
        
    /** Conversion between motor rotations per second and drive meters per second */
    // public static final double DRIVE_VELOCITY_CONVERSION = DRIVE_POSITION_CONVERSION / BasicUnits.SECONDS_PER_MINUTE;
    //Note: TalonFX reports velocity in RPS, not RPM - so no need to divide by 60
    //TODO: verify this exhibits the proper wheel velocity when testing drive
    public static final double DRIVE_VELOCITY_CONVERSION = DRIVE_POSITION_CONVERSION;

    /** Conversion between motor rotations and steer degrees */
    public static final double STEER_POSITION_CONVERSION = BasicUnits.DEGREES_PER_REVOLUTION * STEER_GEAR_RATIO;

    /** Conversion between motor rotations per minute and steer degrees per seconds */
    public static final double STEER_VELOCITY_CONVERSION = STEER_POSITION_CONVERSION / BasicUnits.SECONDS_PER_MINUTE;

    /**
     * The maximum linear velocity in meters per second. Calculate using https://www.reca.lc/drive
     */
    // public static final double MAX_VELOCITY_METERS_PER_SECOND = 5.29;  //TODO: compare to empirical measurement
    //TODO: run the robot at full speed and measure actual speed to verify this value,
    //TODO: update settings in PathPlanner GUI to match if this value is changed
    public static final double MAX_VELOCITY_METERS_PER_SECOND = TunerConstants.kSpeedAt12Volts.in(Units.MetersPerSecond);  //TODO: update CTRE Swerve generated constant value to match theoretical above

    /**
     * The maximum angular velocity of the robot in radians per second. This is a
     * measure of how fast the robot can rotate in place.
     * 5.29 M/Sec / 0.39 M = 13.5 Rad / Sec = 773.49 degrees / sec
     */
    public static final double MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND = MAX_VELOCITY_METERS_PER_SECOND / DRIVETRAIN_BASE_RADIUS; //TODO: compare to empirical measurement (calc in comment seems high)

    /**
     * Used  to convert desired chassis velocity into individual swerve smodule states
     * See https://docs.wpilib.org/en/stable/docs/software/basic-programming/coordinate-system.html#robot-drive-kinematics
     */
    public static final SwerveDriveKinematics SWERVE_KINEMATICS = new SwerveDriveKinematics(
            new Translation2d(DRIVETRAIN_TRACKWIDTH_METERS / 2.0, DRIVETRAIN_WHEELBASE_METERS / 2.0),    //Front Left
            new Translation2d(DRIVETRAIN_TRACKWIDTH_METERS / 2.0, -DRIVETRAIN_WHEELBASE_METERS / 2.0),   //Front Right
            new Translation2d(-DRIVETRAIN_TRACKWIDTH_METERS / 2.0, DRIVETRAIN_WHEELBASE_METERS / 2.0),   //Back Left
            new Translation2d(-DRIVETRAIN_TRACKWIDTH_METERS / 2.0, -DRIVETRAIN_WHEELBASE_METERS / 2.0)); //Back Right

    public static final double TIP_THRESHOLD_DEGREES = 7.0;

    /* Change in linear acceleration greater than this value will trigger collision detected */
    public static final double COLLISION_THRESHOLD_DELTA_G_TELE_OP = 1.6;  //TODO: verify this value for tele-op using advantagescope gyro graph
    public static final double COLLISION_THRESHOLD_DELTA_G_AUTONOMOUS = 1.4;
    /* Pose estimate should not be reset until after this long after collision */
    public static final long MICROSECONDS_SINCE_COLLISION_THRESHOLD = 250000;  //0.25 seconds

    public static final Angle EPSILON_ANGLE_TO_GOAL = Units.Degrees.of(1.0);

    public static final PIDController ROTATE_TO_POSE_PID_CONTROLLER = getRotateToPoseController();

    private static final PIDController getRotateToPoseController() {
      //TODO: test/tune PID
        PIDController controller = new PIDController(1.5, 0.0, 0.0);
        controller.enableContinuousInput(-Math.PI, Math.PI);

        return controller;
    }

    // public static final Matrix<N3, N1> WHEEL_ODOMETRY_POSE_STANDARD_DEVIATIONS = VecBuilder.fill(0.05, 0.05, Units.degreesToRadians(5));
    public static final Matrix<N3, N1> WHEEL_ODOMETRY_POSE_STANDARD_DEVIATIONS = VecBuilder.fill(0.1, 0.1, Units.Degrees.of(5.0).in(Units.Radians));
  }

  public static final class PathPlanner {
    //TODO: update the driveGearing value here to match the CTRE Swerve gearing for MK5n modules once available
    public static final ModuleConfig MODULE_CONFIG = new ModuleConfig(Kinematics.WHEEL_RADIUS, Kinematics.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND,
      Kinematics.WHEEL_COEFFECIENT_OF_FRICTION, DCMotor.getKrakenX60(1), 6.12, 60.0, 4);
    public static final RobotConfig ROBOT_CONFIG = new RobotConfig(Kinematics.LOADED_MASS, Kinematics.MOMENT_OF_INTERIA,
      MODULE_CONFIG, Kinematics.SWERVE_KINEMATICS.getModules());
      
    //TODO: tune this - max xy error seen roughly 1 - 0.33
    // public static final double rotation_P = 1.5;
    public static final double rotation_P = 2.0;
    public static final double rotation_I = 0.0;
    public static final double rotation_D = 0.0;

    //TODO: tune this - max xy error seen roughly 1 - 0.33
    // public static final double translation_P = 1.0;
    public static final double translation_P = 2.5;
    public static final double translation_I = 0.0;
    public static final double translation_D = 0.0;
  }

  public static final class Vision {
    public static final boolean VISION_ENABLED = true;
    public static final int APRIL_TAG_PIPELINE_INDEX = 0;
    public static final String ARDUCAM_MODEL = "OV9281";
    public static final double POSE_AMBIGUITY_CUTOFF = 0.2;  //https://docs.photonvision.org/en/latest/docs/apriltag-pipelines/3D-tracking.html#ambiguity
    public static final double POSE_AMBIGUITY_SHIFTER = 0.2;
    public static final double POSE_AMBIGUITY_MULTIPLIER = 4.0;
    public static final double NOISY_DISTANCE_METERS = 2.5;  //distance beyond which vision measurements are noisy
    public static final double DISTANCE_CUTOFF = 3.0;  //Tag readings beyond this distance (in meters) will be considered invalid
    public static final double DISTANCE_WEIGHT = 7.0;
    public static final int TAG_PRESENCE_WEIGHT = 10;
  

    /**
     * Standard deviations for vision measurements. Increase these numbers to trust your
     * vision measurements less. This matrix is in the form [x, y, theta]ᵀ, with units in meters and radians.
     * Note that the SwerveDrivePoseEstimator default is 0.9 for all values for vision measurements.
     * 
     * See also https://www.chiefdelphi.com/t/how-do-i-understand-standard-deviation-in-the-poseestimator-class/411492/10?u=mrokitka
     */
    public static final Matrix<N3, N1> DEFAULT_VISION_MEASUREMENT_STANDARD_DEVIATIONS = VecBuilder.fill(0.9, 0.9, 0.9);
    public static final Matrix<N3, N1> SINGLE_TAG_MEASUREMENT_STANDARD_DEVIATIONS = VecBuilder.fill(0.8, 0.8, Units.Degrees.of(30.0).in(Units.Radians));
    public static final Matrix<N3, N1> FRONT_CAMERA_STANDARD_DEVIATIONS = VecBuilder.fill(0.85, 0.85, Units.Degrees.of(20.0).in(Units.Radians));
    public static final Matrix<N3, N1> OTHER_CAMERA_STANDARD_DEVIATIONS = VecBuilder.fill(1.2, 1.2, Units.Degrees.of(30.0).in(Units.Radians));
    public static final double MULTI_TAG_XY_PER_TAG_STANDARD_DEVIATION = 0.3;
    public static final double MULTI_TAG_THETA_STANDARD_DEVIATION = Units.Degrees.of(20.0).in(Units.Radians);

    /**
     * Unique camera names, usable in PhotonCamera instances
     */
    public static final class CameraName {
      //TODO: update these constants for this year, these values are from last years game
      //Note: these names are set in hardware via https://docs.arducam.com/UVC-Camera/Serial-Number-Tool-Guide/
      public static final String FRONT_CENTER = "Arducam_OV9281_USB_Camera-4";
      public static final String BACK_RIGHT = "Arducam_OV9281_USB_Camera-3";
      public static final String BACK_LEFT = "Arducam_OV9281_USB_Camera-1";
    }

    /**
     * Mounting position of the cameras on the Robot
     */
    public static final class CameraPose {
      //Note: these are robot to camera poses (position from center of robot to camera lens) - see also edu.wpi.first.math.ComputerVisionUtil.objectToRobotPose()
      //In transform3d - Translation3d values: x+ = forward, y+ = left, z+ = up, Rotation3d is rotation around the transform3d axes
      // https://docs.wpilib.org/en/stable/docs/software/basic-programming/coordinate-system.html
      // https://vis-ro.web.app/robotics/eulerangles
      // https://www.chiefdelphi.com/t/photonvision-setup-specifically-the-robottocamera-transform/459246/2
      //Note: these values can be visualized in AdvantageScope if published over NetworkTables - https://github.com/Mechanical-Advantage/AdvantageScope/blob/main/docs/tabs/3D-FIELD.md
      //example - https://github.com/Mechanical-Advantage/RobotCode2024/blob/main/src/main/java/org/littletonrobotics/frc2024/subsystems/apriltagvision/AprilTagVisionConstants.java#L30
      //x+, y+, z+, (0, -degrees, 0).rotateBy(0, 0, 45 degrees)

      //TODO: these values are all from last year - need to update for new positions this year

      public static final Transform3d FRONT_CENTER = new Transform3d(Units.Inches.of(14.0).in(Units.Meters), 0.0, Units.Inches.of(11.5).in(Units.Meters), 
        new Rotation3d(0, 0, 0));  //front center - photonvision1
      //x+, y-, z+, (0, -degrees, 0).rotateBy(0, 0, -45 degrees)
      // public static final Transform3d FRONT_RIGHT = new Transform3d(CAM_XY_FROM_CENTER_OF_ROBOT, -CAM_XY_FROM_CENTER_OF_ROBOT, CAM_Z_FROM_FLOOR, 
      //   new Rotation3d(0, CAM_PITCH_ANGLE, 0).rotateBy(new Rotation3d(0, 0, -Units.degreesToRadians(45))));  //front right - photonvision2
      //x-, y-, z+, (0, -degrees, 0).rotateBy(0, 0, -135 degrees)
      public static final Transform3d BACK_RIGHT = new Transform3d(Units.Inches.of(9.5).in(Units.Meters), -Units.Inches.of(9.0).in(Units.Meters), Units.Inches.of(11.0).in(Units.Meters), 
        new Rotation3d(0, 0, 0).rotateBy(new Rotation3d(0, 0, -Units.Degrees.of(90.0).in(Units.Radians))));  //back right - photonvision1
      //x-, y+, z+, (0, -degrees, 0).rotateBy(0, 0, 135 degrees)
      public static final Transform3d BACK_LEFT = new Transform3d(Units.Inches.of(9.5).in(Units.Meters), Units.Inches.of(9.0).in(Units.Meters), Units.Inches.of(11.0).in(Units.Meters),
        new Rotation3d(0, 0, 0).rotateBy(new Rotation3d(0, 0, Units.Degrees.of(90.0).in(Units.Radians))));  //back left - photonvision2
    }
  }

  public static final class Shuffleboard {
    public static final ShuffleboardTab COMPETITION_TAB = edu.wpi.first.wpilibj.shuffleboard.Shuffleboard.getTab("Competition");
    public static final ShuffleboardTab DIAG_TAB = edu.wpi.first.wpilibj.shuffleboard.Shuffleboard.getTab("Diagnostics");
  }
}
