package frc.robot.subsystems;

import static edu.wpi.first.units.Units.*;

import java.util.function.Supplier;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.SparkBase.ControlType;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.MutAngle;
import edu.wpi.first.units.measure.MutAngularVelocity;
import edu.wpi.first.units.measure.MutDistance;
import edu.wpi.first.units.measure.MutLinearVelocity;
import edu.wpi.first.units.measure.MutVoltage;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.DutyCycle;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants;

public class Shooter extends SubsystemBase {

    //TODO: set one kraken and one vortex as the primary motors, and let the others follow them
    private final TalonFX krakenLeft = new TalonFX(Constants.CanID.SHOOTER_KRAKEN_LEFT, CANBus.roboRIO());
    private final TalonFX krakenRight = new TalonFX(Constants.CanID.SHOOTER_KRAKEN_RIGHT, CANBus.roboRIO());
    private final SparkMax vortexLeft = new SparkMax(Constants.CanID.SHOOTER_VORTEX_LEFT, MotorType.kBrushless);
    private final RelativeEncoder leftVortexEncoder = vortexLeft.getEncoder();
    private SparkClosedLoopController leftVortexPidController = vortexLeft.getClosedLoopController();
    
    //TODO: run the vortex motors in duty cycle mode for simplicity??
    private final SparkMax vortexRight = new SparkMax(Constants.CanID.SHOOTER_VORTEX_RIGHT, MotorType.kBrushless);
    private final RelativeEncoder rightVortexEncoder = vortexRight.getEncoder();

    private final VelocityVoltage velocityRequest = new VelocityVoltage(0);  //Velocity control request for Krakens
    private final DutyCycleOut dutyCycleRequest = new DutyCycleOut(0); //Duty cycle control request for Krakens

    private final InterpolatingDoubleTreeMap rpmTable = new InterpolatingDoubleTreeMap();

    private final VoltageOut ctreSysIdControl = new VoltageOut(0);
    // Mutable holder for unit-safe voltage values, persisted to avoid reallocation.
    protected final MutVoltage sysIdAppliedVoltage = Volts.mutable(0);
    // Mutable holder for unit-safe linear distance values, persisted to avoid reallocation.
    protected final MutDistance sysIdDistance = Meters.mutable(0);

    private final MutAngle sysIdPosition = Rotations.mutable(0);
    private final MutAngularVelocity sysIdVelocity = RotationsPerSecond.mutable(0);

    private final SysIdRoutine ctreSysIdRoutine = new SysIdRoutine(
        new SysIdRoutine.Config(
            null,         // Use default ramp rate (1 V/s)
            Volts.of(4), // Reduce dynamic voltage to 4 to prevent brownout
            null,          // Use default timeout (10 s)
                                // Log state with Phoenix SignalLogger class
            state -> SignalLogger.writeString("state", state.toString())
        ),
        new SysIdRoutine.Mechanism(
            volts -> {
                krakenLeft.setControl(ctreSysIdControl.withOutput(volts));
                krakenRight.setControl(ctreSysIdControl.withOutput(volts));
            },
            null,
            this
        )
    );

    private final SysIdRoutine revSysIdRoutine = new SysIdRoutine(
         new SysIdRoutine.Config(
            null,        // Use default ramp rate (1 V/s)
            Volts.of(4), // Reduce dynamic step voltage to 4 to prevent brownout
            null,        // Use default timeout (10 s)s
            null
        ),
        new SysIdRoutine.Mechanism(
            // Tell SysId how to plumb the driving voltage to the motors.
            (Voltage volts) -> {
                vortexLeft.setVoltage(volts.in(Volts));
                vortexRight.setVoltage(volts.in(Volts));
            },
            // Tell SysId how to record a frame of data for each motor on the mechanism being
            // characterized.
            log -> {
                // Record a frame for the motors.
                log.motor("shooter-vortex-left")
                    .voltage(sysIdAppliedVoltage.mut_replace(vortexLeft.getAppliedOutput() * vortexLeft.getBusVoltage(), Volts))
                    .angularPosition(sysIdPosition.mut_replace(leftVortexEncoder.getPosition(), Rotations))
                    .angularVelocity(sysIdVelocity.mut_replace(leftVortexEncoder.getVelocity() / 60.0, RotationsPerSecond));  //divide by 60 to convert RPM to RPS
                log.motor("shooter-vortex-right")
                    .voltage(sysIdAppliedVoltage.mut_replace(vortexRight.getAppliedOutput() * vortexRight.getBusVoltage(), Volts))
                    .angularPosition(sysIdPosition.mut_replace(rightVortexEncoder.getPosition(), Rotations))
                    .angularVelocity(sysIdVelocity.mut_replace(rightVortexEncoder.getVelocity() / 60.0, RotationsPerSecond));
            },
            // Tell SysId to make generated commands require this subsystem, suffix test state in
            // WPILog with this subsystem's name
            this)
    );

    private final SysIdRoutine unifiedSysIdRoutine = new SysIdRoutine(
        new SysIdRoutine.Config(
            null,         // Use default ramp rate (1 V/s)
            Volts.of(4), // Reduce dynamic voltage to 4 to prevent brownout
            null,          // Use default timeout (10 s)
                                // Log state with Phoenix SignalLogger class
            state -> SignalLogger.writeString("state", state.toString())
        ),
        new SysIdRoutine.Mechanism(
            volts -> {
                krakenLeft.setControl(ctreSysIdControl.withOutput(volts));
                krakenRight.setControl(ctreSysIdControl.withOutput(volts));
                vortexLeft.setVoltage(volts.in(Volts));
                vortexRight.setVoltage(volts.in(Volts));
            },
            log -> {
                // Record a frame for the motors.
                log.motor("shooter-kraken-left")
                    .voltage(sysIdAppliedVoltage.mut_replace(krakenLeft.getMotorVoltage().getValueAsDouble(), Volts))
                    .angularPosition(sysIdPosition.mut_replace(krakenLeft.getRotorPosition().getValueAsDouble(), Rotations))
                    .angularVelocity(sysIdVelocity.mut_replace(krakenLeft.getRotorVelocity().getValueAsDouble(), RotationsPerSecond));
                log.motor("shooter-kraken-right")
                    .voltage(sysIdAppliedVoltage.mut_replace(krakenRight.getMotorVoltage().getValueAsDouble(), Volts))
                    .angularPosition(sysIdPosition.mut_replace(krakenRight.getRotorPosition().getValueAsDouble(), Rotations))
                    .angularVelocity(sysIdVelocity.mut_replace(krakenRight.getRotorVelocity().getValueAsDouble(), RotationsPerSecond));
                log.motor("shooter-vortex-left")
                    .voltage(sysIdAppliedVoltage.mut_replace(vortexLeft.getAppliedOutput() * vortexLeft.getBusVoltage(), Volts))
                    .angularPosition(sysIdPosition.mut_replace(leftVortexEncoder.getPosition(), Rotations))
                    .angularVelocity(sysIdVelocity.mut_replace(leftVortexEncoder.getVelocity() / 60.0, RotationsPerSecond));  //divide by 60 to convert RPM to RPS
                log.motor("shooter-vortex-right")
                    .voltage(sysIdAppliedVoltage.mut_replace(vortexRight.getAppliedOutput() * vortexRight.getBusVoltage(), Volts))
                    .angularPosition(sysIdPosition.mut_replace(rightVortexEncoder.getPosition(), Rotations))
                    .angularVelocity(sysIdVelocity.mut_replace(rightVortexEncoder.getVelocity() / 60.0, RotationsPerSecond));
            },
            this
        )
    );

    //change this to switch between CTRE and REV SysId routines
    private final SysIdRoutine effectiveSysIdRoutine = unifiedSysIdRoutine;
    public Shooter() {
        //TODO: test and assign these values properly - those listed are placeholders
        rpmTable.put(2.0, 1500.0); // At 2m, 1500 RPM
        rpmTable.put(4.0, 2500.0); // At 4m, 2500 RPM
        rpmTable.put(6.0, 3000.0); // At 6m, 3000 RPM

        // PID + feedforward tuning for velocity control of the shooter wheel
        //TODO: test and assign these values properly - those listed are placeholders
        Slot0Configs slot0 = new Slot0Configs();
        slot0.kP = Constants.Shooter.CtrePidf.K_P;
        slot0.kI = Constants.Shooter.CtrePidf.K_I;
        slot0.kD = Constants.Shooter.CtrePidf.K_D;
        slot0.kV = Constants.Shooter.CtrePidf.K_V;
        slot0.kS = Constants.Shooter.CtrePidf.K_S;

        TalonFXConfiguration cfg = new TalonFXConfiguration();
        cfg.Slot0 = slot0;

        krakenLeft.getConfigurator().apply(cfg);

        SparkMaxConfig vortexConfig = new SparkMaxConfig();
        vortexConfig.voltageCompensation(12.0).idleMode(IdleMode.kCoast);
        vortexConfig.closedLoop.pid(Constants.Shooter.RevPidf.K_P, Constants.Shooter.RevPidf.K_I, Constants.Shooter.RevPidf.K_D);
        vortexConfig.closedLoop.feedForward.kS(Constants.Shooter.RevPidf.K_S).kV(Constants.Shooter.RevPidf.K_V);
        vortexLeft.configure(vortexConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        //TODO: setup follower motors and inverse settings as appropriate
    }

    public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
        return effectiveSysIdRoutine.quasistatic(direction);
    }

    public Command sysIdDynamic(SysIdRoutine.Direction direction) {
        return effectiveSysIdRoutine.dynamic(direction);
    }

    private double getTargetShooterRPMForDistance(Distance distance) {
        return rpmTable.get(distance.in(Meters));
    }

    private void runShooterDutyCycle(double dutyCycle) {
        krakenLeft.setControl(dutyCycleRequest.withOutput(dutyCycle));
        vortexLeft.set(dutyCycle);
    }

    private void runShooterAtRPM(double rpm) {
        double rps = rpm / 60.0;
        krakenLeft.setControl(velocityRequest.withVelocity(rps));
        leftVortexPidController.setSetpoint(rpm, ControlType.kVelocity);
    }

    public Command prepVariableDistanceShot(Supplier<Distance> shotDistanceSupplier) {
        return startEnd(() -> runShooterAtRPM(getTargetShooterRPMForDistance(shotDistanceSupplier.get())), () -> runShooterAtRPM(0.0));
    }

    //See https://blog.eeshwark.com/robotblog/shooting-on-the-fly
    public Command autoShoot(Supplier<Pose2d> robotPoseSupplier, Supplier<ChassisSpeeds> robotSpeedSupplier, Supplier<Distance> shotDistanceSupplier) {
        Distance targetDistance = shotDistanceSupplier.get();
        // 1. LATENCY COMP
        double latency = 0.15; // Tuned constant
        Pose2d robotPose = robotPoseSupplier.get();
        ChassisSpeeds robotSpeed = robotSpeedSupplier.get();
        Translation2d futurePos = robotPose.getTranslation().plus(
            new Translation2d(robotSpeed.vxMetersPerSecond, robotSpeed.vyMetersPerSecond).times(latency)
        );

        // 2. GET TARGET VECTOR
        Translation2d goalLocation = Constants.Game.getHubPose().getTranslation().toTranslation2d();
        Translation2d targetVec = goalLocation.minus(futurePos);
        double dist = targetVec.getNorm();

        // 3. CALCULATE IDEAL SHOT (Stationary)
        // Note: This returns HORIZONTAL velocity component
        
        double idealHorizontalSpeed = getTargetShooterRPMForDistance(targetDistance);

        // 4. VECTOR SUBTRACTION
        Translation2d robotVelVec = new Translation2d(robotSpeed.vxMetersPerSecond, robotSpeed.vyMetersPerSecond);
        Translation2d shotVec = targetVec.div(dist).times(idealHorizontalSpeed).minus(robotVelVec);

        // 5. CONVERT TO CONTROLS
        double turretAngleInDegrees = shotVec.getAngle().getDegrees();
        double newHorizontalSpeed = shotVec.getNorm();

        // 6. SOLVE FOR NEW PITCH/RPM
        // Assuming constant total exit velocity, variable hood:
        double totalExitVelocity = 15.0; // m/s
        // Clamp to avoid domain errors if we need more speed than possible
        double ratio = Math.min(newHorizontalSpeed / totalExitVelocity, 1.0);
        double newPitch = Math.acos(ratio);

        // 7. SET OUTPUTS

        //TODO: refactor this to match our actual mechanisms

        // turret.setAngle(turretAngleInDegrees);
        // hood.setAngle(Math.toDegrees(newPitch));
        // shooter.setRPM(calcRPM(totalExitVelocity));

        //TODO: wrap this in an actual command
        return Commands.none();
    }

    private double getShooterRPM() {
        //TODO: change motorRotationsPerWheelRotation if we add gearing between the motor and the wheel
        // (ex: 2:1 reduction => 2.0 motorRotationsPerWheelRotation) 
        double motorRotationsPerWheelRotation = 1.0; //Note: this currently assumes direct drive from motor to wheel
        double rotorRps = krakenLeft.getRotorVelocity().getValueAsDouble(); // rotations/sec 
        double wheelRps = rotorRps / motorRotationsPerWheelRotation;

        return wheelRps * 60.0;  //RPS to RPM
    }

    /*
    possible YAMS based implementation:

    // Upper motor group (2 Krakens)
var upperMotorModel = edu.wpi.first.math.system.plant.DCMotor.getKrakenX60(2);
// wrap your CTRE controllers into a SmartMotorController here...

FlyWheel upper = new FlyWheel(
  new FlyWheelConfig(upperSmc)
    .withDiameter(Inches.of(4))
    .withMass(Pounds.of(1))
);

// Lower motor group (2 Vortex)
var lowerMotorModel = edu.wpi.first.math.system.plant.DCMotor.getNeoVortex(2);
// wrap your REV controllers into a SmartMotorController here...

FlyWheel lower = new FlyWheel(
  new FlyWheelConfig(lowerSmc)
    .withDiameter(Inches.of(4))
    .withMass(Pounds.of(1))
);

public void setUpperSpeed(AngularVelocity upperTarget) {
  // belt ratio: upperOverLower = ω_upper / ω_lower
  AngularVelocity lowerTarget = upperTarget.div(upperOverLower);

  upper.setMechanismVelocitySetpoint(upperTarget);
  lower.setMechanismVelocitySetpoint(lowerTarget);
}

@Override
public void simulationPeriodic() {
  upper.simIterate();
  lower.simIterate();
}

     */
}
