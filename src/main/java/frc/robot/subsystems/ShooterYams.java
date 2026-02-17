package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.MutAngle;
import edu.wpi.first.units.measure.MutAngularVelocity;
import edu.wpi.first.units.measure.MutVoltage;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants;
import yams.gearing.MechanismGearing;
import yams.mechanisms.config.FlyWheelConfig;
import yams.mechanisms.velocity.FlyWheel;
import yams.motorcontrollers.SmartMotorController;
import yams.motorcontrollers.SmartMotorControllerConfig;
import yams.motorcontrollers.SmartMotorControllerConfig.ControlMode;
import yams.motorcontrollers.SmartMotorControllerConfig.MotorMode;
import yams.motorcontrollers.SmartMotorControllerConfig.TelemetryVerbosity;
import yams.motorcontrollers.local.SparkWrapper;
import yams.motorcontrollers.remote.TalonFXWrapper;

public class ShooterYams extends SubsystemBase {

    private final TalonFX shooterKrakenLeft = new TalonFX(Constants.CanID.SHOOTER_KRAKEN_LEFT, CANBus.roboRIO()); //follower
    private final TalonFX shooterKrakenRight = new TalonFX(Constants.CanID.SHOOTER_KRAKEN_RIGHT, CANBus.roboRIO());  //primary
    private final SparkFlex shooterVortexLeft = new SparkFlex(Constants.CanID.SHOOTER_VORTEX_LEFT, MotorType.kBrushless);  //follower - negative rotation spins in the direction we want
    private final RelativeEncoder leftVortexEncoder = shooterVortexLeft.getEncoder();
    private final SparkMax shooterVortexRight = new SparkMax(Constants.CanID.SHOOTER_VORTEX_RIGHT, MotorType.kBrushless);  //primary - positive rotation spins in the direction we want
    private final RelativeEncoder rightVortexEncoder = shooterVortexRight.getEncoder();

    private final SmartMotorControllerConfig shooterKrakenConfig = new SmartMotorControllerConfig(this)
            .withControlMode(ControlMode.CLOSED_LOOP)
            .withFollowers(Pair.of(shooterKrakenLeft, true))
            // Feedback Constants (PID Constants)
            .withClosedLoopController(Constants.Shooter.CtrePidf.K_P, Constants.Shooter.CtrePidf.K_I,
                    Constants.Shooter.CtrePidf.K_D, Units.RadiansPerSecond.of(524.0),
                    Units.RadiansPerSecondPerSecond.of(1400.0))
            .withSimClosedLoopController(Constants.Shooter.CtrePidf.K_P, Constants.Shooter.CtrePidf.K_I,
                    Constants.Shooter.CtrePidf.K_D, Units.RadiansPerSecond.of(524.0),
                    Units.RadiansPerSecondPerSecond.of(1400.0))
            // Feedforward Constants
            .withFeedforward(new SimpleMotorFeedforward(Constants.Shooter.CtrePidf.K_S, Constants.Shooter.CtrePidf.K_V, Constants.Shooter.CtrePidf.K_A))
            .withSimFeedforward(new SimpleMotorFeedforward(Constants.Shooter.CtrePidf.K_S, Constants.Shooter.CtrePidf.K_V, Constants.Shooter.CtrePidf.K_A))
            // Telemetry name and verbosity level
            .withTelemetry("CtreShooterMotors", TelemetryVerbosity.HIGH)
            // Gearing from the motor rotor to final shaft - 1:1 (direct drive)
            .withGearing(new MechanismGearing(1))
            .withMotorInverted(false)
            .withIdleMode(MotorMode.COAST)
            .withVoltageCompensation(Units.Volts.of(12))
            // Motor properties to prevent over currenting.
            .withStatorCurrentLimit(Units.Amps.of(100));  //TODO: verify this limit is not being hit during operation using telemetry, and adjust if necessary

    private final SmartMotorController shooterKrakenSmartMotorController = new TalonFXWrapper(shooterKrakenRight, DCMotor.getKrakenX60(1), shooterKrakenConfig);

    private final FlyWheelConfig shooterKrakenShooterConfig = new FlyWheelConfig(shooterKrakenSmartMotorController)
            // Diameter of the flywheel.
            .withDiameter(Constants.Shooter.SHOOTER_WHEEL_DIAMETER)
            // Mass of the flywheel.
            .withMass(Constants.Shooter.FLYWHEEL_MASS)
            // Maximum speed of the shooter.
            .withUpperSoftLimit(Units.RPM.of(6380))  //Kraken x60 has a max speed of 6380 RPM, so we set the limit slightly above that to prevent commanding an impossible speed while still allowing for some overshoot in closed loop control.
            // Telemetry name and verbosity for the arm.
            .withTelemetry("CtreShooter", TelemetryVerbosity.HIGH);

    // Shooter Mechanism
    private FlyWheel shooterKraken = new FlyWheel(shooterKrakenShooterConfig);

    // SysId setup for characterization
    private final VoltageOut ctreSysIdControl = new VoltageOut(0);
    private final MutAngle sysIdPosition = Units.Rotations.mutable(0);
    private final MutAngularVelocity sysIdVelocity = Units.RotationsPerSecond.mutable(0);
    private final MutVoltage sysIdAppliedVoltage = Volts.mutable(0);

    private final SysIdRoutine unifiedSysIdRoutine = new SysIdRoutine(
        new SysIdRoutine.Config(
            null,         // Use default ramp rate (1 V/s)
            Volts.of(4), // Reduce dynamic voltage to 4 to prevent brownout
            null,          // Use default timeout (10 s)
                                // Log state with Phoenix SignalLogger class
            state -> SignalLogger.writeString("shooter-state", state.toString())
        ),
        new SysIdRoutine.Mechanism(
            volts -> {
                shooterKrakenLeft.setControl(ctreSysIdControl.withOutput(volts));
                shooterKrakenRight.setControl(ctreSysIdControl.withOutput(volts));
                shooterVortexLeft.setVoltage(volts.in(Volts));
                shooterVortexRight.setVoltage(volts.in(Volts));
            },
            log -> {
                // Record a frame for the motors.
                log.motor("shooter-kraken-left")
                    .voltage(sysIdAppliedVoltage.mut_replace(shooterKrakenLeft.getMotorVoltage().getValueAsDouble(), Volts))
                    .angularPosition(sysIdPosition.mut_replace(shooterKrakenLeft.getRotorPosition().getValueAsDouble(), Units.Rotations))
                    .angularVelocity(sysIdVelocity.mut_replace(shooterKrakenLeft.getRotorVelocity().getValueAsDouble(), Units.RotationsPerSecond));
                log.motor("shooter-kraken-right")
                    .voltage(sysIdAppliedVoltage.mut_replace(shooterKrakenRight.getMotorVoltage().getValueAsDouble(), Volts))
                    .angularPosition(sysIdPosition.mut_replace(shooterKrakenRight.getRotorPosition().getValueAsDouble(), Units.Rotations))
                    .angularVelocity(sysIdVelocity.mut_replace(shooterKrakenRight.getRotorVelocity().getValueAsDouble(), Units.RotationsPerSecond));
                log.motor("shooter-vortex-left")
                    .voltage(sysIdAppliedVoltage.mut_replace(shooterVortexLeft.getAppliedOutput() * shooterVortexLeft.getBusVoltage(), Volts))
                    .angularPosition(sysIdPosition.mut_replace(leftVortexEncoder.getPosition(), Units.Rotations))
                    .angularVelocity(sysIdVelocity.mut_replace(leftVortexEncoder.getVelocity() / 60.0, Units.RotationsPerSecond));  //divide by 60 to convert RPM to RPS
                log.motor("shooter-vortex-right")
                    .voltage(sysIdAppliedVoltage.mut_replace(shooterVortexRight.getAppliedOutput() * shooterVortexRight.getBusVoltage(), Volts))
                    .angularPosition(sysIdPosition.mut_replace(rightVortexEncoder.getPosition(), Units.Rotations))
                    .angularVelocity(sysIdVelocity.mut_replace(rightVortexEncoder.getVelocity() / 60.0, Units.RotationsPerSecond));
            },
            this
        )
    );

    public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
        //ensure motors are in brake mode for quasistatic characterization to prevent coasting from skewing results
        setBrakeModeForAllMotors(true);
        return unifiedSysIdRoutine.quasistatic(direction);
    }

    public Command sysIdDynamic(SysIdRoutine.Direction direction) {
        setBrakeModeForAllMotors(true);
        return unifiedSysIdRoutine.dynamic(direction);
    }

    @Override
    public void simulationPeriodic() {
        shooterKraken.simIterate();
    }

    public void setShooterAngularVelocity(AngularVelocity rpmTarget) {
        setBrakeModeForAllMotors(false);
        shooterKraken.setMechanismVelocitySetpoint(rpmTarget);
        double leaderVolts = shooterKrakenRight.getMotorVoltage().getValueAsDouble();
        shooterVortexLeft.setVoltage(leaderVolts);
        shooterVortexRight.setVoltage(leaderVolts);
    }

    private void setBrakeModeForAllMotors(boolean isBrake) {
        var leftKrakenConfigurator = shooterKrakenLeft.getConfigurator()
        var leftKrakenConfigs = new TalonFXConfiguration();
        leftKrakenConfigurator.refresh(leftKrakenConfigs);  //read device settings from the motor and store them in the config object
        leftKrakenConfigs.MotorOutput.NeutralMode = isBrake ? NeutralModeValue.Brake : NeutralModeValue.Coast;  //modify the config object to set the desired brake mode
        leftKrakenConfigurator.apply(leftKrakenConfigs);  //write the updated config back to the motor

        var rightKrakenConfigurator = shooterKrakenRight.getConfigurator()
        var rightKrakenConfigs = new TalonFXConfiguration();
        rightKrakenConfigurator.refresh(rightKrakenConfigs);
        rightKrakenConfigs.MotorOutput.NeutralMode = isBrake ? NeutralModeValue.Brake : NeutralModeValue.Coast;
        rightKrakenConfigurator.apply(rightKrakenConfigs);

        var sparkMaxConfig = new SparkMaxConfig();
        sparkMaxConfig.idleMode(isBrake ? IdleMode.kBrake : IdleMode.kCoast);
        var sparkFlexConfig = new SparkFlexConfig();
        sparkFlexConfig.idleMode(isBrake ? IdleMode.kBrake : IdleMode.kCoast);
        shooterVortexLeft.configure(sparkFlexConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
        shooterVortexRight.configure(sparkMaxConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
    }

    public void stopShooter() {
        setBrakeModeForAllMotors(true);
        shooterKraken.set(0);
        shooterVortexLeft.setVoltage(0);
        shooterVortexRight.setVoltage(0);
    }

    public Command runAtAngularVelocity(AngularVelocity rpmTarget) {
        return this.run(() -> setShooterAngularVelocity(rpmTarget)).withName("Shooter at velocity: " + rpmTarget);
    }

    public Command idleAtLowRpm() {
        return runAtAngularVelocity(Units.RPM.of(500)).withName("Idle Shooter");  //TODO: determine appropriate idle speed
    }

    public Command stop() {
        return this.run(() -> stopShooter()).withName("Stop Shooter");
    }
     
}
