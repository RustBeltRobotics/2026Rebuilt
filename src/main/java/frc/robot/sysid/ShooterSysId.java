package frc.robot.sysid;

import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
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

import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.MutAngle;
import edu.wpi.first.units.measure.MutAngularVelocity;
import edu.wpi.first.units.measure.MutVoltage;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants;

public class ShooterSysId extends SubsystemBase {

    private final TalonFX shooterKrakenLeft = new TalonFX(Constants.CanID.SHOOTER_KRAKEN_LEFT, CANBus.roboRIO()); //follower
    private final TalonFX shooterKrakenRight = new TalonFX(Constants.CanID.SHOOTER_KRAKEN_RIGHT, CANBus.roboRIO());  //primary
    private final SparkFlex shooterVortexLeft = new SparkFlex(Constants.CanID.SHOOTER_VORTEX_LEFT, MotorType.kBrushless);  //follower - negative rotation spins in the direction we want
    private final RelativeEncoder leftVortexEncoder = shooterVortexLeft.getEncoder();
    private final SparkMax shooterVortexRight = new SparkMax(Constants.CanID.SHOOTER_VORTEX_RIGHT, MotorType.kBrushless);  //primary - positive rotation spins in the direction we want
    private final RelativeEncoder rightVortexEncoder = shooterVortexRight.getEncoder();

    // SysId setup for characterization
    private final VoltageOut ctreSysIdControl = new VoltageOut(0);
    private final Follower ctreFollowerControl = new Follower(Constants.CanID.SHOOTER_KRAKEN_RIGHT, MotorAlignmentValue.Opposed);  //set follower to follow the primary with inverted output so that both motors spin in the same direction
    private final MutAngle sysIdPosition = Units.Rotations.mutable(0);
    private final MutAngularVelocity sysIdVelocity = Units.RotationsPerSecond.mutable(0);
    private final MutVoltage sysIdAppliedVoltage = Volts.mutable(0);

    private boolean sysIdTestsStarted = false;

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
                shooterKrakenRight.setControl(ctreSysIdControl.withOutput(volts.in(Volts))); //positive voltage = clockwise
                shooterKrakenLeft.setControl(ctreFollowerControl);
                shooterVortexRight.setVoltage(volts.in(Volts));
                // shooterVortexLeft.setVoltage(volts.unaryMinus().in(Volts));  //we don't set voltage for the left vortex since it's following the right vortex with inverted output, so it will automatically apply the correct voltage to spin in the same direction as the right vortex at the same speed
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

    public ShooterSysId() {
        var leftKrakenConfigurator = shooterKrakenLeft.getConfigurator();
        var leftKrakenConfigs = new TalonFXConfiguration();
        leftKrakenConfigurator.refresh(leftKrakenConfigs);  //read device settings from the motor and store them in the config object
        leftKrakenConfigs.MotorOutput.NeutralMode = NeutralModeValue.Brake;  //modify the config object to set the desired brake mode
        leftKrakenConfigurator.apply(leftKrakenConfigs);  //write the updated config back to the motor

        var rightKrakenConfigurator = shooterKrakenRight.getConfigurator();
        var rightKrakenConfigs = new TalonFXConfiguration();
        rightKrakenConfigurator.refresh(rightKrakenConfigs);
        rightKrakenConfigs.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        rightKrakenConfigurator.apply(rightKrakenConfigs);

        var sparkMaxConfig = new SparkMaxConfig();
        sparkMaxConfig.idleMode(IdleMode.kBrake);
        var sparkFlexConfig = new SparkFlexConfig();
        sparkFlexConfig.idleMode(IdleMode.kBrake);
        sparkFlexConfig.follow(Constants.CanID.SHOOTER_VORTEX_RIGHT, true);  //set left vortex to follow right vortex with inverted output
        shooterVortexLeft.configure(sparkFlexConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
        shooterVortexRight.configure(sparkMaxConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
    }

    public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
        return startSysIdLogging().andThen(unifiedSysIdRoutine.quasistatic(direction));
    }

    public Command sysIdDynamic(SysIdRoutine.Direction direction) {
        return startSysIdLogging().andThen(unifiedSysIdRoutine.dynamic(direction));
    }

    public Command startSysIdLogging() {
        return runOnce(() -> {
            if (!sysIdTestsStarted) {
                SignalLogger.stop();
                sysIdTestsStarted = true;
                SignalLogger.start();
            }
        });
    }

    public Command stopSysIdLogging() {
        return runOnce(() -> {
            sysIdTestsStarted = false;
            SignalLogger.stop();
        });
    }
     
}
