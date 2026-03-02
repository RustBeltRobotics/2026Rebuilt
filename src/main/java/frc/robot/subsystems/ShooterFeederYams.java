package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.AngularVelocity;
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
import yams.motorcontrollers.remote.TalonFXWrapper;

public class ShooterFeederYams extends SubsystemBase {

    private final TalonFX feederKrakenLeft = new TalonFX(Constants.CanID.FEEDER_KRAKEN_LEFT, CANBus.roboRIO()); //primary
    private final TalonFX feederKrakenRight = new TalonFX(Constants.CanID.FEEDER_KRAKEN_RIGHT, CANBus.roboRIO()); //follower
    
    private final VoltageOut sysIdControl = new VoltageOut(0);
    private boolean sysIdTestsStarted = false;

    private final SysIdRoutine sysIdRoutine = new SysIdRoutine(
        new SysIdRoutine.Config(
            null,         // Use default ramp rate (1 V/s)
            Volts.of(4), // Reduce dynamic voltage to 4 to prevent brownout
            null,          // Use default timeout (10 s)
                                // Log state with Phoenix SignalLogger class
            state -> SignalLogger.writeString("feeder-state", state.toString())
        ),
        new SysIdRoutine.Mechanism(
            volts -> {
                feederKrakenLeft.setControl(sysIdControl.withOutput(volts.in(Volts)));
                feederKrakenRight.setControl(sysIdControl.withOutput(volts.unaryMinus().in(Volts)));
            },
            null,
            this
        )
    );

    SmartMotorControllerConfig feederKrakenConfig = new SmartMotorControllerConfig(this)
            .withControlMode(ControlMode.CLOSED_LOOP)
            .withFollowers(Pair.of(feederKrakenRight, true))
            // Feedback Constants (PID Constants)
            .withClosedLoopController(Constants.ShooterFeeder.K_P, Constants.ShooterFeeder.K_I,
                    Constants.ShooterFeeder.K_D, Units.RadiansPerSecond.of(524.0),
                    Units.RadiansPerSecondPerSecond.of(1400.0))
            .withSimClosedLoopController(Constants.ShooterFeeder.K_P, Constants.ShooterFeeder.K_I,
                    Constants.ShooterFeeder.K_D, Units.RadiansPerSecond.of(524.0),
                    Units.RadiansPerSecondPerSecond.of(1400.0))
            // Feedforward Constants
            .withFeedforward(new SimpleMotorFeedforward(Constants.ShooterFeeder.K_S, Constants.ShooterFeeder.K_V, Constants.ShooterFeeder.K_A))
            .withSimFeedforward(new SimpleMotorFeedforward(Constants.ShooterFeeder.K_S, Constants.ShooterFeeder.K_V, Constants.ShooterFeeder.K_A))
            // Telemetry name and verbosity level
            .withTelemetry("FeederMotor", TelemetryVerbosity.HIGH)
            // Gearing from the motor rotor to final shaft - 1:1 (direct drive)
            .withGearing(new MechanismGearing(1))
            .withMotorInverted(false)
            .withIdleMode(MotorMode.BRAKE)
            // .withVoltageCompensation(Units.Volts.of(12))  //Note: we can't use this - apparently it's a Pro/paid feature
            // Motor properties to prevent over currenting.
            .withStatorCurrentLimit(Units.Amps.of(100));  //was 100

    SmartMotorController krakenSmartMotorController = new TalonFXWrapper(feederKrakenLeft, DCMotor.getKrakenX60(1), feederKrakenConfig);

    private final FlyWheelConfig feederKrakenShooterConfig = new FlyWheelConfig(krakenSmartMotorController)
            // Diameter of the flywheel.
            .withDiameter(Constants.ShooterFeeder.FEEDER_WHEEL_DIAMETER)
            // Mass of the flywheel.
            .withMass(Constants.ShooterFeeder.FLYWHEEL_MASS)
            // Maximum speed of the shooter.
            .withUpperSoftLimit(Units.RPM.of(6380))  //Kraken x60 has a max speed of 6380 RPM, so we set the limit slightly above that to prevent commanding an impossible speed while still allowing for some overshoot in closed loop control.
            // Telemetry name and verbosity for the arm.
            .withTelemetry("ShooterFeeder", TelemetryVerbosity.HIGH);

    private FlyWheel feederFlywheel = new FlyWheel(feederKrakenShooterConfig);

    public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
        return startSysIdLogging().andThen(sysIdRoutine.quasistatic(direction));
    }

    public Command sysIdDynamic(SysIdRoutine.Direction direction) {
        return startSysIdLogging().andThen(sysIdRoutine.dynamic(direction));
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

    @Override
    public void simulationPeriodic() {
        feederFlywheel.simIterate();
    }

    public void setFeederAngularVelocity(AngularVelocity rpmTarget) {
        feederFlywheel.setMechanismVelocitySetpoint(rpmTarget);
    }

    public Command runAtAngularVelocity(AngularVelocity rpmTarget) {
        return this.run(() -> setFeederAngularVelocity(rpmTarget)).withName("Feeder velocity: " + rpmTarget);
    }

    public Command stop() {
        return this.run(() -> feederFlywheel.setMechanismVelocitySetpoint(Units.RPM.of(0))).withName("Stop Shooter Feeder");
    }

}
