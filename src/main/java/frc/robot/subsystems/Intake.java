package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Tracer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants;

public class Intake extends SubsystemBase {

    private final SparkMax rotateIntakeShaftMotor = new SparkMax(Constants.CanID.INTAKE_ROTATE_MOTOR, MotorType.kBrushless);
    private final RelativeEncoder rotateIntakeShaftEncoder = rotateIntakeShaftMotor.getEncoder();
    private final SparkMax extendRetractMotor = new SparkMax(Constants.CanID.INTAKE_EXTEND_RETRACT_MOTOR, MotorType.kBrushless); //27:1 gear ratio on extend/retract motor
    private final RelativeEncoder extendRetractEncoder = extendRetractMotor.getEncoder();
    private double extendRetractMotorOutputCurrentAmps;
    private boolean isExtending;
    private boolean stallDetected;
    private Debouncer stallDetectionDebouncer = new Debouncer(0.25, DebounceType.kRising);
    private Trigger shouldStopExtendRetract = new Trigger(() -> {
        if (isExtending && stallDetected) {
            return true;
        } else if (!isExtending && stallDetected && Math.abs(extendRetractEncoder.getPosition()) <= Constants.Intake.ENCODER_POSITION_ZERO_THRESHOLD) {
            return true;
        } else {
            return false;
        }
    });

    private final DoublePublisher rotateIntakeShaftVelocityPublisher = NetworkTableInstance.getDefault().getDoubleTopic("/RBR/Intake/Rotation/Velocity").publish();
    private final DoublePublisher extendRetractVelocityPublisher = NetworkTableInstance.getDefault().getDoubleTopic("/RBR/Intake/ExtendRetract/Velocity").publish();
    
    public Intake() {
        var rotateIntakeShaftConfig = new SparkMaxConfig();
        rotateIntakeShaftConfig.idleMode(IdleMode.kBrake);
        rotateIntakeShaftMotor.configure(rotateIntakeShaftConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        var extendRetractConfig = new SparkMaxConfig();
        extendRetractConfig.idleMode(IdleMode.kBrake);
        extendRetractMotor.configure(extendRetractConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    @Override
    public void periodic() {
        extendRetractMotorOutputCurrentAmps = extendRetractMotor.getOutputCurrent();
        // If the current exceeds 10 amps for more than 0.25 seconds, we consider the mechanism to be stalled
        //TODO: Test this amperage
        stallDetected = stallDetectionDebouncer.calculate(extendRetractMotorOutputCurrentAmps >= 10.0);
        if (stallDetected) {
            extendRetractEncoder.setPosition(0.0);
        }
        rotateIntakeShaftVelocityPublisher.set(rotateIntakeShaftEncoder.getVelocity());
        extendRetractVelocityPublisher.set(extendRetractEncoder.getVelocity());
    }

    public void runIntakeWheelsAtDutyCycle(double dutyCycle) {
        rotateIntakeShaftMotor.set(dutyCycle);
    }

    public void runExtendRetractAtDutyCycle(double dutyCycle) {
        extendRetractMotor.set(dutyCycle);
    }

    public Command intakeFuel() {
        return this.startEnd(() -> runIntakeWheelsAtDutyCycle(-1.0), () -> runIntakeWheelsAtDutyCycle(0.0)).withName("Intake rotate fuel");
    }

    public Command stopIntakeWheelRotation() {
        return this.runOnce(() -> runIntakeWheelsAtDutyCycle(0.0)).withName("Stop Intake Wheel Rotation");
    }

    public Command stopExtendRetract() {
        return this.runOnce(() -> runExtendRetractAtDutyCycle(0.0)).withName("Stop Intake Extend/Retract");
    }

    public Command extend() {
       return run(() -> {
            isExtending = true;
            runExtendRetractAtDutyCycle(-0.2);
        })
        .until(() -> shouldStopExtendRetract.getAsBoolean())
        .andThen(stopExtendRetract());
    }

    public Command extendForIntakeSequence() {
       return this.startEnd(() -> { 
            runIntakeWheelsAtDutyCycle(-1.0); 
            runExtendRetractAtDutyCycle(-0.03);
        }, () -> {
            runIntakeWheelsAtDutyCycle(0.0);
            stopExtendRetract();
        }).withName("Intake rotate fuel with extension");

    }

    public Command retract() {
        return run(() -> {
            isExtending = false;
            runExtendRetractAtDutyCycle(0.35);
        })
        .until(() -> shouldStopExtendRetract.getAsBoolean())
        .andThen(stopExtendRetract());
    }
}
