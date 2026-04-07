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
import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants;
import frc.robot.util.AlertManager;

public class IntakeArm extends SubsystemBase {

    private final SparkMax extendRetractMotor = new SparkMax(Constants.CanID.INTAKE_EXTEND_RETRACT_MOTOR, MotorType.kBrushless); //27:1 gear ratio on extend/retract motor
    private final RelativeEncoder extendRetractEncoder = extendRetractMotor.getEncoder();
    private double extendRetractMotorOutputCurrentAmps;
    private boolean isExtending;
    private boolean stallDetected;
    private Debouncer stallDetectionDebouncer = new Debouncer(0.1, DebounceType.kRising);
    private Trigger shouldStopExtendRetract = new Trigger(() -> {
        if (isExtending && stallDetected) {
            return true;
        } else if (!isExtending && stallDetected && Math.abs(extendRetractEncoder.getPosition()) <= Constants.Intake.ENCODER_POSITION_ZERO_THRESHOLD) {
            return true;
        } else {
            return false;
        }
    });

    private final DoublePublisher extendRetractVelocityPublisher = NetworkTableInstance.getDefault().getDoubleTopic("/RBR/Intake/ExtendRetract/Velocity").publish();
    private final DoublePublisher extendRetractCurrentPublisher = NetworkTableInstance.getDefault().getDoubleTopic("/RBR/Intake/ExtendRetract/Current").publish();
    private final DoublePublisher extendRetractPositionPublisher = NetworkTableInstance.getDefault().getDoubleTopic("/RBR/Intake/ExtendRetract/Position").publish();
    private final BooleanPublisher atHardStopPublisher = NetworkTableInstance.getDefault().getBooleanTopic("/RBR/Intake/ExtendRetract/AtHardStop").publish();

    //When initially extending from fully retracted position, current will spike to 45+ amps and takes approx 40ms (0.04 S) until velocity increases above 0

    public IntakeArm() {
        var extendRetractConfig = new SparkMaxConfig();
        extendRetractConfig.idleMode(IdleMode.kBrake);
        //TODO: update this note on gear ratio is it likely is changing
        //Note: the gear ratio on this mechanism is 64:1 (4x4x4)
        extendRetractConfig.smartCurrentLimit(70).secondaryCurrentLimit(60);
        extendRetractMotor.configure(extendRetractConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        extendRetractEncoder.setPosition(0.0);  //zero the encoder on startup (the arm will be retracted within the robot perimeter)
    }

    @Override
    public void periodic() {
        extendRetractMotorOutputCurrentAmps = extendRetractMotor.getOutputCurrent();
        extendRetractCurrentPublisher.set(extendRetractMotorOutputCurrentAmps);
        extendRetractPositionPublisher.set(extendRetractEncoder.getPosition());
        double rawVelocity = extendRetractEncoder.getVelocity();
        extendRetractVelocityPublisher.set(rawVelocity);
        double extendRetractMotorVelocityRps = Math.abs(rawVelocity / 60);
        stallDetected = stallDetectionDebouncer.calculate(extendRetractMotorVelocityRps < 1 && extendRetractMotorOutputCurrentAmps >= 50.0);
        atHardStopPublisher.set(stallDetected);
        
        AlertManager.addAlert("IntakeArm", "IntakeArm stallDetected? " + (stallDetected ? "Yes" : "No"), AlertType.kInfo);
    }

    public void runExtendRetractAtDutyCycle(double dutyCycle) {
        extendRetractMotor.set(dutyCycle);
    }

    public Command stopExtendRetract() {
        return runOnce(() -> {
            isExtending = true;
            runExtendRetractAtDutyCycle(0.0);
        }).withName("Stop Intake Extend/Retract");
    }

    public Command extend() {
       return run(() -> {
            isExtending = true;
            runExtendRetractAtDutyCycle(0.3);
        })
        .until(() -> shouldStopExtendRetract.getAsBoolean())
        .andThen(stopExtendRetract());
    }

    public Command extendForAutonomous() {
       return run(() -> {
            isExtending = true;
            runExtendRetractAtDutyCycle(0.5);
        })
        .until(() -> shouldStopExtendRetract.getAsBoolean())
        .andThen(stopExtendRetract());
    }

    public Command extendForAutoAgitate() {
       return run(() -> {
            isExtending = true;
            runExtendRetractAtDutyCycle(0.15);
        })
        .until(() -> shouldStopExtendRetract.getAsBoolean())
        .andThen(stopExtendRetract());
    }

    public Command extendForIntakeSequence() {
       return this.startEnd(() -> { 
            runExtendRetractAtDutyCycle(0.03);
        }, () -> {
            stopExtendRetract();
        }).withName("Extending intake arm for fuel intake sequence");
    }

    public Command extendForIntakeSequenceAuto() {
       return this.runOnce(() -> { 
            runExtendRetractAtDutyCycle(0.06);
        });
    }

    public Command retract() {
        return run(() -> {
            isExtending = false;
            runExtendRetractAtDutyCycle(-0.40);
        })
        .until(() -> shouldStopExtendRetract.getAsBoolean())
        .andThen(stopExtendRetract());
    }

    public Command retractForAutoAgitate() {
        return run(() -> {
            isExtending = false;
            runExtendRetractAtDutyCycle(-0.25);
        })
        .until(() -> shouldStopExtendRetract.getAsBoolean())
        .andThen(stopExtendRetract());
    }

    public Command retractForAgitate() {
        return run(() -> {
            isExtending = false;
            runExtendRetractAtDutyCycle(-0.50);
        })
        .andThen(stopExtendRetract());
    }
}
