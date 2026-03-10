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

    public IntakeArm() {
        var extendRetractConfig = new SparkMaxConfig();
        extendRetractConfig.idleMode(IdleMode.kBrake);
        //Note: the gear ratio on this mechanism is 64:1 (4x4x4)
        extendRetractConfig.smartCurrentLimit(70).secondaryCurrentLimit(60);
        extendRetractMotor.configure(extendRetractConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        extendRetractEncoder.setPosition(0.0);  //zero the encoder on startup
    }

    @Override
    public void periodic() {
        extendRetractMotorOutputCurrentAmps = extendRetractMotor.getOutputCurrent();
        extendRetractCurrentPublisher.set(extendRetractMotorOutputCurrentAmps);
        double rawVelocity = extendRetractEncoder.getVelocity();
        extendRetractVelocityPublisher.set(rawVelocity);
        double extendRetractMotorVelocityRps = Math.abs(rawVelocity / 60);
        stallDetected = stallDetectionDebouncer.calculate(extendRetractMotorVelocityRps < 1 && extendRetractMotorOutputCurrentAmps >= 50.0);
        
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
            runExtendRetractAtDutyCycle(-0.4);
        })
        .until(() -> shouldStopExtendRetract.getAsBoolean())
        .andThen(stopExtendRetract());
    }

    public Command extendForIntakeSequence() {
       return this.startEnd(() -> { 
            runExtendRetractAtDutyCycle(-0.06);
        }, () -> {
            stopExtendRetract();
        }).withName("Extending intake arm for fuel intake sequence");
    }

    public Command extendForIntakeSequenceAuto() {
       return this.runOnce(() -> { 
            runExtendRetractAtDutyCycle(-0.12);
        });
    }

    public Command retract() {
        return run(() -> {
            isExtending = false;
            runExtendRetractAtDutyCycle(0.80);
        })
        .until(() -> shouldStopExtendRetract.getAsBoolean())
        .andThen(stopExtendRetract());
    }

    public Command retractForAgitate() {
        return run(() -> {
            isExtending = false;
            runExtendRetractAtDutyCycle(0.80);
        })
        .andThen(stopExtendRetract());
    }

    /* 
    //TODO: Test this bound to a button first, then integrate it into the shoot sequence
    public Command agitate() {
        Command intakeFuelCommand = run(() -> runIntakeWheelsAtDutyCycle(-1.0));
        Command liftAndDropIntakeArmCommand = run(() -> runExtendRetractAtDutyCycle(0.35)).withTimeout(0.10)
                .andThen(run(() -> runExtendRetractAtDutyCycle(-0.20)).withTimeout(0.10))
                .repeatedly();

                //TODO: can't run these in parallel since both require this subsystem
        // return new ParallelCommandGroup(intakeFuelCommand, liftAndDropIntakeArmCommand.finallyDo(() -> {
        //     runIntakeWheelsAtDutyCycle(0.0);
        //     stopExtendRetract();
        // }).withName("Agitate Intake"));

        return idle();
    }
        */
}
