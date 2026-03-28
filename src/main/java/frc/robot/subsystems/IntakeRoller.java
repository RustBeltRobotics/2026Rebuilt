package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class IntakeRoller extends SubsystemBase {

    private final SparkMax rotateIntakeRightShaftMotor = new SparkMax(Constants.CanID.INTAKE_ROTATE_MOTOR_RIGHT, MotorType.kBrushless);
    private final SparkMax rotateIntakeLeftShaftMotor = new SparkMax(Constants.CanID.INTAKE_ROTATE_MOTOR_LEFT, MotorType.kBrushless);
    private final RelativeEncoder rotateIntakeRightShaftEncoder = rotateIntakeRightShaftMotor.getEncoder();
    private final RelativeEncoder rotateIntakeLeftShaftEncoder = rotateIntakeLeftShaftMotor.getEncoder();

    private final DoublePublisher rotateIntakeShaftCurrentPublisher = NetworkTableInstance.getDefault().getDoubleTopic("/RBR/Intake/Rotation/Current").publish();
    private final DoublePublisher rotateIntakeShaftVelocityPublisher = NetworkTableInstance.getDefault().getDoubleTopic("/RBR/Intake/Rotation/Velocity").publish();
    private final DoublePublisher rotateIntakeLeftVoltagePublisher = NetworkTableInstance.getDefault().getDoubleTopic("/RBR/Intake/Rotation/Voltage/Left").publish();
    private final DoublePublisher rotateIntakeRightVoltagePublisher = NetworkTableInstance.getDefault().getDoubleTopic("/RBR/Intake/Rotation/Voltage/Right").publish();

    public IntakeRoller() {
        var rotateIntakeRightShaftConfig = new SparkMaxConfig();
        rotateIntakeRightShaftConfig.idleMode(IdleMode.kBrake);
        rotateIntakeRightShaftConfig.inverted(false);
        rotateIntakeRightShaftConfig.smartCurrentLimit(75).secondaryCurrentLimit(80);

        rotateIntakeRightShaftMotor.configure(rotateIntakeRightShaftConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        var rotateIntakeLeftShaftConfig = new SparkMaxConfig();
        rotateIntakeLeftShaftConfig.idleMode(IdleMode.kBrake);
        rotateIntakeLeftShaftConfig.inverted(false);
        rotateIntakeLeftShaftConfig.smartCurrentLimit(75).secondaryCurrentLimit(80);

        rotateIntakeLeftShaftMotor.configure(rotateIntakeLeftShaftConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        rotateIntakeLeftShaftConfig.follow(Constants.CanID.INTAKE_ROTATE_MOTOR_RIGHT, true);
    }

    @Override
    public void periodic() {
        rotateIntakeShaftCurrentPublisher.set(rotateIntakeRightShaftMotor.getOutputCurrent());
        rotateIntakeShaftVelocityPublisher.set(rotateIntakeRightShaftEncoder.getVelocity());
        rotateIntakeLeftVoltagePublisher.set(rotateIntakeLeftShaftMotor.getAppliedOutput() * rotateIntakeLeftShaftMotor.getBusVoltage());
        rotateIntakeRightVoltagePublisher.set(rotateIntakeRightShaftMotor.getAppliedOutput() * rotateIntakeRightShaftMotor.getBusVoltage());
    }

    public void runIntakeWheelsAtDutyCycle(double dutyCycle) {
        rotateIntakeRightShaftMotor.set(dutyCycle);
        rotateIntakeLeftShaftMotor.set(-dutyCycle);
    }

    public Command intakeFuel() {
        return startEnd(() -> runIntakeWheelsAtDutyCycle(0.9), () -> runIntakeWheelsAtDutyCycle(0.0)).withName("Intake rotate fuel");
    }

    public Command intakeFuelForAuto() {
        return runOnce(() -> runIntakeWheelsAtDutyCycle(1.0));
    }

    public Command outtakeFuel() {
        return startEnd(() -> runIntakeWheelsAtDutyCycle(-0.9), () -> runIntakeWheelsAtDutyCycle(0.0)).withName("Outtake rotate fuel");
    }

    public Command stopIntakeWheelRotation() {
        return runOnce(() -> runIntakeWheelsAtDutyCycle(0.0)).withName("Stop Intake Wheel Rotation");
    }

}
