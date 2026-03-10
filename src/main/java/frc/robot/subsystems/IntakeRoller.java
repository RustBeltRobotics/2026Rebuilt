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

    private final SparkMax rotateIntakeShaftMotor = new SparkMax(Constants.CanID.INTAKE_ROTATE_MOTOR, MotorType.kBrushless);
    private final RelativeEncoder rotateIntakeShaftEncoder = rotateIntakeShaftMotor.getEncoder();

    private final DoublePublisher rotateIntakeShaftCurrentPublisher = NetworkTableInstance.getDefault().getDoubleTopic("/RBR/Intake/Rotation/Current").publish();
    private final DoublePublisher rotateIntakeShaftVelocityPublisher = NetworkTableInstance.getDefault().getDoubleTopic("/RBR/Intake/Rotation/Velocity").publish();

    public IntakeRoller() {
        var rotateIntakeShaftConfig = new SparkMaxConfig();
        rotateIntakeShaftConfig.idleMode(IdleMode.kBrake);
        rotateIntakeShaftConfig.inverted(true);
        rotateIntakeShaftConfig.smartCurrentLimit(75).secondaryCurrentLimit(80);

        rotateIntakeShaftMotor.configure(rotateIntakeShaftConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    @Override
    public void periodic() {
        rotateIntakeShaftCurrentPublisher.set(rotateIntakeShaftMotor.getOutputCurrent());
        rotateIntakeShaftVelocityPublisher.set(rotateIntakeShaftEncoder.getVelocity());
    }

    public void runIntakeWheelsAtDutyCycle(double dutyCycle) {
        rotateIntakeShaftMotor.set(dutyCycle);
    }

    public Command intakeFuel() {
        return startEnd(() -> runIntakeWheelsAtDutyCycle(-1.0), () -> runIntakeWheelsAtDutyCycle(0.0)).withName("Intake rotate fuel");
    }

    public Command intakeFuelForAuto() {
        return runOnce(() -> runIntakeWheelsAtDutyCycle(-1.0));
    }

    public Command outtakeFuel() {
        return startEnd(() -> runIntakeWheelsAtDutyCycle(1.0), () -> runIntakeWheelsAtDutyCycle(0.0)).withName("Outtake rotate fuel");
    }

    public Command stopIntakeWheelRotation() {
        return runOnce(() -> runIntakeWheelsAtDutyCycle(0.0)).withName("Stop Intake Wheel Rotation");
    }

}
