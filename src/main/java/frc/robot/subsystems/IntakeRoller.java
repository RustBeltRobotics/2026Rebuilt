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
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants;

public class IntakeRoller extends SubsystemBase {

    private final SparkMax rotateIntakeShaftMotor = new SparkMax(Constants.CanID.INTAKE_ROTATE_MOTOR, MotorType.kBrushless);
    private final RelativeEncoder rotateIntakeShaftEncoder = rotateIntakeShaftMotor.getEncoder();

    private final DoublePublisher rotateIntakeShaftVelocityPublisher = NetworkTableInstance.getDefault().getDoubleTopic("/RBR/Intake/Rotation/Velocity").publish();
    
    public IntakeRoller() {
        var rotateIntakeShaftConfig = new SparkMaxConfig();
        rotateIntakeShaftConfig.idleMode(IdleMode.kBrake);
        rotateIntakeShaftConfig.inverted(true);
        rotateIntakeShaftMotor.configure(rotateIntakeShaftConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    @Override
    public void periodic() {
        rotateIntakeShaftVelocityPublisher.set(rotateIntakeShaftEncoder.getVelocity());
    }

    public void runIntakeWheelsAtDutyCycle(double dutyCycle) {
        rotateIntakeShaftMotor.set(dutyCycle);
    }

    public Command intakeFuel() {
        return startEnd(() -> runIntakeWheelsAtDutyCycle(-1.0), () -> runIntakeWheelsAtDutyCycle(0.0)).withName("Intake rotate fuel");
    }

    public Command stopIntakeWheelRotation() {
        return runOnce(() -> runIntakeWheelsAtDutyCycle(0.0)).withName("Stop Intake Wheel Rotation");
    }

}
