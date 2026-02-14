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
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Intake extends SubsystemBase {

    private final SparkMax rotateIntakeShaftMotor = new SparkMax(Constants.CanID.INTAKE_ROTATE_MOTOR, MotorType.kBrushless);
    private final RelativeEncoder rotateIntakeShaftEncoder = rotateIntakeShaftMotor.getEncoder();
    private final SparkMax extendRetractMotor = new SparkMax(Constants.CanID.INTAKE_EXTEND_RETRACT_MOTOR, MotorType.kBrushless); //27:1 gear ratio on extend/retract motor
    private final RelativeEncoder extendRetractEncoder = extendRetractMotor.getEncoder();
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
        rotateIntakeShaftVelocityPublisher.set(rotateIntakeShaftEncoder.getVelocity());
        extendRetractVelocityPublisher.set(extendRetractEncoder.getVelocity());
    }

    public void runIntakeWheelsAtDutyCycle(double dutyCycle) {
        rotateIntakeShaftMotor.set(dutyCycle);
    }

    public Command intakeFuel() {
        return this.startEnd(() -> runIntakeWheelsAtDutyCycle(0.5), () -> runIntakeWheelsAtDutyCycle(0.0));
    }

    public Command stopIntakeWheelRotation() {
        return this.runOnce(() -> runIntakeWheelsAtDutyCycle(0.0));
    }

    public Command extend() {
        return Commands.none();
    }

    public Command retract() {
        return Commands.none();
    }
}
