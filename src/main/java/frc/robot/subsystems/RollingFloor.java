package frc.robot.subsystems;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
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

public class RollingFloor extends SubsystemBase {

    private final TalonFX rightMotor = new TalonFX(Constants.CanID.ROLLING_FLOOR_RIGHT, CANBus.roboRIO());  //turn clockwise (leader)
    private final TalonFX leftMotor = new TalonFX(Constants.CanID.ROLLING_FLOOR_LEFT, CANBus.roboRIO());  //turn CCW (follower)
    
    private final DoublePublisher leftNeoVelocityPublisher = NetworkTableInstance.getDefault().getDoubleTopic("/RBR/RollingFloor/Velocity/Left").publish();
    private final DoublePublisher rightNeoVelocityPublisher = NetworkTableInstance.getDefault().getDoubleTopic("/RBR/RollingFloor/Velocity/Right").publish();
    private final DoublePublisher leftNeoVoltagePublisher = NetworkTableInstance.getDefault().getDoubleTopic("/RBR/RollingFloor/Voltage/Left").publish();
    private final DoublePublisher rightNeoVoltagePublisher = NetworkTableInstance.getDefault().getDoubleTopic("/RBR/RollingFloor/Voltage/Right").publish();
    private final DoublePublisher leftNeoCurrentPublisher = NetworkTableInstance.getDefault().getDoubleTopic("/RBR/RollingFloor/Current/Left").publish();
    private final DoublePublisher rightNeoCurrentPublisher = NetworkTableInstance.getDefault().getDoubleTopic("/RBR/RollingFloor/Current/Right").publish();

    private final Follower ctreFollowerControl = new Follower(Constants.CanID.ROLLING_FLOOR_RIGHT, MotorAlignmentValue.Opposed);

    public RollingFloor() {
        var rightMotorConfig = new TalonFXConfiguration();
        rightMotorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        rightMotorConfig.CurrentLimits.SupplyCurrentLimit = 40.0;
        rightMotorConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        rightMotor.getConfigurator().apply(rightMotorConfig);

        var leftMotorConfig = new TalonFXConfiguration();
        leftMotorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        leftMotorConfig.CurrentLimits.SupplyCurrentLimit = 40.0;
        leftMotorConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        leftMotor.getConfigurator().apply(leftMotorConfig);
    }

    @Override
    public void periodic() {
        leftNeoVelocityPublisher.set(leftMotor.getVelocity().getValueAsDouble());
        leftNeoVoltagePublisher.set(leftMotor.getMotorVoltage().getValueAsDouble());
        leftNeoCurrentPublisher.set(leftMotor.getStatorCurrent().getValueAsDouble());
        rightNeoVelocityPublisher.set(rightMotor.getVelocity().getValueAsDouble());
        rightNeoVoltagePublisher.set(rightMotor.getMotorVoltage().getValueAsDouble());
        rightNeoCurrentPublisher.set(rightMotor.getStatorCurrent().getValueAsDouble());
    }

    //run duty cycle
    public void setDutyCycleSpeed(double speed) {
        rightMotor.set(speed);
        leftMotor.setControl(ctreFollowerControl);
        // leftMotor.set(-speed);
    }

    public Command runAtDutyCycle(double speed) {
        return this.run(() -> setDutyCycleSpeed(speed)).withName("Run RollingFloor at speed: " + speed);
    }

    public Command rollInwards() {
        return runAtDutyCycle(Constants.RollingFloor.ROLL_DUTY_CYCLE);
    }


    public Command rollInwardsFast() {
        return runAtDutyCycle(0.88);
    }

    public Command rollInwardsSlowly() {
        return runAtDutyCycle(0.10);
    }

    public Command rollOutwards() {
        return runAtDutyCycle(-Constants.RollingFloor.ROLL_DUTY_CYCLE);
    }

    public Command stop() {
        return this.run(() -> setDutyCycleSpeed(0.0)).withName("Stop RollingFloor");
    }
}
