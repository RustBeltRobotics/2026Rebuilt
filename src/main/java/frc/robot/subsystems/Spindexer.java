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

public class Spindexer extends SubsystemBase {

    private final SparkMax neoRight = new SparkMax(Constants.CanID.SPINDEXER_RIGHT, MotorType.kBrushless);  //turn clockwise (leader)
    private final RelativeEncoder rightNeoEncoder = neoRight.getEncoder();
    private final SparkMax neoLeft = new SparkMax(Constants.CanID.SPINDEXER_LEFT, MotorType.kBrushless);  //turn CCW (follower)
    private final RelativeEncoder leftNeoEncoder = neoLeft.getEncoder();
    
    private final DoublePublisher leftNeoVelocityPublisher = NetworkTableInstance.getDefault().getDoubleTopic("/RBR/Spindexer/Velocity/Left").publish();
    private final DoublePublisher rightNeoVelocityPublisher = NetworkTableInstance.getDefault().getDoubleTopic("/RBR/Spindexer/Velocity/Right").publish();
    private final DoublePublisher leftNeoVoltagePublisher = NetworkTableInstance.getDefault().getDoubleTopic("/RBR/Spindexer/Voltage/Left").publish();
    private final DoublePublisher rightNeoVoltagePublisher = NetworkTableInstance.getDefault().getDoubleTopic("/RBR/Spindexer/Voltage/Right").publish();
    private final DoublePublisher leftNeoCurrentPublisher = NetworkTableInstance.getDefault().getDoubleTopic("/RBR/Spindexer/Current/Left").publish();
    private final DoublePublisher rightNeoCurrentPublisher = NetworkTableInstance.getDefault().getDoubleTopic("/RBR/Spindexer/Current/Right").publish();

    public Spindexer() {
        var rightRevConfig = new SparkMaxConfig();
        rightRevConfig.idleMode(IdleMode.kBrake);
        rightRevConfig.voltageCompensation(12.0);
        rightRevConfig.smartCurrentLimit(Constants.CurrentLimit.SparkMax.Neo.SMART_DEFAULT).secondaryCurrentLimit(Constants.CurrentLimit.SparkMax.Neo.SECONDARY_MAX);
        neoRight.configure(rightRevConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        var leftRevConfig = new SparkMaxConfig();
        leftRevConfig.idleMode(IdleMode.kBrake);
        leftRevConfig.voltageCompensation(12.0);
        leftRevConfig.smartCurrentLimit(Constants.CurrentLimit.SparkMax.Neo.SMART_DEFAULT).secondaryCurrentLimit(Constants.CurrentLimit.SparkMax.Neo.SECONDARY_MAX);
        leftRevConfig.inverted(true);
        // leftRevConfig.follow(Constants.CanID.SPINDEXER_RIGHT, true);
        neoLeft.configure(leftRevConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    @Override
    public void periodic() {
        leftNeoVelocityPublisher.set(leftNeoEncoder.getVelocity());
        leftNeoVoltagePublisher.set(neoLeft.getAppliedOutput() * neoLeft.getBusVoltage());
        leftNeoCurrentPublisher.set(neoLeft.getOutputCurrent());
        rightNeoVelocityPublisher.set(rightNeoEncoder.getVelocity());
        rightNeoVoltagePublisher.set(neoRight.getAppliedOutput() * neoRight.getBusVoltage());
        rightNeoCurrentPublisher.set(neoRight.getOutputCurrent());
    }

    //run duty cycle
    public void setDutyCycleSpeed(double speed) {
        neoRight.set(speed);
        neoLeft.set(speed);
    }

    public Command runAtDutyCycle(double speed) {
        return this.run(() -> setDutyCycleSpeed(speed)).withName("Run Spindexer at speed: " + speed);
    }

    public Command spin() {
        return runAtDutyCycle(Constants.Spindexer.SPIN_DUTY_CYCLE);
    }

    public Command stop() {
        return this.run(() -> setDutyCycleSpeed(0.0)).withName("Stop Spindexer");
    }
}
