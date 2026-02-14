package frc.robot.subsystems;

import com.ctre.phoenix6.SignalLogger;
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
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants;

public class Spindexer extends SubsystemBase {

    private final SparkMax neoRight = new SparkMax(Constants.CanID.SPINDEXER_RIGHT, MotorType.kBrushless);  //turn clockwise (leader)
    private final RelativeEncoder rightNeoEncoder = neoRight.getEncoder();
    private final SparkMax neoLeft = new SparkMax(Constants.CanID.SPINDEXER_LEFT, MotorType.kBrushless);  //turn CCW (follower)
    private final RelativeEncoder leftNeoEncoder = neoLeft.getEncoder();
    
    private final DoublePublisher leftNeoVelocityPublisher = NetworkTableInstance.getDefault().getDoubleTopic("/RBR/Spindexer/Velocity/Left").publish();
    private final DoublePublisher rightNeoVelocityPublisher = NetworkTableInstance.getDefault().getDoubleTopic("/RBR/Spindexer/Velocity/Right").publish();


    public Spindexer() {
        var rightRevConfig = new SparkMaxConfig();
        rightRevConfig.idleMode(IdleMode.kBrake);
        rightRevConfig.voltageCompensation(12.0);
        neoRight.configure(rightRevConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        var leftRevConfig = new SparkMaxConfig();
        leftRevConfig.idleMode(IdleMode.kBrake);
        leftRevConfig.voltageCompensation(12.0);
        leftRevConfig.follow(Constants.CanID.SPINDEXER_RIGHT, true);
        neoLeft.configure(leftRevConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    @Override
    public void periodic() {
        leftNeoVelocityPublisher.set(leftNeoEncoder.getVelocity());
        rightNeoVelocityPublisher.set(rightNeoEncoder.getVelocity());
    }

    //run duty cycle
    public void setDutyCycleSpeed(double speed) {
        neoRight.set(speed);
        neoLeft.set(speed);
    }

    public Command runAtDutyCycle(double speed) {
        return this.run(() -> setDutyCycleSpeed(speed));
    }

    public Command stop() {
        return this.runOnce(() -> setDutyCycleSpeed(0.0));
    }
}
