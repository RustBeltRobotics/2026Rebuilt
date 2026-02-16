package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Volts;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.sim.SparkMaxSim;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkAbsoluteEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.AbsoluteEncoderConfig;
import com.revrobotics.spark.config.ClosedLoopConfig;
import com.revrobotics.spark.config.FeedForwardConfig;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.MutAngle;
import edu.wpi.first.units.measure.MutAngularVelocity;
import edu.wpi.first.units.measure.MutVoltage;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants;

public class ShooterHood extends SubsystemBase {

    private final SparkMax hoodMotor = new SparkMax(Constants.CanID.SHOOTER_HOOD, MotorType.kBrushless);
    private final SparkMaxSim hoodMotorSim = new SparkMaxSim(hoodMotor, DCMotor.getNeo550(1));
    private final SingleJointedArmSim hoodMechanismSim;
    private final SparkClosedLoopController sparkPidController = hoodMotor.getClosedLoopController();
    private final RelativeEncoder relativeEncoder = hoodMotor.getAlternateEncoder();
    private final RelativeEncoder throughBoreRelativeEncoder = hoodMotor.getAlternateEncoder();
    private final SparkAbsoluteEncoder throughBoreAbsoluteEncoder = hoodMotor.getAbsoluteEncoder();
    private final DoublePublisher absoluteEncoderPositionPublisher = NetworkTableInstance.getDefault().getDoubleTopic("/RBR/Hood/Absolute/Position").publish();
    private final DoublePublisher quadratureEncoderPositionPublisher = NetworkTableInstance.getDefault().getDoubleTopic("/RBR/Hood/Quadrature/Position").publish();
    private final DoublePublisher relativeEncoderPositionPublisher = NetworkTableInstance.getDefault().getDoubleTopic("/RBR/Hood/Relative/Position").publish();

    private final MutVoltage sysIdAppliedVoltage = Volts.mutable(0);
    private final MutAngle sysIdPosition = Units.Rotations.mutable(0);
    private final MutAngularVelocity sysIdVelocity = Units.RotationsPerSecond.mutable(0);

    private final SysIdRoutine sysIdRoutine = new SysIdRoutine(
         new SysIdRoutine.Config(
            null,        // Use default ramp rate (1 V/s)
            Volts.of(4), // Reduce dynamic step voltage to 4 to prevent brownout
            null,        // Use default timeout (10 s)s
            null
        ),
        new SysIdRoutine.Mechanism(
            // Tell SysId how to plumb the driving voltage to the motors.
            (Voltage volts) -> {
                hoodMotor.setVoltage(volts.in(Volts));
            },
            // Tell SysId how to record a frame of data for each motor on the mechanism being
            // characterized.
            log -> {
                // Record a frame for the motors.
                log.motor("hood-motor")
                    .voltage(sysIdAppliedVoltage.mut_replace(hoodMotor.getAppliedOutput() * hoodMotor.getBusVoltage(), Volts))
                    .angularPosition(sysIdPosition.mut_replace(throughBoreRelativeEncoder.getPosition(), Units.Rotations))
                    .angularVelocity(sysIdVelocity.mut_replace(throughBoreRelativeEncoder.getVelocity() / 60.0, Units.RotationsPerSecond));
            },
            // Tell SysId to make generated commands require this subsystem, suffix test state in
            // WPILog with this subsystem's name
            this)
    );

    public ShooterHood() {
        AbsoluteEncoderConfig absoluteEncoderConfig = new AbsoluteEncoderConfig()
            .positionConversionFactor(360.0 / Constants.ShooterHood.MOTOR_ROTATIONS_PER_HOOD_ROTATION)  // hood degrees
            .velocityConversionFactor(360.0 / Constants.ShooterHood.MOTOR_ROTATIONS_PER_HOOD_ROTATION)
            .zeroOffset(Constants.ShooterHood.THROUGH_BORE_ZERO_OFFSET);
        ClosedLoopConfig pidConfig = new ClosedLoopConfig();
        FeedForwardConfig feedForwardConfig = new FeedForwardConfig();
        pidConfig.pid(Constants.ShooterHood.K_P, Constants.ShooterHood.K_I, Constants.ShooterHood.K_D);
        feedForwardConfig.kS(Constants.ShooterHood.K_S);
        feedForwardConfig.kV(Constants.ShooterHood.K_V);
        feedForwardConfig.kA(Constants.ShooterHood.K_A);
        var motorConfig = new SparkMaxConfig();
        motorConfig.idleMode(IdleMode.kBrake);
        motorConfig.voltageCompensation(12.0);
        motorConfig.absoluteEncoder.apply(absoluteEncoderConfig);
        motorConfig.alternateEncoder.countsPerRevolution(8192).positionConversionFactor(1 / 8192);  //quadrature
        motorConfig.closedLoop.apply(pidConfig);
        motorConfig.closedLoop.feedForward.apply(feedForwardConfig);
        motorConfig.closedLoop.feedbackSensor(FeedbackSensor.kAlternateOrExternalEncoder);  //use the through bore quadrature encoder as the feedback device for closed loop control 
        hoodMotor.configure(motorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        hoodMechanismSim = new SingleJointedArmSim(
          DCMotor.getNeo550(1),
          108.0,
          SingleJointedArmSim.estimateMOI(Units.Inches.of(8).in(Units.Meters), Units.Pounds.of(2).in(Units.Kilograms)),
          Units.Inches.of(8).in(Units.Meters),
          Units.Degrees.of(Constants.ShooterHood.MIN_DEGREES).in(Units.Radians),
          Units.Degrees.of(Constants.ShooterHood.MAX_DEGREES).in(Units.Radians),
          false,
          Units.Degrees.of(Constants.ShooterHood.MIN_DEGREES).in(Units.Radians)
      );
    }

    @Override
    public void periodic() {
        absoluteEncoderPositionPublisher.set(throughBoreAbsoluteEncoder.getPosition());
        quadratureEncoderPositionPublisher.set(throughBoreRelativeEncoder.getPosition());
        relativeEncoderPositionPublisher.set(relativeEncoder.getPosition());
    }
    
    @Override
    public void simulationPeriodic() {
        hoodMechanismSim.setInputVoltage(hoodMotorSim.getAppliedOutput() * RoboRioSim.getVInVoltage());

        hoodMechanismSim.update(0.02);

        // Update battery voltage simulation
        RoboRioSim.setVInVoltage(BatterySim.calculateDefaultBatteryLoadedVoltage(hoodMechanismSim.getCurrentDrawAmps()));
    }
    
    public void setAngle(double angleInDegrees) { 
        sparkPidController.setSetpoint(angleInDegrees, ControlType.kPosition);
    }

    public double getAngle() {
        return throughBoreRelativeEncoder.getPosition();
    }

    public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
        return sysIdRoutine.quasistatic(direction);
    }

    public Command sysIdDynamic(SysIdRoutine.Direction direction) {
        return sysIdRoutine.dynamic(direction);
    }
}
