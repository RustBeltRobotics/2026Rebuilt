package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Volts;

import java.util.function.DoubleSupplier;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.sim.SparkMaxSim;
import com.revrobotics.spark.SparkAbsoluteEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.AngularVelocity;
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
    private final RelativeEncoder relativeEncoder = hoodMotor.getEncoder();
    //throughbore is connected to the vortex right motor - a reference to the encoders will be passed via constructor
    // private RelativeEncoder throughBoreRelativeEncoder = hoodMotor.getAlternateEncoder();
    // private SparkAbsoluteEncoder throughBoreAbsoluteEncoder = hoodMotor.getAbsoluteEncoder();
    private final SimpleMotorFeedforward hoodAngleFeedforward;
    private final PIDController hoodAnglePidController;

    // private final DoublePublisher absoluteEncoderPositionPublisher = NetworkTableInstance.getDefault().getDoubleTopic("/RBR/Hood/Absolute/Position").publish();
    // private final DoublePublisher quadratureEncoderPositionPublisher = NetworkTableInstance.getDefault().getDoubleTopic("/RBR/Hood/Quadrature/Position").publish();
    private final DoublePublisher relativeEncoderPositionPublisher = NetworkTableInstance.getDefault().getDoubleTopic("/RBR/Hood/Relative/Position").publish();
    private final DoublePublisher currentPublisher = NetworkTableInstance.getDefault().getDoubleTopic("/RBR/Hood/Current").publish();

    private final MutVoltage sysIdAppliedVoltage = Volts.mutable(0);
    private final MutAngle sysIdPosition = Units.Rotations.mutable(0);
    private final MutAngularVelocity sysIdVelocity = Units.RotationsPerSecond.mutable(0);

    public ShooterHood() {
        //108:1 gear ratio, 360 degrees per revolution of the output shaft
        //5x5x7xZ = (175) * (48 large gear teeth = 12 small gear teeth) = 700:1
        // double conversionFactor = (1 / 700.0) * 360.0;  //0.5142857
        double conversionFactor = 360 / 700.0;
        // System.out.println("** Through bore quadrature encoder present - initial position = " + throughBoreRelativeEncoder.getPosition());

        hoodAngleFeedforward = new SimpleMotorFeedforward(Constants.ShooterHood.K_S, Constants.ShooterHood.K_V, Constants.ShooterHood.K_A);
        hoodAnglePidController = new PIDController(Constants.ShooterHood.K_P, Constants.ShooterHood.K_I, Constants.ShooterHood.K_D);

        var motorConfig = new SparkMaxConfig();
        motorConfig.idleMode(IdleMode.kBrake);
        motorConfig.voltageCompensation(12.0);
        // motorConfig.alternateEncoder.countsPerRevolution(8192).positionConversionFactor(conversionFactor).velocityConversionFactor(conversionFactor);
        motorConfig.encoder.positionConversionFactor(conversionFactor).velocityConversionFactor(conversionFactor);
        motorConfig.smartCurrentLimit(Constants.CurrentLimit.SparkMax.Neo550.SMART_DEFAULT).secondaryCurrentLimit(Constants.CurrentLimit.SparkMax.Neo550.SECONDARY_MAX);
        hoodMotor.configure(motorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        relativeEncoder.setPosition(0.0);
        // throughBoreRelativeEncoder.setPosition(0.0);  //Zero the through bore relative encoder on startup

        // if (throughBoreAbsoluteEncoder != null) {
        //     System.out.println("** Through bore absolute encoder present - position = " + throughBoreAbsoluteEncoder.getPosition());
        // } 

        hoodMechanismSim = new SingleJointedArmSim(
          DCMotor.getNeo550(1),
          108.0,  //108:1 reduction
          SingleJointedArmSim.estimateMOI(Units.Inches.of(8).in(Units.Meters), Units.Pounds.of(2).in(Units.Kilograms)),
          Units.Inches.of(8).in(Units.Meters),
          Units.Degrees.of(Constants.ShooterHood.MIN_DEGREES).in(Units.Radians),
          Units.Degrees.of(Constants.ShooterHood.MAX_DEGREES).in(Units.Radians),
          false,
          Units.Degrees.of(Constants.ShooterHood.MIN_DEGREES).in(Units.Radians)
      );
    }

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
                    .angularPosition(sysIdPosition.mut_replace(relativeEncoder.getPosition(), Units.Degrees))
                    .angularVelocity(sysIdVelocity.mut_replace(relativeEncoder.getVelocity() / 60.0, Units.DegreesPerSecond));
            },
            // Tell SysId to make generated commands require this subsystem, suffix test state in
            // WPILog with this subsystem's name
            this)
    );

    @Override
    public void periodic() {
        // if (throughBoreAbsoluteEncoder != null) {
        //     absoluteEncoderPositionPublisher.set(throughBoreAbsoluteEncoder.getPosition());
        // }
        // quadratureEncoderPositionPublisher.set(throughBoreRelativeEncoder.getPosition());
        relativeEncoderPositionPublisher.set(relativeEncoder.getPosition());
        currentPublisher.set(hoodMotor.getOutputCurrent());
    }
    
    @Override
    public void simulationPeriodic() {
        hoodMechanismSim.setInputVoltage(hoodMotorSim.getAppliedOutput() * RoboRioSim.getVInVoltage());

        hoodMechanismSim.update(0.02);

        // Update battery voltage simulation
        RoboRioSim.setVInVoltage(BatterySim.calculateDefaultBatteryLoadedVoltage(hoodMechanismSim.getCurrentDrawAmps()));
    }
    
    public void setAngle(double targetAngleInDegrees) {
        double currentAngle = getAngle();
        double feedforwardVolts = hoodAngleFeedforward.calculate(targetAngleInDegrees);
        double feedbackVolts = hoodAnglePidController.calculate(currentAngle, targetAngleInDegrees);
        hoodMotor.setVoltage(feedforwardVolts + feedbackVolts);
    }

    public double getAngle() {
        return relativeEncoder.getPosition();
    }

    public Command runToAngle(double targetAngleInDegrees) {
        return this.run(() -> setAngle(targetAngleInDegrees)).withName("Hood at angle: " + targetAngleInDegrees);
    }

    public Command runDutyCycle(DoubleSupplier speedSupplier) {
        return this.run(() -> hoodMotor.set(speedSupplier.getAsDouble())).withName("Hood run duty cycle");
    }

    public Command stop() {
        return this.run(() -> hoodMotor.stopMotor()).withName("Stop Hood");
    }

    public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
        return sysIdRoutine.quasistatic(direction);
    }

    public Command sysIdDynamic(SysIdRoutine.Direction direction) {
        return sysIdRoutine.dynamic(direction);
    }
}
