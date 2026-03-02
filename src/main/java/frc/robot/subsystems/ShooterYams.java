package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Volts;

import java.util.function.Supplier;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkAbsoluteEncoder;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.AlternateEncoderConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.MutAngle;
import edu.wpi.first.units.measure.MutAngularVelocity;
import edu.wpi.first.units.measure.MutVoltage;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants;
import yams.gearing.MechanismGearing;
import yams.mechanisms.config.FlyWheelConfig;
import yams.mechanisms.velocity.FlyWheel;
import yams.motorcontrollers.SmartMotorController;
import yams.motorcontrollers.SmartMotorControllerConfig;
import yams.motorcontrollers.SmartMotorControllerConfig.ControlMode;
import yams.motorcontrollers.SmartMotorControllerConfig.MotorMode;
import yams.motorcontrollers.SmartMotorControllerConfig.TelemetryVerbosity;
import yams.motorcontrollers.remote.TalonFXWrapper;

public class ShooterYams extends SubsystemBase {

    private final TalonFX shooterKrakenLeft = new TalonFX(Constants.CanID.SHOOTER_KRAKEN_LEFT, CANBus.roboRIO()); //follower
    private final TalonFX shooterKrakenRight = new TalonFX(Constants.CanID.SHOOTER_KRAKEN_RIGHT, CANBus.roboRIO());  //primary
    // private final SparkFlex shooterVortexLeft = new SparkFlex(Constants.CanID.SHOOTER_VORTEX_LEFT, MotorType.kBrushless);  //follower - negative rotation spins in the direction we want
    // private final RelativeEncoder leftVortexEncoder = shooterVortexLeft.getEncoder();
    // private final SparkMax shooterVortexRight = new SparkMax(Constants.CanID.SHOOTER_VORTEX_RIGHT, MotorType.kBrushless);  //primary - positive rotation spins in the direction we want
    // private final RelativeEncoder rightVortexEncoder = shooterVortexRight.getEncoder();
    private boolean defaultCommandIsStop = true;

    private final DoublePublisher rightKrakenVelocityPublisher = NetworkTableInstance.getDefault().getDoubleTopic("/RBR/Shooter/Kraken/Right/Velocity").publish();
    private final DoublePublisher rightKrakenVoltagePublisher = NetworkTableInstance.getDefault().getDoubleTopic("/RBR/Shooter/Kraken/Right/Voltage").publish();
    private final DoublePublisher leftKrakenVelocityPublisher = NetworkTableInstance.getDefault().getDoubleTopic("/RBR/Shooter/Kraken/Left/Velocity").publish();
    private final DoublePublisher leftKrakenVoltagePublisher = NetworkTableInstance.getDefault().getDoubleTopic("/RBR/Shooter/Kraken/Left/Voltage").publish();
    // private final DoublePublisher rightVortexVoltagePublisher = NetworkTableInstance.getDefault().getDoubleTopic("/RBR/Shooter/Vortex/Right/Voltage").publish();
    // private final DoublePublisher rightVortexVelocityPublisher = NetworkTableInstance.getDefault().getDoubleTopic("/RBR/Shooter/Vortex/Right/Velocity").publish();
    // private final DoublePublisher leftVortexVelocityPublisher = NetworkTableInstance.getDefault().getDoubleTopic("/RBR/Shooter/Vortex/Left/Velocity").publish();
    // private final DoublePublisher leftVortexVoltagePublisher = NetworkTableInstance.getDefault().getDoubleTopic("/RBR/Shooter/Vortex/Left/Voltage").publish();

    private final InterpolatingDoubleTreeMap rpmTable = new InterpolatingDoubleTreeMap();

    private final SmartMotorControllerConfig shooterKrakenConfig = new SmartMotorControllerConfig(this)
            .withControlMode(ControlMode.CLOSED_LOOP)
            .withFollowers(Pair.of(shooterKrakenLeft, true))
            // Feedback Constants (PID Constants)
            .withClosedLoopController(Constants.Shooter.CtrePidf.K_P, Constants.Shooter.CtrePidf.K_I,
                    Constants.Shooter.CtrePidf.K_D, Units.RadiansPerSecond.of(524.0),
                    Units.RadiansPerSecondPerSecond.of(1400.0))
            .withSimClosedLoopController(Constants.Shooter.CtrePidf.K_P, Constants.Shooter.CtrePidf.K_I,
                    Constants.Shooter.CtrePidf.K_D, Units.RadiansPerSecond.of(524.0),
                    Units.RadiansPerSecondPerSecond.of(1400.0))
            // Feedforward Constants
            .withFeedforward(new SimpleMotorFeedforward(Constants.Shooter.CtrePidf.K_S, Constants.Shooter.CtrePidf.K_V, Constants.Shooter.CtrePidf.K_A))
            .withSimFeedforward(new SimpleMotorFeedforward(Constants.Shooter.CtrePidf.K_S, Constants.Shooter.CtrePidf.K_V, Constants.Shooter.CtrePidf.K_A))
            // Telemetry name and verbosity level
            .withTelemetry("CtreShooterMotors", TelemetryVerbosity.HIGH)
            // Gearing from the motor rotor to final shaft - 1:1 (direct drive)
            .withGearing(new MechanismGearing(1))
            .withMotorInverted(false)
            .withIdleMode(MotorMode.COAST)
            // .withVoltageCompensation(Units.Volts.of(12)) //Note: we can't use this - apparently it's a Pro/paid feature
            // Motor properties to prevent over currenting.
            .withStatorCurrentLimit(Units.Amps.of(80));  //TODO: verify this limit is not being hit during operation using telemetry, and adjust if necessary

    private final SmartMotorController shooterKrakenSmartMotorController = new TalonFXWrapper(shooterKrakenRight, DCMotor.getKrakenX60(1), shooterKrakenConfig);

    private final FlyWheelConfig shooterKrakenShooterConfig = new FlyWheelConfig(shooterKrakenSmartMotorController)
            // Diameter of the flywheel.
            .withDiameter(Constants.Shooter.SHOOTER_WHEEL_DIAMETER)
            // Mass of the flywheel.
            .withMass(Constants.Shooter.FLYWHEEL_MASS)
            // Maximum speed of the shooter.
            .withUpperSoftLimit(Units.RPM.of(6380))  //Kraken x60 has a max speed of 6380 RPM
            // Telemetry name and verbosity for the arm.
            .withTelemetry("CtreShooter", TelemetryVerbosity.HIGH);

    // Shooter Mechanism
    private FlyWheel shooterKraken = new FlyWheel(shooterKrakenShooterConfig);

    // SysId setup for characterization
    private final VoltageOut ctreSysIdControl = new VoltageOut(0);
    private final MutAngle sysIdPosition = Units.Rotations.mutable(0);
    private final MutAngularVelocity sysIdVelocity = Units.RotationsPerSecond.mutable(0);
    private final MutVoltage sysIdAppliedVoltage = Volts.mutable(0);

    private boolean sysIdTestsStarted = false;

    private final SysIdRoutine unifiedSysIdRoutine = new SysIdRoutine(
        new SysIdRoutine.Config(
            null,         // Use default ramp rate (1 V/s)
            Volts.of(4), // Reduce dynamic voltage to 4 to prevent brownout
            null,          // Use default timeout (10 s)
                                // Log state with Phoenix SignalLogger class
            state -> SignalLogger.writeString("shooter-state", state.toString())
        ),
        new SysIdRoutine.Mechanism(
            volts -> {
                shooterKrakenLeft.setControl(ctreSysIdControl.withOutput(volts.unaryMinus().in(Volts)));  //negative voltage = clockwise
                shooterKrakenRight.setControl(ctreSysIdControl.withOutput(volts.in(Volts))); //positive voltage = clockwise
                // shooterVortexLeft.setVoltage(volts.unaryMinus().in(Volts));
                // shooterVortexRight.setVoltage(volts.in(Volts));
            },
            log -> {
                // Record a frame for the motors.
                log.motor("shooter-kraken-left")
                    .voltage(sysIdAppliedVoltage.mut_replace(shooterKrakenLeft.getMotorVoltage().getValueAsDouble(), Volts))
                    .angularPosition(sysIdPosition.mut_replace(shooterKrakenLeft.getRotorPosition().getValueAsDouble(), Units.Rotations))
                    .angularVelocity(sysIdVelocity.mut_replace(shooterKrakenLeft.getRotorVelocity().getValueAsDouble(), Units.RotationsPerSecond));
                log.motor("shooter-kraken-right")
                    .voltage(sysIdAppliedVoltage.mut_replace(shooterKrakenRight.getMotorVoltage().getValueAsDouble(), Volts))
                    .angularPosition(sysIdPosition.mut_replace(shooterKrakenRight.getRotorPosition().getValueAsDouble(), Units.Rotations))
                    .angularVelocity(sysIdVelocity.mut_replace(shooterKrakenRight.getRotorVelocity().getValueAsDouble(), Units.RotationsPerSecond));
                // log.motor("shooter-vortex-left")
                //     .voltage(sysIdAppliedVoltage.mut_replace(shooterVortexLeft.getAppliedOutput() * shooterVortexLeft.getBusVoltage(), Volts))
                //     .angularPosition(sysIdPosition.mut_replace(leftVortexEncoder.getPosition(), Units.Rotations))
                //     .angularVelocity(sysIdVelocity.mut_replace(leftVortexEncoder.getVelocity() / 60.0, Units.RotationsPerSecond));  //divide by 60 to convert RPM to RPS
                // log.motor("shooter-vortex-right")
                //     .voltage(sysIdAppliedVoltage.mut_replace(shooterVortexRight.getAppliedOutput() * shooterVortexRight.getBusVoltage(), Volts))
                //     .angularPosition(sysIdPosition.mut_replace(rightVortexEncoder.getPosition(), Units.Rotations))
                //     .angularVelocity(sysIdVelocity.mut_replace(rightVortexEncoder.getVelocity() / 60.0, Units.RotationsPerSecond));
            },
            this
        )
    );

    public ShooterYams() {
        //TODO: test and assign these values properly - those listed are placeholders
        rpmTable.put(2.0, 1500.0); // At 2m, 1500 RPM
        rpmTable.put(4.0, 2500.0); // At 4m, 2500 RPM
        rpmTable.put(6.0, 3000.0); // At 6m, 3000 RPM

        var sparkMaxConfig = new SparkMaxConfig();
        sparkMaxConfig.idleMode(IdleMode.kCoast);
        sparkMaxConfig.voltageCompensation(12.0);

        // var sparkFlexConfig = new SparkFlexConfig();
        // sparkFlexConfig.idleMode(IdleMode.kCoast);
        // sparkFlexConfig.follow(Constants.CanID.SHOOTER_VORTEX_RIGHT, true);
        // sparkFlexConfig.voltageCompensation(12.0);

        // shooterVortexLeft.configure(sparkFlexConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
        // shooterVortexRight.configure(sparkMaxConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
    }

    public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
        return startSysIdLogging().andThen(unifiedSysIdRoutine.quasistatic(direction));
    }

    public Command sysIdDynamic(SysIdRoutine.Direction direction) {
        return startSysIdLogging().andThen(unifiedSysIdRoutine.dynamic(direction));
    }

    public Command startSysIdLogging() {
        return runOnce(() -> {
            if (!sysIdTestsStarted) {
                SignalLogger.stop();
                sysIdTestsStarted = true;
                SignalLogger.start();
            }
        });
    }

    public Command stopSysIdLogging() {
        return runOnce(() -> {
            sysIdTestsStarted = false;
            SignalLogger.stop();
        });
    }

    @Override
    public void simulationPeriodic() {
        shooterKraken.simIterate();
    }

    @Override
    public void periodic() {
        rightKrakenVelocityPublisher.set(shooterKrakenRight.getVelocity().getValueAsDouble() / 60);  //Convert RPS to RPM
        rightKrakenVoltagePublisher.set(shooterKrakenRight.getMotorVoltage().getValueAsDouble());
        leftKrakenVelocityPublisher.set(shooterKrakenLeft.getVelocity().getValueAsDouble() / 60);
        leftKrakenVoltagePublisher.set(shooterKrakenLeft.getMotorVoltage().getValueAsDouble());

        // rightVortexVoltagePublisher.set(shooterVortexRight.getAppliedOutput() * shooterVortexRight.getBusVoltage());
        // rightVortexVelocityPublisher.set(rightVortexEncoder.getVelocity());
        // leftVortexVoltagePublisher.set(shooterVortexLeft.getAppliedOutput() * shooterVortexLeft.getBusVoltage());
        // leftVortexVelocityPublisher.set(leftVortexEncoder.getVelocity());
    }

    public void setShooterAngularVelocity(AngularVelocity rpmTarget) {
        shooterKraken.setMechanismVelocitySetpoint(rpmTarget);
        double leaderVolts = shooterKrakenRight.getMotorVoltage().getValueAsDouble();
        // shooterVortexLeft.setVoltage(-leaderVolts);  //left is follow, will auto apply voltage from primary
        //TODO: uncomment the below again when done  testing
        // shooterVortexRight.setVoltage(leaderVolts);
    }

    private void setBrakeModeForAllMotors(boolean isBrake) {
        var leftKrakenConfigurator = shooterKrakenLeft.getConfigurator();
        var leftKrakenConfigs = new TalonFXConfiguration();
        leftKrakenConfigurator.refresh(leftKrakenConfigs);  //read device settings from the motor and store them in the config object
        leftKrakenConfigs.MotorOutput.NeutralMode = isBrake ? NeutralModeValue.Brake : NeutralModeValue.Coast;  //modify the config object to set the desired brake mode
        leftKrakenConfigurator.apply(leftKrakenConfigs);  //write the updated config back to the motor

        var rightKrakenConfigurator = shooterKrakenRight.getConfigurator();
        var rightKrakenConfigs = new TalonFXConfiguration();
        rightKrakenConfigurator.refresh(rightKrakenConfigs);
        rightKrakenConfigs.MotorOutput.NeutralMode = isBrake ? NeutralModeValue.Brake : NeutralModeValue.Coast;
        rightKrakenConfigurator.apply(rightKrakenConfigs);

        // var sparkMaxConfig = new SparkMaxConfig();
        // sparkMaxConfig.idleMode(isBrake ? IdleMode.kBrake : IdleMode.kCoast);
        // var sparkFlexConfig = new SparkFlexConfig();
        // sparkFlexConfig.idleMode(isBrake ? IdleMode.kBrake : IdleMode.kCoast);
        // shooterVortexLeft.configure(sparkFlexConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
        // shooterVortexRight.configure(sparkMaxConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
    }

    public void stopShooter() {
        // setBrakeModeForAllMotors(true);
        shooterKraken.setMechanismVelocitySetpoint(Units.RPM.of(0));
        double krakenVolts = shooterKrakenRight.getMotorVoltage().getValueAsDouble();
        //TODO: uncomment the below again when done testing
        // shooterVortexLeft.setVoltage(krakenVolts);
        // shooterVortexRight.setVoltage(krakenVolts);
    }

    public Command prepVariableDistanceShot(Supplier<Distance> shotDistanceSupplier) {
        return startEnd(() -> runAtAngularVelocity(getTargetShooterAngularVelocityForDistance(shotDistanceSupplier.get())), () -> idleAtLowRpm());
    }

    public Command runAtAngularVelocity(AngularVelocity rpmTarget) {
        return this.run(() -> setShooterAngularVelocity(rpmTarget)).withName("Shooter at velocity: " + rpmTarget);
    }

    public Command idleAtLowRpm() {
        return runAtAngularVelocity(Units.RPM.of(1000)).withName("Idle Shooter");  //TODO: determine appropriate idle speed
    }

    public Command stop() {
        return this.run(() -> stopShooter()).withName("Stop Shooter");
    }

    private AngularVelocity getTargetShooterAngularVelocityForDistance(Distance distance) {
        return Units.RPM.of(rpmTable.get(distance.in(Units.Meters)));
    }

    //See https://blog.eeshwark.com/robotblog/shooting-on-the-fly
    public Command autoShoot(Supplier<Pose2d> robotPoseSupplier, Supplier<ChassisSpeeds> robotSpeedSupplier, Supplier<Distance> shotDistanceSupplier) {
        Distance targetDistance = shotDistanceSupplier.get();
        // 1. LATENCY COMP
        double latency = 0.15; // Tuned constant
        Pose2d robotPose = robotPoseSupplier.get();
        ChassisSpeeds robotSpeed = robotSpeedSupplier.get();
        Translation2d futurePos = robotPose.getTranslation().plus(
            new Translation2d(robotSpeed.vxMetersPerSecond, robotSpeed.vyMetersPerSecond).times(latency)
        );

        // 2. GET TARGET VECTOR
        Translation2d goalLocation = Constants.Game.getHubPose().getTranslation().toTranslation2d();
        Translation2d targetVec = goalLocation.minus(futurePos);
        double dist = targetVec.getNorm();

        // 3. CALCULATE IDEAL SHOT (Stationary)
        // Note: This returns HORIZONTAL velocity component
        
        AngularVelocity targetAngularVelocity = getTargetShooterAngularVelocityForDistance(targetDistance);
        double idealHorizontalSpeed = targetAngularVelocity.in(Units.RPM);

        // 4. VECTOR SUBTRACTION
        Translation2d robotVelVec = new Translation2d(robotSpeed.vxMetersPerSecond, robotSpeed.vyMetersPerSecond);
        Translation2d shotVec = targetVec.div(dist).times(idealHorizontalSpeed).minus(robotVelVec);

        // 5. CONVERT TO CONTROLS
        double turretAngleInDegrees = shotVec.getAngle().getDegrees();
        double newHorizontalSpeed = shotVec.getNorm();

        // 6. SOLVE FOR NEW PITCH/RPM
        // Assuming constant total exit velocity, variable hood:
        double totalExitVelocity = 15.0; // m/s
        // Clamp to avoid domain errors if we need more speed than possible
        double ratio = Math.min(newHorizontalSpeed / totalExitVelocity, 1.0);
        double newPitch = Math.acos(ratio);

        // 7. SET OUTPUTS

        //TODO: refactor this to match our actual mechanisms

        // turret.setAngle(turretAngleInDegrees);
        // hood.setAngle(Math.toDegrees(newPitch));
        // shooter.setRPM(calcRPM(totalExitVelocity));

        //TODO: wrap this in an actual command
        return Commands.none();
    }

    public boolean isDefaultCommandIsStop() {
        return defaultCommandIsStop;
    }

    public void setDefaultCommandIsStop(boolean defaultCommandIsStop) {
        this.defaultCommandIsStop = defaultCommandIsStop;
    }

    
}
