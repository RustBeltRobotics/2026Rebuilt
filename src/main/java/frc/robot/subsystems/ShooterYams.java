package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Volts;

import java.util.function.Supplier;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.DoubleEntry;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
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
    private boolean defaultCommandIsStop = true;
    private boolean atTargetRpm = false;
    private double targetRpm;
    private final Trigger atRpmTrigger = new Trigger(() -> atTargetRpm);

    private final DoublePublisher rightKrakenVelocityPublisher = NetworkTableInstance.getDefault().getDoubleTopic("/RBR/Shooter/Kraken/Right/Rotor/Velocity").publish();
    private final DoublePublisher rightKrakenVoltagePublisher = NetworkTableInstance.getDefault().getDoubleTopic("/RBR/Shooter/Kraken/Right/Voltage").publish();
    private final DoublePublisher rightKrakenCurrentPublisher = NetworkTableInstance.getDefault().getDoubleTopic("/RBR/Shooter/Kraken/Right/Current").publish();
    private final DoublePublisher leftKrakenVelocityPublisher = NetworkTableInstance.getDefault().getDoubleTopic("/RBR/Shooter/Kraken/Left/Rotor/Velocity").publish();
    private final DoublePublisher leftKrakenVoltagePublisher = NetworkTableInstance.getDefault().getDoubleTopic("/RBR/Shooter/Kraken/Left/Voltage").publish();
    private final DoublePublisher leftKrakenCurrentPublisher = NetworkTableInstance.getDefault().getDoubleTopic("/RBR/Shooter/Kraken/Left/Current").publish();

    private final DoublePublisher shooterCurrentMechanismVelocityPublisher = NetworkTableInstance.getDefault().getDoubleTopic("/RBR/Shooter/Mechanism/Velocity/Current").publish();
    private final DoublePublisher shooterTargetMechanismVelocityPublisher = NetworkTableInstance.getDefault().getDoubleTopic("/RBR/Shooter/Mechanism/Velocity/Target").publish();
    private final DoublePublisher autoRpmCalculationPublisher = NetworkTableInstance.getDefault().getDoubleTopic("/RBR/Shooter/AutoDistance/RPM").publish();

    private final DoubleEntry tunableRpmEntry = NetworkTableInstance.getDefault().getTable("Tuning")
            .getDoubleTopic("Shooter/RPM")
            .getEntry(Constants.Shooter.SHOOTER_TEST_RPM.in(Units.RPM));

    private final BooleanPublisher atRpmPublisher = NetworkTableInstance.getDefault().getBooleanTopic("/RBR/Shooter/atRPM").publish();

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
            // Gearing from the motor rotor to final shaft - 1.66667:1 = Max Free speed of 3600 RPM (~377 rad/s)
            // .withGearing(new MechanismGearing(1.66667))
            .withGearing(new MechanismGearing(1.0))
            .withMotorInverted(false)
            .withIdleMode(MotorMode.COAST)
            // .withVoltageCompensation(Units.Volts.of(12)) //Note: we can't use this - apparently it's a Pro/paid feature
            // Motor properties to prevent over currenting.
            .withSupplyCurrentLimit(Units.Amps.of(70))
            .withStatorCurrentLimit(Units.Amps.of(70));  //TODO: verify this limit is not being hit during operation using telemetry, and adjust if necessary

    //TODO: use vendor config to potentially set min motor closed loop output to 0 to avoid driving flywheel backwords with PID

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
           null,
            this
        )
    );

    public ShooterYams() {
        //meters to RPM, Note: distance is measured from center of robot (drivetrain.getState().Pose) to center of hub
        tunableRpmEntry.setDefault(Constants.Shooter.SHOOTER_TEST_RPM.in(Units.RPM));
        //TODO: Add a few more measured values
        //measured values with laser distance tool: 1.3441m, 2.4811m
        // rpmTable.put(1.3212, Constants.Shooter.SHOOTER_LAYUP_RPM.in(Units.RPM));  //corresponds to 3.297, 4.067
        // rpmTable.put(2.3872, Constants.Shooter.SHOOTER_SHORT_DEFENSE_SHOT_RPM.in(Units.RPM));  //2.231, 4.003 (one full robot length back from the hub - i.e. short defensive shot)
        // rpmTable.put(3.4371, Constants.Shooter.SHOOTER_LONG_DEFENSE_SHOT_RPM.in(Units.RPM));  //1.181, 4.003 (two full robot length back from the hub - i.e. long defensive shot)
        // rpmTable.put(3.5744, Constants.Shooter.SHOOTER_TRENCH_RPM.in(Units.RPM)); //corresponds to 3.385, 0.684
        rpmTable.put(1.6764, 2550.0);
        rpmTable.put(2.2555, 2750.0);
        rpmTable.put(2.4600, 2800.0);
        rpmTable.put(2.7920, 3100.0);
        rpmTable.put(3.1181, 3350.0);
        rpmTable.put(3.3985, 3420.0);
        rpmTable.put(3.8191, 3580.0);
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
        double currentRpm = shooterKraken.getSpeed().in(Units.RPM);
        if (Constants.Game.ENABLE_DEBUG_NT_LOGGING) {
            rightKrakenVelocityPublisher.set(shooterKrakenRight.getRotorVelocity().getValueAsDouble() / 60);  //Convert RPS to RPM
            rightKrakenVoltagePublisher.set(shooterKrakenRight.getMotorVoltage().getValueAsDouble());
            rightKrakenCurrentPublisher.set(shooterKrakenRight.getSupplyCurrent().getValueAsDouble());
            leftKrakenVelocityPublisher.set(shooterKrakenLeft.getRotorVelocity().getValueAsDouble() / 60);
            leftKrakenVoltagePublisher.set(shooterKrakenLeft.getMotorVoltage().getValueAsDouble());
            leftKrakenCurrentPublisher.set(shooterKrakenLeft.getSupplyCurrent().getValueAsDouble());
            shooterCurrentMechanismVelocityPublisher.set(currentRpm);
            shooterTargetMechanismVelocityPublisher.set(targetRpm);
        }

        if (targetRpm != 0.0 && !atTargetRpm) {
            if ((Math.abs(targetRpm) - Math.abs(currentRpm)) <= 10.00) {
                atTargetRpm = true;
            }
        }

        if (Constants.Game.ENABLE_DEBUG_NT_LOGGING) {
            atRpmPublisher.set(atTargetRpm);
        }
    }

    public void setShooterAngularVelocity(AngularVelocity rpmTarget) {
        targetRpm = rpmTarget.in(Units.RPM);
        shooterKraken.setMechanismVelocitySetpoint(rpmTarget);
    }

    //clear prior target reached state
    public Command resetShooterAtTargetRpm() {
        return runOnce(() -> {
            targetRpm = 0.0;
            atTargetRpm = false;
        });
    }

    public void stopShooter() {
        targetRpm = 0.0;
        atTargetRpm = false;
        shooterKraken.setVoltageSetpoint(Units.Volts.of(0));
    }

    public Command variableDistanceShot(Supplier<Distance> shotDistanceSupplier) {
        return startEnd(() -> setShooterAngularVelocity(getTargetShooterAngularVelocityForDistance(shotDistanceSupplier.get())), () -> stopShooter());
    }

    public Command runAtAngularVelocity(AngularVelocity rpmTarget) {
        return this.run(() -> setShooterAngularVelocity(rpmTarget)).withName("Shooter at velocity: " + rpmTarget);
    }

    public Command runAtTunableAngularVelocity() {
        return this.run(() -> setShooterAngularVelocity(Units.RPM.of(getTunableRpm()))).withName("Shooter at velocity: " + getTunableRpm());
    }

    public Command idleAtLowRpm() {
        return runAtAngularVelocity(Units.RPM.of(1000)).withName("Idle Shooter");  //TODO: determine appropriate idle speed
    }

    public Command stop() {
        return this.run(() -> stopShooter()).withName("Stop Shooter");
    }

    public Command stopImmediately() {
        return this.run(() -> {
            targetRpm = 0.0;
            atTargetRpm = false;
            shooterKraken.setMechanismVelocitySetpoint(Units.RPM.of(0.0));
        });
    }

    private AngularVelocity getTargetShooterAngularVelocityForDistance(Distance distance) {
        double targetRpm = rpmTable.get(distance.in(Units.Meters));
        autoRpmCalculationPublisher.set(targetRpm);

        return Units.RPM.of(targetRpm);
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

    public Trigger getAtRpmTrigger() {
        return atRpmTrigger;
    }

    public double getTunableRpm() {
        return tunableRpmEntry.get();
    }

}
