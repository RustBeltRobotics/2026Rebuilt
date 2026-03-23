package frc.robot.commands;

import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.Command;

public class RotateAzimuthCommand extends Command {

    private final TalonFX steerMotor;
    private final TalonFX driveMotor;
    private final double rotations;
    private final PositionVoltage positionRequest = new PositionVoltage(0);
    private double steerMotorStartPosition;
    private double driveMotorStartPosition;

    /**
     * Rotates a swerve module's azimuth (steer motor) a given number of rotations.
     * Used for measuring kCoupleRatio by observing how much the drive encoder moves.
     *
     * @param steerMotor The steer TalonFX for the module you want to rotate
     * @param driveMotor The drive TalonFX for the same module (used to observe encoder changes)
     * @param rotations  Number of azimuth rotations (e.g. 3.0)
     */
    public RotateAzimuthCommand(TalonFX steerMotor, TalonFX driveMotor, double rotations) {
        this.steerMotor = steerMotor;
        this.driveMotor = driveMotor;
        this.rotations = rotations;
    }

    @Override
    public void initialize() {
        steerMotorStartPosition = steerMotor.getPosition().getValueAsDouble();
        driveMotorStartPosition = driveMotor.getPosition().getValueAsDouble();
        positionRequest.Position = steerMotorStartPosition + (rotations * 26.09090909090909);  //TunerConstants.kSteerGearRatio
        steerMotor.setControl(positionRequest);
    }

    @Override
    public void execute() {
        // Position control is handled by the motor controller onboard PID,
        // nothing to do here each loop
    }

    @Override
    public boolean isFinished() {
        double error = Math.abs(
            steerMotor.getPosition().getValueAsDouble() - positionRequest.Position
        );
        return error < 0.5; // within 0.5 motor rotations of target
    }

    @Override
    public void end(boolean interrupted) {
        steerMotor.stopMotor();
        System.out.println("Azimuth rotation complete.");
        System.out.println("Steer motor start position: "
            + steerMotorStartPosition);
        System.out.println("Steer motor final position: "
            + steerMotor.getPosition().getValueAsDouble());
        System.out.println("Drive motor start position: "
            + driveMotorStartPosition);
        System.out.println("Drive motor final position: "
            + driveMotor.getPosition().getValueAsDouble());
        // The drive motor encoder change divided by the number of azimuth rotations gives us the kCoupleRatio for the CTRE swerve modules
        double driveMotorChange = driveMotor.getPosition().getValueAsDouble() - driveMotorStartPosition;
        double kCoupleRatio = driveMotorChange / rotations;
        System.out.println("kCoupleRatio: " + kCoupleRatio);
    }
}