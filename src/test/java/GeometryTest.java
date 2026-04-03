import org.photonvision.PhotonUtils;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.units.Units;
import frc.robot.Constants;
import yams.gearing.GearBox;
import yams.gearing.MechanismGearing;

import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

import static org.junit.jupiter.api.Assertions.assertEquals;

public class GeometryTest {

    @Test
    public void testTargetRotationAngleFromPoses() {
        Pose2d robotPose = new Pose2d(Units.Meters.of(4.618), Units.Meters.of(4.039), Rotation2d.kCCW_90deg);  //facing north from blue origin
        Pose2d targetPose = new Pose2d(Units.Meters.of(11.914), Units.Meters.of(4.039), new Rotation2d());
        Rotation2d targetRotation = PhotonUtils.getYawToPose(robotPose, targetPose);
        double targetYawDegrees = targetRotation.getDegrees();
        assertEquals(-90.0, targetYawDegrees, 0.01, "Target yaw should be -90 degrees (90 degrees towards right/east) when target is directly east of robot facing north");
        System.out.println("Target Yaw Degrees: " + targetYawDegrees);
    }
    
    @Test public void testBoostVisionStdDeviations() {
        Matrix<N3, N1> originalStdDevs = Constants.Vision.DEFAULT_VISION_MEASUREMENT_STANDARD_DEVIATIONS;
        Matrix<N3, N1> boostedStdDevs = originalStdDevs.times(0.25); //boost by factor of 4

        System.out.println("Original StdDevs: " + originalStdDevs);
        System.out.println("Boosted StdDevs: " + boostedStdDevs);
    }

    @Test
    public void testGearBox() {
        GearBox gearBox = new GearBox(new String[]{"5.27:1"});
        System.out.println("Gearbox ratio: " + gearBox.getInputToOutputConversionFactor());
    }
}
