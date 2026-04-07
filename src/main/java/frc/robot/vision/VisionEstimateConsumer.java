package frc.robot.vision;

@FunctionalInterface
public interface VisionEstimateConsumer {
    void consumeVisionPoseEstimate(VisionPoseEstimationResult visionPoseEstimationResult);
}
