package frc.robot.vision;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
import java.util.function.Supplier;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFieldLayout.OriginPosition;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.apriltag.AprilTagFields;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.util.AlertManager;

// See latest photonvision examples for reference
// https://github.com/PhotonVision/photonvision/blob/master/photonlib-java-examples/poseest/src/main/java/frc/robot/Vision.java

/**
 * Photonvision based vision system - note we are NOT modeled as a Subsystem, since we don't want to block any commands
 * 
 */
public class VisionSystem {

    private final AprilTagFieldLayout fieldLayout;
    private final List<VisionCamera> visionCameras = new ArrayList<>();
    private final LinearFilter averageLatencyFilter = LinearFilter.movingAverage(40);
    private boolean receivedNewVisionData = false;
    private VisionEstimateConsumer visionEstimateConsumer;
    private final Supplier<Pose2d> currentRobotPoseSupplier;
    private DoublePublisher averageLatencyMsPublisher = NetworkTableInstance.getDefault().getDoubleTopic("/RBR/Vision/Latency").publish();
    private final StructArrayPublisher<Pose3d> acceptedTagPublisher = NetworkTableInstance.getDefault()
        .getStructArrayTopic("/RBR/Vision/AprilTags/Accepted", Pose3d.struct).publish();
    private final StructArrayPublisher<Pose3d> rejectedTagPublisher = NetworkTableInstance.getDefault()
        .getStructArrayTopic("/RBR/Vision/AprilTags/Rejected", Pose3d.struct).publish();
    private final StructArrayPublisher<Pose3d> acceptedVisionPosePublisher = NetworkTableInstance.getDefault()
        .getStructArrayTopic("/RBR/Vision/PoseEstimates/Accepted", Pose3d.struct).publish();
    private final StructArrayPublisher<Pose3d> rejectedVisionPosePublisher = NetworkTableInstance.getDefault()
        .getStructArrayTopic("/RBR/Vision/PoseEstimates/Rejected", Pose3d.struct).publish();
    // private final StructPublisher<Pose2d> frontCenterCameraPosePublisher = NetworkTableInstance.getDefault()
    //     .getStructTopic("/RBR/Vision/PoseEstimates/Camera/FC", Pose2d.struct).publish();
    // private final StructPublisher<Pose2d> frontRightCameraPosePublisher = NetworkTableInstance.getDefault()
    //     .getStructTopic("/RBR/Vision/PoseEstimates/Camera/FR", Pose2d.struct).publish();
    private final StructPublisher<Pose3d> backLeftCameraPoseEstimatePublisher = NetworkTableInstance.getDefault()
        .getStructTopic("/RBR/Vision/PoseEstimates/Camera/BL", Pose3d.struct).publish();
    private final StructPublisher<Pose3d> backRightCameraPoseEstimatePublisher = NetworkTableInstance.getDefault()
        .getStructTopic("/RBR/Vision/PoseEstimates/Camera/BR", Pose3d.struct).publish();
    private final StructPublisher<Pose3d> backCenterCameraPoseEstimatePublisher = NetworkTableInstance.getDefault()
        .getStructTopic("/RBR/Vision/PoseEstimates/Camera/BC", Pose3d.struct).publish();
    private final StructPublisher<Pose3d> backRightForwardCameraPoseEstimatePublisher = NetworkTableInstance.getDefault()
        .getStructTopic("/RBR/Vision/PoseEstimates/Camera/BRF", Pose3d.struct).publish();

    private final StructPublisher<Pose2d> backLeftCameraLocationPublisher = NetworkTableInstance.getDefault()
        .getStructTopic("/RBR/Vision/Locations/Camera/BL", Pose2d.struct).publish();
    private final StructPublisher<Pose2d> backRightCameraLocationPublisher = NetworkTableInstance.getDefault()
        .getStructTopic("/RBR/Vision/Locations/Camera/BR", Pose2d.struct).publish();
    private final StructPublisher<Pose2d> backCenterCameraLocationPublisher = NetworkTableInstance.getDefault()
        .getStructTopic("/RBR/Vision/Locations/Camera/BC", Pose2d.struct).publish();
        private final StructPublisher<Pose2d> backRightForwardCameraLocationPublisher = NetworkTableInstance.getDefault()
        .getStructTopic("/RBR/Vision/Locations/Camera/BRF", Pose2d.struct).publish();

    //Simulation
    private List<PhotonCameraSim> simulatedCameras = new ArrayList<>();
    private VisionSystemSim visionSim;

    //Links for troubleshooting / understanding:
    /*
     * https://docs.photonvision.org/en/latest/docs/apriltag-pipelines/index.html
     * https://docs.wpilib.org/en/stable/docs/software/advanced-controls/geometry/pose.html
     * https://www.chiefdelphi.com/t/trouble-with-cameratotarget-and-robot-pose-from-photonvision/454206
     * https://github.com/TexasTorque/TorqueLib/blob/master/sensors/TorqueVision.java
     */

    public VisionSystem(VisionEstimateConsumer visionEstimateConsumer, Supplier<Pose2d> currentRobotPoseSupplier) {
        this.visionEstimateConsumer = visionEstimateConsumer;
        this.currentRobotPoseSupplier = currentRobotPoseSupplier;
        //TODO: if the venue is not using the default welded field layout and is instead using AndyMark, update this to AprilTagFields.k2025ReefscapeAndyMark
        fieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);
        fieldLayout.setOrigin(OriginPosition.kBlueAllianceWallRightSide);

        // VisionCamera frontCenterCamera = new VisionCamera(Constants.Vision.CameraName.FRONT_CENTER, CameraPosition.FRONT_CENTER,
        //     Constants.Vision.CameraPose.FRONT_CENTER, fieldLayout);   
        // VisionCamera frontRightCamera = new VisionCamera(Constants.Vision.CameraName.FRONT_RIGHT, CameraPosition.FRONT_RIGHT,
        //     Constants.Vision.CameraPose.FRONT_RIGHT, fieldLayout);
        VisionCamera backRightCamera = new VisionCamera(Constants.Vision.CameraName.BACK_RIGHT, CameraPosition.BACK_RIGHT,
            Constants.Vision.CameraPose.BACK_RIGHT, fieldLayout);
        VisionCamera backLeftCamera = new VisionCamera(Constants.Vision.CameraName.BACK_LEFT, CameraPosition.BACK_LEFT,
            Constants.Vision.CameraPose.BACK_LEFT, fieldLayout);
        // VisionCamera backCenterCamera = new VisionCamera(Constants.Vision.CameraName.BACK_CENTER, CameraPosition.BACK_CENTER,
        //     Constants.Vision.CameraPose.BACK_CENTER, fieldLayout);
        VisionCamera backRightForwardCamera = new VisionCamera(Constants.Vision.CameraName.BACK_RIGHT_FORWARD, CameraPosition.BACK_RIGHT_FORWARD,
            Constants.Vision.CameraPose.BACK_RIGHT_FORWARD, fieldLayout);
        // visionCameras.add(frontCenterCamera);           
        // visionCameras.add(frontRightCamera);           
        visionCameras.add(backRightCamera);           
        visionCameras.add(backLeftCamera);
        // visionCameras.add(backCenterCamera);
        visionCameras.add(backRightForwardCamera);

        if (Robot.isSimulation()) {
            // Create the vision system simulation which handles cameras and targets on the field.
            visionSim = new VisionSystemSim("main");
            // Add all the AprilTags inside the tag layout as visible targets to this simulated field.
            visionSim.addAprilTags(fieldLayout);

            // Create simulated camera properties. These can be set to mimic your actual camera.
            var cameraProp = new SimCameraProperties();
            //assumes a 75 degree horizontal FOV
            //TODO: verify this resolution matches what we have configured on the Arducams in the Photonvision UI
            cameraProp.setCalibration(1280, 800, Rotation2d.fromDegrees(84.28));
            cameraProp.setCalibError(0.77, 0.25);
            cameraProp.setFPS(45); //TODO: verify this FPS matches what we see for the cameras in the Photonvision UI
            cameraProp.setAvgLatencyMs(30);  //TODO: update this to match the average latency we're publishing to NT at /RBR/Vision/Latency
            cameraProp.setLatencyStdDevMs(15);

            for (VisionCamera visionCamera : visionCameras) {
                // Create a PhotonCameraSim which will update the linked PhotonCamera's values with visible targets.
                PhotonCameraSim cameraSim = new PhotonCameraSim(visionCamera.getCameraInstance(), cameraProp);
                simulatedCameras.add(cameraSim);
                // Add the simulated camera to view the targets on this simulated field.
                visionSim.addCamera(cameraSim, visionCamera.getRobotToCameraPose());
                cameraSim.enableDrawWireframe(true);
            }
        } else {
            //Do not track cameras that are not actively connected at time of constructor initialization
            visionCameras.removeIf(c -> {
                boolean isDisconnected = !c.isCameraConnected();
                if (isDisconnected) {
                    AlertManager.addAlert(c.getCameraName(), "Camera (" + c.getCameraName() + ") disconnected!", AlertType.kWarning);
                } else {
                    AlertManager.removeAlert(c.getCameraName());
                    c.getCameraInstance().setPipelineIndex(Constants.Vision.APRIL_TAG_PIPELINE_INDEX);
                }

                return isDisconnected;
            });
        }
    }

    public void periodic() {
        //Publish camera positions to verify/visualize they are where we expect them to be in relation to the robot center
        Pose2d currentRobotPose = currentRobotPoseSupplier.get();
        if (currentRobotPose != null) {
            for (VisionCamera visionCamera : visionCameras) {
                Transform3d robotCenterToCameraTransform = visionCamera.getRobotToCameraPose();
                Translation2d translation2d = new Translation2d(robotCenterToCameraTransform.getX(), robotCenterToCameraTransform.getY());
                Rotation2d rotation2d = robotCenterToCameraTransform.getRotation().toRotation2d();

                Pose2d cameraFieldPose = currentRobotPose.transformBy(new Transform2d(translation2d, rotation2d));

                if (visionCamera.getCameraPosition() == CameraPosition.BACK_LEFT) {
                    backLeftCameraLocationPublisher.set(cameraFieldPose);
                } else if (visionCamera.getCameraPosition() == CameraPosition.BACK_RIGHT) {
                    backRightCameraLocationPublisher.set(cameraFieldPose);
                } else if (visionCamera.getCameraPosition() == CameraPosition.BACK_CENTER) {
                    backCenterCameraLocationPublisher.set(cameraFieldPose);
                } else if (visionCamera.getCameraPosition() == CameraPosition.BACK_RIGHT_FORWARD) {
                    backRightForwardCameraLocationPublisher.set(cameraFieldPose);
                }
            }
        }

        List<VisionPoseEstimationResult> newVisionPoseEstimates = getRobotPoseEstimationResults();
        receivedNewVisionData = !newVisionPoseEstimates.isEmpty();
        
        for (int i = 0; i < newVisionPoseEstimates.size(); i++) {
            VisionPoseEstimationResult poseEstimationResult = newVisionPoseEstimates.get(i);
            visionEstimateConsumer.consumeVisionPoseEstimate(poseEstimationResult);
        }
        
        if (Robot.isSimulation()) {
            getSimDebugField().getObject("VisionEstimation").setPoses(newVisionPoseEstimates.stream().map(p -> p.getEstimatedRobotPose().estimatedPose.toPose2d()).toList());
        } 
    }
 
    /**
     * Get position estimates of robot based on vision data (April Tag readings)
     * 
     * @return List of VisionPoseEstimationResult - one per camera that found a pose estimate
     */
    public List<VisionPoseEstimationResult> getRobotPoseEstimationResults() {
        List<VisionPoseEstimationResult> estimationResults = new ArrayList<>(visionCameras.size());
        List<EstimatedRobotPose> acceptedPoses = new ArrayList<>(visionCameras.size());
        List<EstimatedRobotPose> rejectedPoses = new ArrayList<>(visionCameras.size());
        List<Pose3d> usedAprilTags = new ArrayList<>();
        List<Pose3d> rejectedAprilTags = new ArrayList<>();

        for (int i = 0; i < visionCameras.size(); i++) {
            VisionCamera visionCamera = visionCameras.get(i);
            PhotonCamera photonCamera = visionCamera.getCameraInstance();
            PhotonPoseEstimator poseEstimator = visionCamera.getPoseEstimator();

            for (PhotonPipelineResult pipelineResult : photonCamera.getAllUnreadResults()) {
                processPhotonPipelineResult(pipelineResult, estimationResults, acceptedPoses, rejectedPoses, usedAprilTags,
                    rejectedAprilTags, visionCamera, poseEstimator);
            }
        }

        if (!usedAprilTags.isEmpty()) {
            acceptedTagPublisher.accept(usedAprilTags.toArray(new Pose3d[0]));
        }
        if (!rejectedAprilTags.isEmpty()) {
            rejectedTagPublisher.accept(rejectedAprilTags.toArray(new Pose3d[0]));
        }

        if (!acceptedPoses.isEmpty()) {
            List<Pose3d> acceptedPoses3dList = new ArrayList<>();
            for (EstimatedRobotPose pose : acceptedPoses) {
                acceptedPoses3dList.add(pose.estimatedPose);
            }
            acceptedVisionPosePublisher.accept(acceptedPoses3dList.toArray(new Pose3d[0]));
        }
        if (!rejectedPoses.isEmpty()) {
            List<Pose3d> rejectedPoses3dList = new ArrayList<>();
            for (EstimatedRobotPose pose : rejectedPoses) {
                rejectedPoses3dList.add(pose.estimatedPose);
            }
            rejectedVisionPosePublisher.accept(rejectedPoses3dList.toArray(new Pose3d[0]));
        }

        return estimationResults;
    }

    private void processPhotonPipelineResult(PhotonPipelineResult pipelineResult, List<VisionPoseEstimationResult> estimationResults, 
            List<EstimatedRobotPose> acceptedPoses, List<EstimatedRobotPose> rejectedPoses, 
            List<Pose3d> usedAprilTags, List<Pose3d> rejectedAprilTags, VisionCamera visionCamera, PhotonPoseEstimator poseEstimator) {
        List<Pose3d> usedAprilTagsForCamera = new ArrayList<>();
        List<Pose3d> rejectedAprilTagsForCamera = new ArrayList<>();

        if (pipelineResult.hasTargets()) {
            int numAprilTagsSeen = pipelineResult.getTargets().size();

            for (PhotonTrackedTarget target : pipelineResult.getTargets()) {
                //poseAmbiguity is between 0 and 1 (0 being no ambiguity, and 1 meaning both have the same reprojection error). Numbers above 0.2 are likely to be ambiguous. -1 if invalid.
                double poseAmbiguity = target.getPoseAmbiguity();
                boolean isUnambiguous = poseAmbiguity < Constants.Vision.POSE_AMBIGUITY_CUTOFF && poseAmbiguity >= 0;
                boolean isCloseEnough = target.getBestCameraToTarget().getTranslation().getNorm() < Constants.Vision.DISTANCE_CUTOFF;
                boolean shouldUseTarget = (isUnambiguous || numAprilTagsSeen > 1) && isCloseEnough;
                Optional<Pose3d> targetPose = fieldLayout.getTagPose(target.getFiducialId());
                if (targetPose.isPresent()) {
                    Pose3d tagPose = targetPose.get();

                    if (shouldUseTarget) {
                        usedAprilTagsForCamera.add(tagPose);
                    } else {
                        rejectedAprilTagsForCamera.add(tagPose);
                    }
                }
            }

            if (!usedAprilTagsForCamera.isEmpty()) {
                usedAprilTags.addAll(usedAprilTagsForCamera);
            }
            if (!rejectedAprilTagsForCamera.isEmpty()) {
                rejectedAprilTags.addAll(rejectedAprilTagsForCamera);
            }
        }

        Optional<EstimatedRobotPose> poseEstimateResult = poseEstimator.estimateCoprocMultiTagPose(pipelineResult);
        if (poseEstimateResult.isEmpty()) {
            //fallback to lowest ambiguity if multi-tag PNP on coprocessor did not return a result
            poseEstimateResult = poseEstimator.estimateLowestAmbiguityPose(pipelineResult);
        }
        
        if (poseEstimateResult.isPresent()) {
            EstimatedRobotPose poseEstimate = poseEstimateResult.get();
            double visionPoseTimestampSeconds = poseEstimate.timestampSeconds;
            long networkTablesPoseTimestampMicroSeconds = Math.round(visionPoseTimestampSeconds * 1000000); //microseconds

            double now = Timer.getFPGATimestamp();
            double observedLatencyMs = (now - visionPoseTimestampSeconds) * 1000.0; //this latency can be used to compute avg latency for photon camera simulation
            double runningAverageLatencyMs = averageLatencyFilter.calculate(observedLatencyMs);
            averageLatencyMsPublisher.accept(runningAverageLatencyMs);

            Pose3d estimatedPose = poseEstimate.estimatedPose;

            if (!usedAprilTagsForCamera.isEmpty()) {
                // Do not use pose if robot pose is off the field 
                if (estimatedPose.getX() < -Constants.Game.FIELD_POSE_XY_ERROR_MARGIN_METERS
                    || estimatedPose.getX() > fieldLayout.getFieldLength() + Constants.Game.FIELD_POSE_XY_ERROR_MARGIN_METERS
                    || estimatedPose.getY() < -Constants.Game.FIELD_POSE_XY_ERROR_MARGIN_METERS
                    || estimatedPose.getY() > fieldLayout.getFieldWidth() + Constants.Game.FIELD_POSE_XY_ERROR_MARGIN_METERS) {
                        rejectedPoses.add(poseEstimate);
                        return;
                }

                acceptedPoses.add(poseEstimate);
                Matrix<N3, N1> poseEstimateStandardDeviations = getVisionMeasurementStandardDeviation(poseEstimate, visionCamera);
                estimationResults.add(new VisionPoseEstimationResult(visionCamera, poseEstimate, poseEstimateStandardDeviations));

                if (visionCamera.getCameraPosition() == CameraPosition.BACK_LEFT) {
                    backLeftCameraPoseEstimatePublisher.set(estimatedPose, networkTablesPoseTimestampMicroSeconds);
                } else if (visionCamera.getCameraPosition() == CameraPosition.BACK_RIGHT) {
                    backRightCameraPoseEstimatePublisher.set(estimatedPose, networkTablesPoseTimestampMicroSeconds);
                } else if (visionCamera.getCameraPosition() == CameraPosition.BACK_CENTER) {
                    backCenterCameraPoseEstimatePublisher.set(estimatedPose, networkTablesPoseTimestampMicroSeconds);
                } else if (visionCamera.getCameraPosition() == CameraPosition.BACK_RIGHT_FORWARD) {
                    backRightForwardCameraPoseEstimatePublisher.set(estimatedPose, networkTablesPoseTimestampMicroSeconds);
                }
            } else {
                rejectedPoses.add(poseEstimate);
            }
        }
    }

    public Matrix<N3, N1> getVisionMeasurementStandardDeviation(EstimatedRobotPose estimation, VisionCamera visionCamera) {
        CameraPosition cameraPosition = visionCamera.getCameraPosition();

        if (Constants.Vision.USE_STATIC_STD_DEV) {
            if (cameraPosition == CameraPosition.FRONT_CENTER) {
                return Constants.Vision.FRONT_CAMERA_STANDARD_DEVIATIONS;
            } else {
                return Constants.Vision.OTHER_CAMERA_STANDARD_DEVIATIONS;
            }
        } else {
            //TODO: these std devs are def a little too aggressive

            //Logic based on https://github.com/PhotonVision/photonvision/blob/main/photonlib-java-examples/poseest/src/main/java/frc/robot/Vision.java
            var estStdDevs = Constants.Vision.SINGLE_TAG_MEASUREMENT_STANDARD_DEVIATIONS;
            
            int numTags = 0;
            double avgDist = 0;

            for (PhotonTrackedTarget target : estimation.targetsUsed) {
                var tagPose = visionCamera.getPoseEstimator().getFieldTags().getTagPose(target.getFiducialId());
                if (tagPose.isEmpty()) {
                    continue;
                } 
                numTags++;
                avgDist += tagPose.get().toPose2d().getTranslation().getDistance(estimation.estimatedPose.toPose2d().getTranslation());
            }

            if (numTags == 0) {
                return Constants.Vision.SINGLE_TAG_MEASUREMENT_STANDARD_DEVIATIONS;
            }

            avgDist /= numTags;

            // Decrease std devs for multitag
            if (numTags > 1) {
                estStdDevs = Constants.Vision.MULTI_TAG_MEASUREMENT_STANDARD_DEVIATIONS;
            } 

            // Reject single tag measurements beyond 4 meters
            if (numTags == 1 && avgDist > 4) {
                estStdDevs = VecBuilder.fill(Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE);
            } else {
                // Scale by distance squared
                estStdDevs = estStdDevs.times(1 + (avgDist * avgDist / 30));
            }

            return estStdDevs;
        }
        

        //Old logic below:
/*
        double smallestTargetDistance = Double.POSITIVE_INFINITY;  //in meters
        double smallestPoseAmbiguity = Double.POSITIVE_INFINITY;
        int numTargetsUsed = estimation.targetsUsed.size();
        boolean singleTarget = numTargetsUsed == 1;
        double poseAmbiguityFactor = 1;  //default is for the case of multiple targets

        for (PhotonTrackedTarget target : estimation.targetsUsed) {
            //x = forward, y = left, z = up - from the camera
            Transform3d transform3d = target.getBestCameraToTarget();
            double poseAmbiguity = target.getPoseAmbiguity();
            double distance = Math.sqrt(Math.pow(transform3d.getX(), 2) + Math.pow(transform3d.getY(), 2) + Math.pow(transform3d.getZ(), 2));
            if (distance < smallestTargetDistance) {
                smallestTargetDistance = distance;
            }
            if (poseAmbiguity < smallestPoseAmbiguity) {
                smallestPoseAmbiguity = poseAmbiguity;
            }
        }

        if (singleTarget) {
            double ambiguityScore = smallestPoseAmbiguity + Constants.Vision.POSE_AMBIGUITY_SHIFTER;
            poseAmbiguityFactor = Math.max(1, ambiguityScore * Constants.Vision.POSE_AMBIGUITY_MULTIPLIER);
            double targetDistanceAdjusted = Math.max(0, smallestTargetDistance - Constants.Vision.NOISY_DISTANCE_METERS);
            double weightedDistance = Math.max(1, targetDistanceAdjusted * Constants.Vision.DISTANCE_WEIGHT); //weighted distance (further distances will be trusted less than shorter)
            double targetScore = weightedDistance * poseAmbiguityFactor;
            double weightedTagPresence = (1 + ((numTargetsUsed - 1) * Constants.Vision.TAG_PRESENCE_WEIGHT));  //more targets = more confidence
            double confidenceMultiplier = Math.max(1, targetScore / weightedTagPresence);

            return Constants.Vision.DEFAULT_VISION_MEASUREMENT_STANDARD_DEVIATIONS.times(confidenceMultiplier);
        }

        //Note: higher values = trust the vision measurement less
        double xyStdDev = Constants.Vision.MULTI_TAG_XY_PER_TAG_STANDARD_DEVIATION * numTargetsUsed;
        double thetaStdDev = Constants.Vision.MULTI_TAG_THETA_STANDARD_DEVIATION;
        if (smallestPoseAmbiguity >= 0.05) {
            xyStdDev += smallestPoseAmbiguity;
            thetaStdDev += smallestPoseAmbiguity;
        }
        
        return VecBuilder.fill(xyStdDev, xyStdDev, thetaStdDev);
*/
    }

    public void takeRawImageSnapshot() {
        for (VisionCamera visionCamera : visionCameras) {
            if (visionCamera.isCameraConnected()) {
                visionCamera.getCameraInstance().takeInputSnapshot();
            }
        }
    }

    public void takeImageSnapshot() {
        for (VisionCamera visionCamera : visionCameras) {
            if (visionCamera.isCameraConnected()) {
                visionCamera.getCameraInstance().takeOutputSnapshot();
            }
        }
    }

    public void resetTelemetry() {
        backLeftCameraPoseEstimatePublisher.set(Pose3d.kZero);
        backRightCameraPoseEstimatePublisher.set(Pose3d.kZero);
        backCenterCameraPoseEstimatePublisher.set(Pose3d.kZero);
        backRightForwardCameraPoseEstimatePublisher.set(Pose3d.kZero);
    }

    public void simulationPeriodic(Pose2d robotSimPose) {
        visionSim.update(robotSimPose);
    }

    /** Reset pose history of the robot in the vision system simulation. */
    public void resetSimPose(Pose2d pose) {
        if (Robot.isSimulation()) visionSim.resetRobotPose(pose);
    }

    /** A Field2d for visualizing our robot and objects on the field. */
    public Field2d getSimDebugField() {
        if (!Robot.isSimulation()) return null;
        return visionSim.getDebugField();
    }

    public boolean isReceivedNewVisionData() {
        return receivedNewVisionData;
    }
}