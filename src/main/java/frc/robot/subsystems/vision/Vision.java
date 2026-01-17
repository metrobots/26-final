package frc.robot.subsystems.vision;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.Constants;
import frc.robot.utils.LimelightLib;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import java.util.Optional;

public class Vision extends SubsystemBase {
    // Cameras
    private final PhotonCamera frontCamera;
    private final PhotonCamera backCamera;
    private final String limelightName = Constants.AutoConstants.limelightName;

    // Limelight data
    private Pose3d targetPose3d;
    private double tX;
    private double tY;
    private boolean tV;
    
    // Pose estimators
    private final PhotonPoseEstimator frontPoseEstimator;
    private final PhotonPoseEstimator backPoseEstimator;
    
    // Field visualization
    private final Field2d field2d = Constants.AutoConstants.field2d;
    
    // Camera to robot transform
    private final Transform3d frontRobotToCamera = new Transform3d(
        0.0, 0.0, 0.5,  // x, y, z in meters (camera position relative to robot center)
        new Rotation3d(0, 0, Math.toRadians(45)) // rotation (45 degrees outwards)
    );
    
    private final Transform3d backRobotToCamera = new Transform3d(
        -0.5, 0.0, 0.5,  // x, y, z in meters (camera position relative to robot center)
        new Rotation3d(0, 0, Math.toRadians(-45)) // rotation (-45 degrees outwards (left))
    );
    
    // Last estimated poses
    private Optional<EstimatedRobotPose> lastFrontEstimatedPose = Optional.empty();
    private Optional<EstimatedRobotPose> lastBackEstimatedPose = Optional.empty();
    private Optional<Pose2d> lastLimelightPose = Optional.empty();
    
    public Vision() {
        // Initialize cameras
        frontCamera = new PhotonCamera("Front"); // Left
        backCamera = new PhotonCamera("Back"); // Right
        
        // Configure pose estimators with the current field layout
        AprilTagFieldLayout tagLayout = AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);

        // Initialize data
        this.tX = 0;
        this.tY = 0;
        this.tV = false;
        this.targetPose3d = new Pose3d();
        
        // Create pose estimators
        frontPoseEstimator = new PhotonPoseEstimator(
            tagLayout, 
            PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
            frontRobotToCamera
        );
        
        backPoseEstimator = new PhotonPoseEstimator(
            tagLayout,
            PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
            backRobotToCamera
        );
        
        // Set fallback strategy for single tag poses
        frontPoseEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);
        backPoseEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);
        
        // // Put field visualization on dashboard
        // SmartDashboard.putData("Field", field2d);
    }
    
    @Override
    public void periodic() {
        // Update pose estimations from both cameras and the backup pose estimation from the limelight
        updatePoseEstimates();
        updateLimelightPose();
        
        // Update the targeting data
        updateLimelightData();
        
        // Visualize latest vision measurements
        updateFieldVisualization();
    }


        /**
     * Returns the yaw (horizontal angle) to the best visible AprilTag target
     * from either the front or back camera, adjusted to robot-relative coordinates.
     *
     * @return Optional<Double> containing yaw in degrees, or empty if no target seen
     */
    public Optional<Double> getBestTargetYawFromCameras() {
        PhotonPipelineResult frontResult = frontCamera.getLatestResult();
        PhotonPipelineResult backResult = backCamera.getLatestResult();

        boolean hasFrontTargets = frontResult.hasTargets();
        boolean hasBackTargets = backResult.hasTargets();

        if (!hasFrontTargets && !hasBackTargets) {
            return Optional.empty();
        }

        PhotonTrackedTarget bestTarget;
        double adjustedYaw;

        // Prefer the camera with the better (lower ambiguity) target
        if (hasFrontTargets && hasBackTargets) {
            PhotonTrackedTarget frontTarget = frontResult.getBestTarget();
            PhotonTrackedTarget backTarget = backResult.getBestTarget();

            double frontAmbiguity = frontTarget.getPoseAmbiguity();
            double backAmbiguity = backTarget.getPoseAmbiguity();

            if (frontAmbiguity <= backAmbiguity) {
                bestTarget = frontTarget;
                adjustedYaw = bestTarget.getYaw() + 45.0; // Front camera is mounted at -45, so +45 to convert to robot frame
            } else {
                bestTarget = backTarget;
                adjustedYaw = bestTarget.getYaw() - 45.0; // Back camera is mounted at +45, so -45 to convert to robot frame
            }
        } else if (hasFrontTargets) {
            bestTarget = frontResult.getBestTarget();
            adjustedYaw = bestTarget.getYaw() + 45.0; // front is at -45 degrees
        } else {
            bestTarget = backResult.getBestTarget();
            adjustedYaw = bestTarget.getYaw() - 45.0; // back is at +45 degrees
        }

        return Optional.of(adjustedYaw);
    }

    
    /**
     * Get the latest estimated robot pose from the front camera
     * @return Optional containing the estimated pose if available
     */
    public Optional<EstimatedRobotPose> getFrontCameraEstimatedPose() {
        return lastFrontEstimatedPose;
    }
    
    /**
     * Get the latest estimated robot pose from the back camera
     * @return Optional containing the estimated pose if available
     */
    public Optional<EstimatedRobotPose> getBackCameraEstimatedPose() {
        return lastBackEstimatedPose;
    }
    
    /**
     * Get the latest best estimated robot pose from either camera
     * Prefers multi-tag solutions, then lowest ambiguity single-tag solutions
     * @return Optional containing the best estimated pose if available
     */
    public Optional<EstimatedRobotPose> getBestEstimatedPose() {
        boolean hasFront = lastFrontEstimatedPose.isPresent();
        boolean hasBack = lastBackEstimatedPose.isPresent();
        
        if (!hasFront && !hasBack) {
            return Optional.empty();
        } else if (hasFront && !hasBack) {
            return lastFrontEstimatedPose;
        } else if (!hasFront && hasBack) {
            return lastBackEstimatedPose;
        }
        
        // We have both poses - determine which is better
        
        // Get the results
        EstimatedRobotPose frontPose = lastFrontEstimatedPose.get();
        EstimatedRobotPose backPose = lastBackEstimatedPose.get();
        
        // Check if either has multiple targets (prefer multi-tag)
        boolean frontHasMulti = frontPose.targetsUsed.size() > 1;
        boolean backHasMulti = backPose.targetsUsed.size() > 1;
        
        if (frontHasMulti && !backHasMulti) {
            return lastFrontEstimatedPose;
        } else if (!frontHasMulti && backHasMulti) {
            return lastBackEstimatedPose;
        } else if (frontHasMulti && backHasMulti) {
            // Both have multiple targets - use the one with more targets
            if (frontPose.targetsUsed.size() > backPose.targetsUsed.size()) {
                return lastFrontEstimatedPose;
            } else {
                return lastBackEstimatedPose;
            }
        }
        
        // Both are single-tag - use the one with lowest ambiguity
        double frontAmbiguity = frontPose.targetsUsed.get(0).getPoseAmbiguity();
        double backAmbiguity = backPose.targetsUsed.get(0).getPoseAmbiguity();
        
        // Lower ambiguity is better
        if (frontAmbiguity < backAmbiguity) {
            return lastFrontEstimatedPose;
        } else {
            return lastBackEstimatedPose;
        }
    }
    
    /**
     * Update pose estimates from both cameras
     */
    private void updatePoseEstimates() {
        // Get poses from both cameras
        lastFrontEstimatedPose = frontPoseEstimator.update(frontCamera.getLatestResult());
        lastBackEstimatedPose = backPoseEstimator.update(backCamera.getLatestResult());
        
        // Log raw target data for debugging
        logRawTargetData();
    }

    /**
     * Updates the robot pose estimation from the Limelight
     */
    private void updateLimelightPose() {
        // Check if Limelight has valid targets
        if (LimelightLib.getTV(limelightName)) {
            // Get the robot pose from Limelight (in field space)
            double[] botpose = LimelightLib.getBotPose_wpiBlue(limelightName);
            
            if (botpose.length >= 6) {
                // Extract the pose components
                double x = botpose[0];
                double y = botpose[1];
                // double z = botpose[2];
                // double roll = botpose[3];
                // double pitch = botpose[4];
                double yaw = botpose[5];
                
                // Create the pose objects
                Pose2d pose2d = new Pose2d(x, y, Rotation2d.fromDegrees(yaw));
                
                // Update the last pose and latency
                lastLimelightPose = Optional.of(pose2d);
                
                // Visualize the limelight pose
                field2d.getObject("Limelight Estimate").setPose(pose2d);
                
                // Log data
                // SmartDashboard.putNumber("Limelight Pose X", x);
                // SmartDashboard.putNumber("Limelight Pose Y", y);
                // SmartDashboard.putNumber("Limelight Pose Yaw", yaw);
            } else {
                lastLimelightPose = Optional.empty();
            }
        } else {
            lastLimelightPose = Optional.empty();
        }
    }
    
    /**
     * Get the latest estimated robot pose from the Limelight
     * @return Optional containing the estimated pose if available
     */
    public Optional<Pose2d> getLimelightEstimatedPose() {
        return lastLimelightPose;
    }

    /**
     * Log raw target data from cameras for debugging
     */
    private void logRawTargetData() {
        // Front camera
        PhotonPipelineResult frontResult = frontCamera.getLatestResult();
        if (frontResult.hasTargets()) {
            SmartDashboard.putNumber("Front camera targets", frontResult.getTargets().size());
            PhotonTrackedTarget bestTarget = frontResult.getBestTarget();
            SmartDashboard.putNumber("Front best target ID", bestTarget.getFiducialId());
            SmartDashboard.putNumber("Front best target ambiguity", bestTarget.getPoseAmbiguity());
        } else {
            SmartDashboard.putNumber("Front camera targets", 0);
        }
        
        // Back camera
        PhotonPipelineResult backResult = backCamera.getLatestResult();
        if (backResult.hasTargets()) {
            SmartDashboard.putNumber("Back camera targets", backResult.getTargets().size());
            PhotonTrackedTarget bestTarget = backResult.getBestTarget();
            SmartDashboard.putNumber("Back best target ID", bestTarget.getFiducialId());
            SmartDashboard.putNumber("Back best target ambiguity", bestTarget.getPoseAmbiguity());
        } else {
            SmartDashboard.putNumber("Back camera targets", 0);
        }
    }
    
    /**
     * Update field visualization for debugging
     */
    private void updateFieldVisualization() {
        // Visualize front camera estimate
        if (lastFrontEstimatedPose.isPresent()) {
            Pose3d pose3d = lastFrontEstimatedPose.get().estimatedPose;
            Pose2d pose2d = new Pose2d(pose3d.getX(), pose3d.getY(), pose3d.getRotation().toRotation2d());
            field2d.getObject("Front Camera Estimate").setPose(pose2d);
        }
        
        // Visualize back camera estimate
        if (lastBackEstimatedPose.isPresent()) {
            Pose3d pose3d = lastBackEstimatedPose.get().estimatedPose;
            Pose2d pose2d = new Pose2d(pose3d.getX(), pose3d.getY(), pose3d.getRotation().toRotation2d());
            field2d.getObject("Back Camera Estimate").setPose(pose2d);
        }
        
        // Visualize best estimate
        Optional<EstimatedRobotPose> bestPose = getBestEstimatedPose();
        if (bestPose.isPresent()) {
            Pose3d pose3d = bestPose.get().estimatedPose;
            Pose2d pose2d = new Pose2d(pose3d.getX(), pose3d.getY(), pose3d.getRotation().toRotation2d());
            field2d.getObject("Best Vision Estimate").setPose(pose2d);
        }
    }

    public void updateLimelightData() {
        tX = LimelightLib.getTX(limelightName);
        tY = LimelightLib.getTY(limelightName);
        tV = LimelightLib.getTV(limelightName);
        targetPose3d = LimelightLib.getTargetPose3d_CameraSpace(limelightName);
    }

    public double getTX() {
        return tX;
    }

    public double getTY() {
        return tY;
    }

    public boolean getTV() {
        return tV;
    }

    public Pose3d getPose() {
        return targetPose3d;
    }
}