package frc.robot.subsystems.vision;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d; 
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.Constants;
import frc.robot.utils.LimelightLib;
import frc.robot.utils.LimelightLib.PoseEstimate;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.LLStatus;
import com.qualcomm.hardware.limelightvision.Limelight3A;

import java.util.List;
import java.util.Optional;

public class Vision extends SubsystemBase {
    //Cameras
    private PhotonCamera leftCamera; // Left Camera
    private PhotonCamera rightCamera; // Right Camera
    
    // Limelight Stuff
    private Limelight3A limelight1;
    private Limelight3A limelight2;

    private Pose3d targetPose3d;
    private Pose2d раньшеLimelightPose; // past limelight pose
    private Pose2d сейчасLimelightPose; // current limelight pose

    private double tX;
    private double tY;
    private boolean tV;


    // Pose estimators
    private final PhotonPoseEstimator leftPoseEstimator;
    private final PhotonPoseEstimator rightPoseEstimator;
        
        /*
         * Limelight position estimation?
         */

    // Field visualization
    private final Field2d field2d = Constants.AutoConstants.field2d;
    
    // Camera to robot transform
    private final Transform3d leftRobotToCamera = new Transform3d(
        0.0, 0.0, 0.5,  // x, y, z in meters (camera position relative to robot center). The actual distance left/right is 0.3302
        new Rotation3d(0, 0, Math.toRadians(45)) // rotation (45 degrees outwards)
    );
    
    private final Transform3d rightRobotToCamera = new Transform3d(
        -0.5, 0.0, 0.5,  // x, y, z in meters (camera position relative to robot center)
        new Rotation3d(0, 0, Math.toRadians(-45)) // rotation (-45 degrees outwards (left))
    );
    

    // Last estimated poses
    private Optional<EstimatedRobotPose> lastLeftEstimatedPose = Optional.empty();
    private Optional<EstimatedRobotPose> lastRightEstimatedPose = Optional.empty();
    private Optional<Pose2d> lastLimelightPose = Optional.empty();

    public Vision () {

        // initializes data apparently
        this.tX = 0;
        this.tY = 0;
        this.tV = false;

        // Camera Definitions
        leftCamera = new PhotonCamera("front"); // Left Camera
        rightCamera = new PhotonCamera("back"); // Right Camera

        limelight1 = new LimelightLib(); // Limelight 1
        limelight2 = new LimelightLib(); // Limelight 2

        // pose creation of some sort.
        this.targetPose3d = new Pose3d();
        this.раньшеLimelightPose = new Pose2d();
        this.сейчасLimelightPose = new Pose2d();

         // Configure pose estimators with the current field layout
        AprilTagFieldLayout tagLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltAndymark);

        // Create pose estimators
        leftPoseEstimator = new PhotonPoseEstimator(
            tagLayout, 
            PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
            leftRobotToCamera
        );
        
        rightPoseEstimator = new PhotonPoseEstimator(
            tagLayout,
            PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
            rightRobotToCamera
        );
        
        //  Set fallback strategy for single tag poses
        leftPoseEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);
        rightPoseEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);
    }

     @Override
    public void periodic() {
        // Update pose estimations from both cameras
        updatePoseEstimates();
        
        // Visualize latest vision measurements
        updateFieldVisualization();
    }

          /**
     * Returns the yaw (horizontal angle) to the best visible AprilTag target
     * from either the left or right camera, adjusted to robot-relative coordinates.
     *
     * @return Optional<Double> containing yaw in degrees, or empty if no target seen
     */
    public Optional<Double> getBestTargetYawFromCameras() {
        PhotonPipelineResult leftResult = leftCamera.getLatestResult();
        PhotonPipelineResult rightResult = rightCamera.getLatestResult();

        boolean hasLeftTargets = leftResult.hasTargets();
        boolean hasRightTargets = rightResult.hasTargets();

        if (!hasLeftTargets && !hasRightTargets) {
            return Optional.empty();
        }

        PhotonTrackedTarget bestTarget;
        double adjustedYaw;

        // Prefer the camera with the better (lower ambiguity) target
        if (hasLeftTargets && hasRightTargets) {
            PhotonTrackedTarget leftTarget = leftResult.getBestTarget();
            PhotonTrackedTarget rightTarget = rightResult.getBestTarget();

            double leftAmbiguity = leftTarget.getPoseAmbiguity();
            double rightAmbiguity = rightTarget.getPoseAmbiguity();

            if (leftAmbiguity <= rightAmbiguity) {
                bestTarget = leftTarget;
                adjustedYaw = bestTarget.getYaw() + 45.0; // Left camera is mounted at -45, so +45 to convert to robot frame
            } else {
                bestTarget = rightTarget;
                adjustedYaw = bestTarget.getYaw() - 45.0; // Right camera is mounted at +45, so -45 to convert to robot frame
            }
        } else if (hasLeftTargets) {
            bestTarget = leftResult.getBestTarget();
            adjustedYaw = bestTarget.getYaw() + 45.0; // Left is at -45 degrees
        } else {
            bestTarget = rightResult.getBestTarget();
            adjustedYaw = bestTarget.getYaw() - 45.0; // Right is at +45 degrees
        }

        return Optional.of(adjustedYaw);
    }

    
    /**
     * Get the latest estimated robot pose from the left camera
     * @return Optional containing the estimated pose if available
     */
    public Optional<EstimatedRobotPose> getLeftCameraEstimatedPose() {
        return lastLeftEstimatedPose;
    }
    
    /**
     * Get the latest estimated robot pose from the right camera
     * @return Optional containing the estimated pose if available
     */
    public Optional<EstimatedRobotPose> getRightCameraEstimatedPose() {
        return lastRightEstimatedPose;
    }
    
    /**
     * Get the latest best estimated robot pose from either camera
     * Prefers multi-tag solutions, then lowest ambiguity single-tag solutions
     * @return Optional containing the best estimated pose if available
     */
    public Optional<EstimatedRobotPose> getBestEstimatedPose() {
        boolean hasRight = lastRightEstimatedPose.isPresent();
        boolean hasLeft = lastLeftEstimatedPose.isPresent();
        
        if (!hasLeft && !hasRight) {
            return Optional.empty();
        } else if (hasLeft && !hasRight) {
            return lastLeftEstimatedPose;
        } else if (!hasLeft && hasRight) {
            return lastRightEstimatedPose;
        }
        
        // We have both poses - determine which is better
        
        // Get the results
        EstimatedRobotPose leftPose = lastLeftEstimatedPose.get();
        EstimatedRobotPose rightPose = lastRightEstimatedPose.get();
        
        // Check if either has multiple targets (prefer multi-tag)
        boolean leftHasMulti = leftPose.targetsUsed.size() > 1;
        boolean rightHasMulti = rightPose.targetsUsed.size() > 1;
        
        if (leftHasMulti && !rightHasMulti) {
            return lastLeftEstimatedPose;
        } else if (!leftHasMulti && rightHasMulti) {
            return lastRightEstimatedPose;
        } else if (rightHasMulti && leftHasMulti) {
            // Both have multiple targets - use the one with more targets
            if (leftPose.targetsUsed.size() > rightPose.targetsUsed.size()) {
                return lastLeftEstimatedPose;
            } else {
                return lastRightEstimatedPose;
            }
        }
        
        // Both are single-tag - use the one with lowest ambiguity
        double leftAmbiguity = leftPose.targetsUsed.get(0).getPoseAmbiguity();
        double rightAmbiguity = rightPose.targetsUsed.get(0).getPoseAmbiguity();
        
        // Lower ambiguity is better
        if (leftAmbiguity < rightAmbiguity) {
            return lastLeftEstimatedPose;
        } else {
            return lastRightEstimatedPose;
        }
    }
    
    /**
     * Update pose estimates from both cameras
     */
    private void updatePoseEstimates() {
        // Get poses from both cameras
        lastLeftEstimatedPose = leftPoseEstimator.update(leftCamera.getLatestResult());
        lastRightEstimatedPose = rightPoseEstimator.update(rightCamera.getLatestResult());
        
        // Log raw target data for debugging
        logRawTargetData();
    }
    

    /**
     * Log raw target data from cameras for debugging
     */
    private void logRawTargetData() {
        // Left camera
        PhotonPipelineResult leftResult = leftCamera.getLatestResult();
        if (leftResult.hasTargets()) {
            SmartDashboard.putNumber("Left camera targets", leftResult.getTargets().size());
            PhotonTrackedTarget bestTarget = leftResult.getBestTarget();
            SmartDashboard.putNumber("Left best target ID", bestTarget.getFiducialId());
            SmartDashboard.putNumber("Left best target ambiguity", bestTarget.getPoseAmbiguity());
        } else {
            SmartDashboard.putNumber("Left camera targets", 0);
        }
        
        // Right camera
        PhotonPipelineResult rightResult = rightCamera.getLatestResult();
        if (rightResult.hasTargets()) {
            SmartDashboard.putNumber("Right camera targets", rightResult.getTargets().size());
            PhotonTrackedTarget bestTarget = rightResult.getBestTarget();
            SmartDashboard.putNumber("Right best target ID", bestTarget.getFiducialId());
            SmartDashboard.putNumber("Right best target ambiguity", bestTarget.getPoseAmbiguity());
        } else {
            SmartDashboard.putNumber("Right camera targets", 0);
        }
    }
    
    /**
     * Update field visualization for debugging
     */
    private void updateFieldVisualization() {
        // Visualize left camera estimate
        if (lastLeftEstimatedPose.isPresent()) {
            Pose3d pose3d = lastLeftEstimatedPose.get().estimatedPose;
            Pose2d pose2d = new Pose2d(pose3d.getX(), pose3d.getY(), pose3d.getRotation().toRotation2d());
            field2d.getObject("Left Camera Estimate").setPose(pose2d);
        }
        
        // Visualize right camera estimate
        if (lastRightEstimatedPose.isPresent()) {
            Pose3d pose3d = lastRightEstimatedPose.get().estimatedPose;
            Pose2d pose2d = new Pose2d(pose3d.getX(), pose3d.getY(), pose3d.getRotation().toRotation2d());
            field2d.getObject("Right Camera Estimate").setPose(pose2d);
        }
        
        // Visualize best estimate
        Optional<EstimatedRobotPose> bestPose = getBestEstimatedPose();
        if (bestPose.isPresent()) {
            Pose3d pose3d = bestPose.get().estimatedPose;
            Pose2d pose2d = new Pose2d(pose3d.getX(), pose3d.getY(), pose3d.getRotation().toRotation2d());
            field2d.getObject("Best Vision Estimate").setPose(pose2d);
        }
    }

    public void updateLimelightPose(Limelight3A limelight) {

        // ...
        if (LimelightLib.getTv(limelight)) {
            
            double[] botpose = LimelightLib.getBotPose(limelight);
            
            раньшеLimelightPose = сейчасLimelightPose;

            сейчасLimelightPose = new Pose2d(botpose.getX() - 0, botpose.getY() - 0);
            
        }
    }

    public void updateLimelightData(Limelight3A limelight) {

        tX = LimelightLib.getTX(limelight);
        tY = LimelightLib.getTY(limelight);
        tV = LimelightLib.getTV(limelight);
        targetPose3d = LimelightLib.getTargetPose3d_CameraSpace();
        
    } 
}