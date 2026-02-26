package frc.robot.subsystems.drivetrain;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.VecBuilder;
import frc.robot.utils.Constants;

public class PoseEstimator extends SubsystemBase {

    private final Drivetrain drivetrain;

    private final Field2d field = new Field2d();

    private final SwerveDrivePoseEstimator poseEstimator;

    public PoseEstimator(Drivetrain drivetrain) {
        this.drivetrain = drivetrain;

        poseEstimator = new SwerveDrivePoseEstimator(
            Constants.DriveConstants.kDriveKinematics,
            drivetrain.getGyroRotation(),
            drivetrain.getModulePositions(),
            new Pose2d(),
            VecBuilder.fill(0.05, 0.05, Math.toRadians(1)),
            VecBuilder.fill(0.7, 0.7, Math.toRadians(10))
        );

        SmartDashboard.putData("Field", field);
    }

    @Override
    public void periodic() {
        poseEstimator.update(
            drivetrain.getGyroRotation(),
            drivetrain.getModulePositions()
        );

        field.setRobotPose(getPose());
    }

    public Pose2d getPose() {
        return poseEstimator.getEstimatedPosition();
    }

    public void resetPose(Pose2d pose) {
        poseEstimator.resetPosition(
            drivetrain.getGyroRotation(),
            drivetrain.getModulePositions(),
            pose
        );
    }

    public void addVisionMeasurement(Pose2d pose, double timestamp) {
        poseEstimator.addVisionMeasurement(pose, timestamp);
    }
}