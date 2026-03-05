package frc.robot.subsystems.drivetrain;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.studica.frc.AHRS;
import com.studica.frc.AHRS.NavXComType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.*;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.FieldObject2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.subsystems.turret.Turret;
import frc.robot.utils.Constants.AutoConstants;
import frc.robot.utils.Constants.DriveConstants;
import frc.robot.utils.LimelightLib;
import frc.robot.utils.LimelightLib.PoseEstimate;

public class Drivetrain extends SubsystemBase {

    private final Turret m_turret;
    private static final String LIMELIGHT_NAME = "limelight-front";

    /* ================= FIELD CONSTANTS ================= */

    private static final double FIELD_LENGTH = 16.54;
    private static final double FIELD_WIDTH = 8.21;

    private static final double LEFT_THIRD_ENDPOINT = FIELD_LENGTH / 3.0;

    private static final Translation2d FIELD_CENTER =
            new Translation2d(FIELD_LENGTH / 2.0, FIELD_WIDTH / 2.0);

    /* ================= FIELD DISPLAY ================= */

    private final Field2d field = new Field2d();
    private final FieldObject2d centerVector;

    /* ================= STATE ================= */

    private Pose2d currentPose = new Pose2d();

    /* ================= MODULES ================= */

    private final Module m_frontLeft = new Module(
            DriveConstants.kFrontLeftDrivingCanId,
            DriveConstants.kFrontLeftTurningCanId,
            DriveConstants.kFrontLeftEncoder,
            DriveConstants.kFrontLeftEncoderOffset);

    private final Module m_frontRight = new Module(
            DriveConstants.kFrontRightDrivingCanId,
            DriveConstants.kFrontRightTurningCanId,
            DriveConstants.kFrontRightEncoder,
            DriveConstants.kFrontRightEncoderOffset);

    private final Module m_rearLeft = new Module(
            DriveConstants.kRearLeftDrivingCanId,
            DriveConstants.kRearLeftTurningCanId,
            DriveConstants.kRearLeftEncoder,
            DriveConstants.kRearLeftEncoderOffset);

    private final Module m_rearRight = new Module(
            DriveConstants.kRearRightDrivingCanId,
            DriveConstants.kRearRightTurningCanId,
            DriveConstants.kRearRightEncoder,
            DriveConstants.kRearRightEncoderOffset);

    private final AHRS m_gyro = new AHRS(NavXComType.kMXP_SPI);

    private final PIDController headingCorrector = new PIDController(0, 0, 0);

    private final SlewRateLimiter xLimiter =
            new SlewRateLimiter(AutoConstants.kMaxAccelerationMetersPerSecondSquared);

    private final SlewRateLimiter yLimiter =
            new SlewRateLimiter(AutoConstants.kMaxAccelerationMetersPerSecondSquared);

    private final SlewRateLimiter rotLimiter =
            new SlewRateLimiter(AutoConstants.kMaxAngularSpeedRadiansPerSecondSquared);

    /* ================= POSE ESTIMATORS ================= */

    private final SwerveDrivePoseEstimator m_odometryEstimator =
            new SwerveDrivePoseEstimator(
                    DriveConstants.kDriveKinematics,
                    getGyroRotation(),
                    getModulePositions(),
                    new Pose2d()
            );

    private final SwerveDrivePoseEstimator m_visionEstimator =
            new SwerveDrivePoseEstimator(
                    DriveConstants.kDriveKinematics,
                    getGyroRotation(),
                    getModulePositions(),
                    new Pose2d()
            );

    /* ================= CONSTRUCTOR ================= */

    public Drivetrain(Turret turret) {

        this.m_turret = turret;

        SmartDashboard.putData("Field", field);

        centerVector = field.getObject("RobotToCenter");

        field.getObject("FieldCenter")
                .setPose(new Pose2d(FIELD_CENTER, new Rotation2d()));

        try {

            RobotConfig config = RobotConfig.fromGUISettings();

            AutoBuilder.configure(
                    this::getPose,
                    this::resetOdometry,
                    this::getRobotRelativeSpeeds,
                    this::driveRobotRelative,
                    new PPHolonomicDriveController(
                            new PIDConstants(5, 0, 0),
                            new PIDConstants(5, 0, 0)
                    ),
                    config,
                    () -> DriverStation.getAlliance().isPresent()
                            && DriverStation.getAlliance().get() == DriverStation.Alliance.Red,
                    this
            );

        } catch (Exception e) {

            DriverStation.reportError(
                    "Failed to configure AutoBuilder",
                    e.getStackTrace()
            );

        }
    }

    /* ================= PERIODIC ================= */

    @Override
    public void periodic() {

        m_odometryEstimator.update(getGyroRotation(), getModulePositions());

        m_visionEstimator.update(getGyroRotation(), getModulePositions());

        LimelightLib.SetRobotOrientation(
                LIMELIGHT_NAME,
                MathUtil.inputModulus(getHeading(), 0, 360),
                getTurnRate(),
                0, 0, 0, 0
        );

        PoseEstimate visionEstimate =
                LimelightLib.getBotPoseEstimate_wpiBlue_MegaTag2(LIMELIGHT_NAME);

        if (LimelightLib.validPoseEstimate(visionEstimate)
                && Math.abs(getTurnRate()) < 720) {

            m_visionEstimator.addVisionMeasurement(
                    visionEstimate.pose,
                    visionEstimate.timestampSeconds
            );

            field.getObject("Vision Raw").setPose(visionEstimate.pose);
        }

        Pose2d odomPose = m_odometryEstimator.getEstimatedPosition();
        Pose2d visionPose = m_visionEstimator.getEstimatedPosition();

        currentPose = new Pose2d(
                odomPose.getX(),
                odomPose.getY(),
                getGyroRotation()
        );

        field.setRobotPose(visionPose);

        /* ===== LEFT THIRD CHECK ===== */

        boolean inLeftThird =
                visionPose.getX() >= 0 &&
                visionPose.getX() <= LEFT_THIRD_ENDPOINT;

        SmartDashboard.putBoolean("RobotInLeftThird", inLeftThird);

        /* ===== VECTOR TO CENTER ===== */

        Pose2d centerPose = new Pose2d(FIELD_CENTER, new Rotation2d());

        centerVector.setPoses(
                visionPose,
                centerPose
        );

        SmartDashboard.putNumber(
                "Gyro Heading",
                getGyroRotation().getDegrees()
        );
    }

    /* ================= POSE METHODS ================= */

    public Pose2d getPose() {

        Pose2d pose = m_odometryEstimator.getEstimatedPosition();

        return new Pose2d(
                -pose.getX(),
                -pose.getY(),
                pose.getRotation()
        );
    }

    public void resetOdometry(Pose2d pose) {

        Pose2d invertedPose =
                new Pose2d(-pose.getX(), -pose.getY(), pose.getRotation());

        m_odometryEstimator.resetPosition(
                getGyroRotation(),
                getModulePositions(),
                invertedPose
        );

        m_visionEstimator.resetPosition(
                getGyroRotation(),
                getModulePositions(),
                invertedPose
        );
    }

    /* ================= DRIVE ================= */

    public void drive(
            double xSpeed,
            double ySpeed,
            double rot,
            boolean fieldRelative
    ) {

        double x =
                xLimiter.calculate(xSpeed)
                        * DriveConstants.kMaxSpeedMetersPerSecond;

        double y =
                yLimiter.calculate(ySpeed)
                        * DriveConstants.kMaxSpeedMetersPerSecond;

        double r =
                rotLimiter.calculate(rot)
                        * DriveConstants.kMaxAngularSpeed;

        ChassisSpeeds speeds =
                fieldRelative
                        ? ChassisSpeeds.fromFieldRelativeSpeeds(
                                x, y, r, getGyroRotation())
                        : new ChassisSpeeds(x, y, r);

        setModuleStates(
                DriveConstants.kDriveKinematics
                        .toSwerveModuleStates(speeds)
        );
    }

    public void driveRobotRelative(ChassisSpeeds speeds) {

        ChassisSpeeds inverted = new ChassisSpeeds(
                -speeds.vxMetersPerSecond,
                -speeds.vyMetersPerSecond,
                speeds.omegaRadiansPerSecond
        );

        setModuleStates(
                DriveConstants.kDriveKinematics
                        .toSwerveModuleStates(inverted)
        );
    }

    public void driveLocked(
            double xSpeed,
            double ySpeed,
            double targetHeading,
            boolean fieldRelative
    ) {

        double correction =
                headingCorrector.calculate(getHeading(), targetHeading);

        drive(xSpeed, ySpeed, correction, fieldRelative);
    }

    /* ================= MODULE CONTROL ================= */

    public void setModuleStates(SwerveModuleState[] states) {

        SwerveDriveKinematics.desaturateWheelSpeeds(
                states,
                DriveConstants.kMaxSpeedMetersPerSecond
        );

        m_frontLeft.setDesiredState(states[0]);
        m_frontRight.setDesiredState(states[1]);
        m_rearLeft.setDesiredState(states[2]);
        m_rearRight.setDesiredState(states[3]);
    }

    public ChassisSpeeds getRobotRelativeSpeeds() {

        return DriveConstants.kDriveKinematics.toChassisSpeeds(
                m_frontLeft.getState(),
                m_frontRight.getState(),
                m_rearLeft.getState(),
                m_rearRight.getState()
        );
    }

    /* ================= GYRO ================= */

    public void zeroHeading() {
        m_gyro.reset();
    }

    public double getHeading() {
        return MathUtil.inputModulus(-m_gyro.getAngle(), -180, 180);
    }

    public double getTurnRate() {
        return -m_gyro.getRate();
    }

    public Rotation2d getGyroRotation() {
        return Rotation2d.fromDegrees(-m_gyro.getAngle());
    }

    /* ================= UTIL ================= */

    public SwerveModulePosition[] getModulePositions() {

        return new SwerveModulePosition[]{
                m_frontLeft.getPosition(),
                m_frontRight.getPosition(),
                m_rearLeft.getPosition(),
                m_rearRight.getPosition()
        };
    }

    public void resetEncoders() {

        m_frontLeft.resetEncoders();
        m_frontRight.resetEncoders();
        m_rearLeft.resetEncoders();
        m_rearRight.resetEncoders();
    }
}