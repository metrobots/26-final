package frc.robot.subsystems.shooter;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.AnalogEncoder;
import frc.robot.utils.Config;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.Constants;
import frc.robot.utils.Constants.AutoConstants;
import frc.robot.utils.Constants.DriveConstants;

public class Shooter extends SubsystemBase {
    // Motor Definitions
    private final SparkMax flywheelSpark; // Runs Flywheel Motors
    private final SparkMax aimSpark; // Motor for Aiming Mechanism

    // Encoder Definitions
    private final RelativeEncoder flywheelEncoder; // Gets Flywheel Velocity
    private final AnalogEncoder aimEncoder; // Gets motor data to convert into shooter angle

    public Shooter (int flywheelCANId, int aimCANId, int analogPort, double analogOffset) {
        // Motor Definitions
        flywheelSpark = new SparkMax (flywheelCANId, MotorType.kBrushless);
        aimSpark = new SparkMax (aimCANId, MotorType.kBrushless);
        // Encoder Definitions
        flywheelEncoder = flywheelSpark.getEncoder();
        aimEncoder = new AnalogEncoder(analogPort);
    }
}
