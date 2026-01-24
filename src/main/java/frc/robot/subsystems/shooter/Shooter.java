package frc.robot.subsystems.shooter;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.AnalogEncoder;
import frc.robot.utils.Config;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
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

    private final double aimOffset; // Analog encoder offset for aiming

    // Flywheel PID Controller
    private final double fKP = 0.000000001; // Proportional
    private final double fKI = 0; // Integral
    private final double fKD = 0; // Derivative

    PIDController flywheelPID = new PIDController(fKP, fKI, fKD); // PID Controller

    // Aimming PID Controller
    private final double aKP = 0.000000001; // Proportional
    private final double aKI = 0; // Integral
    private final double aKD = 0; // Derivative

    PIDController aimPID = new PIDController(aKP, aKI, aKD); // PID Controller

    // Constant for converting encoder values to shooting angle; add to Constants once finalized
    double angleConversion = 1;

    // Maximum shooter angle before it breaks
    double maxAngle = 90;

    // Precision tolerance for automatic aiming; tune after PID Controllers
    double aimTolerance = 0;

    public Shooter (int flywheelCANId, int aimCANId, int analogPort, double analogOffset) {
        // Motor Definitions
        flywheelSpark = new SparkMax (flywheelCANId, MotorType.kBrushless);
        aimSpark = new SparkMax (aimCANId, MotorType.kBrushless);
        // Encoder Definitions
        flywheelEncoder = flywheelSpark.getEncoder();
        aimEncoder = new AnalogEncoder(analogPort);

        aimOffset = analogOffset;
    }

    public void aimAndShoot (double fireSpeed, double aimSpeed, double desiredAngle) {
        if ((desiredAngle <= getAimPos() + this.aimTolerance) && (desiredAngle >= getAimPos() - this.aimTolerance)) { // Verifies that we won't miss when automatically aiming
            this.fire(fireSpeed); // Bang bang bang
        }
        this.aim(aimSpeed, desiredAngle); // Aims the shooter
        return;
    }

    public void manualAim (double aimSpeed) {
        double newAngle = this.getAimPos() + aimSpeed; // Gives an angle for the aim method based on current angle rather than external data; Edit equation if needed but this should probably work
        this.aim(Math.abs(aimSpeed), newAngle); // Increases/Decreases aiming angle
        return;
    }

    public void fire (double desiredSpeed) {
        flywheelSpark.set(flywheelPID.calculate(desiredSpeed)); // Bang bang bang
        return;
    }

    public void aim (double aimSpeed, double desiredAngle) {
        if (desiredAngle <= 0) {desiredAngle = 0;} // Makes sure that we don't decrease angle further than should be possible
        else if (desiredAngle >= this.maxAngle) {desiredAngle = this.maxAngle;} // See above but in opposite direction
        aimSpark.set (aimPID.calculate(aimSpeed, angleToRotation(desiredAngle))); // Uses PID Controller to move to a set angle.
    }

    public double getAimPos () {
        double currentPos = rotationToAngle(aimEncoder.get()); // Pulls encoder data and converts it to degrees because I hate rotations as a unit of measurement
        return currentPos;
    }

    public double rotationToAngle (double posRotations) {
        double degrees = posRotations * this.angleConversion; // Converts Rotations to Degrees
        return degrees;
    }

    public double angleToRotation (double posDegrees) {
        double rotations = posDegrees / this.angleConversion; // Converts Degrees to Rotations
        return rotations;
    }
}
