// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter.commands;

import frc.robot.subsystems.drivetrain.Drivetrain;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.vision.commands.Align2D;
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
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.utils.Constants;
import frc.robot.utils.Constants.AutoConstants;
import frc.robot.utils.Constants.DriveConstants;
import java.util.Optional;
import frc.robot.utils.LimelightLib;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class AutoShoot extends Command {
  /** Creates a new Shoot. */

  private Shooter shooter;
  private Drivetrain drive;
  private double hubHeight = 1; // Placeholder idk I'll look it up later
  private final String limelightName = Constants.AutoConstants.limelightName;

  public AutoShoot (Shooter importShooter, Drivetrain importDrive) {
    shooter = importShooter;// Sets command to reference an outside shooting mechanism
    drive = importDrive; // Sets command to reference an outside drivetrain
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Align2D align = new Align2D(drive); // Aligns the robot with the hub
    double lateralOffset = LimelightLib.getTX(limelightName); // Ground Distance from the hub.
    double shootAngle = Math.atan(hubHeight/lateralOffset); // I love trigonometry
    shooter.aimAndShoot(1, 1, shootAngle); // Bang bang bang
    return;
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    // Stops the motors
    shooter.flywheelSpark.stopMotor();
    shooter.aimSpark.stopMotor();
    return;
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}