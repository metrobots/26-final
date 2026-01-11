package frc.robot.subsystems.vision.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drivetrain.Drivetrain;
import frc.robot.utils.Constants;
import frc.robot.utils.Constants.PIDConstants;
import frc.robot.utils.LimelightLib;

/**
 * Command to align the robot using the 2D limelight measurements (TX)
 */
public class Align2D extends Command {
  private final Drivetrain drivetrain;
  private final PIDController yPID = PIDConstants.yPID; 
  private final PIDController xPID = PIDConstants.xPID; 
  private final String limelightName = Constants.AutoConstants.limelightName;
  private final double testSetpoint = 0;
  
  /**
   * Creates a new AlignLeft command that aligns using 2D measurements
   *
   * @param drivetrain The drivetrain subsystem to control
   */
  public Align2D(Drivetrain drivetrain) {
    this.drivetrain = drivetrain;
    
    // Configure PID controllers
    yPID.setTolerance(0);
    xPID.setTolerance(0);
    
    addRequirements(drivetrain);
  }

  @Override
  public void initialize() {
    // Reset PID controllers when command starts
    yPID.reset();
    xPID.reset();
  }

  @Override
  public void execute() {

    // Get the TX, which is the Target offset X. This changes relative to how far you are from the target
    double lateralOffset = LimelightLib.getTX(limelightName);
    
    if (lateralOffset == 0) {
      drivetrain.drive(0, 0, 0, false);
      return;
    }

    // Calculate PID output for side-to-side movement
    double lateralOutput = yPID.calculate(lateralOffset, testSetpoint);
    lateralOutput = MathUtil.clamp(lateralOutput, -0.7, 0.7);
        
    // Apply side-to-side movement
    drivetrain.drive(
      0,
      -lateralOutput,
      0,
      false // Make sure that alignment commands are not field relative
    );
  }

  @Override
  public void end(boolean interrupted) {
    drivetrain.drive(0, 0, 0, false);

  }

  @Override
  public boolean isFinished() {

    // Command never finishes on its own
    return false;
  }
}