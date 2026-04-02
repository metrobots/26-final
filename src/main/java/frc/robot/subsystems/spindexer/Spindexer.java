// package frc.robot.subsystems.spindexer;

// import com.revrobotics.spark.SparkFlex;
// import com.revrobotics.spark.SparkLowLevel.MotorType;
// import com.revrobotics.spark.config.SparkFlexConfig;

// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
// import edu.wpi.first.wpilibj2.command.SubsystemBase;
// import frc.robot.utils.Constants;

// public class Spindexer extends SubsystemBase {

//     private final SparkFlex indexer = new SparkFlex(Constants.kIndexerCanId, MotorType.kBrushless);

//     private static final double STALL_CURRENT_THRESHOLD = 60.0;  // Amps
//     private static final double STALL_VELOCITY_THRESHOLD = 0.5;  // RPM

//     public Spindexer() {
//         SparkFlexConfig config = new SparkFlexConfig();
//         config.smartCurrentLimit(80);
//         indexer.configure(config,
//             com.revrobotics.spark.SparkBase.ResetMode.kResetSafeParameters,
//             com.revrobotics.spark.SparkBase.PersistMode.kPersistParameters);
//     }

//     public double getIndexerCurrent() {
//         return indexer.getOutputCurrent();
//     }

//     public double getIndexerVelocity() {
//         return indexer.getEncoder().getVelocity();
//     }

//     public boolean isStalled() {
//         boolean highCurrent = getIndexerCurrent() > STALL_CURRENT_THRESHOLD;
//         boolean lowVelocity = Math.abs(getIndexerVelocity()) < STALL_VELOCITY_THRESHOLD;
//         return highCurrent && lowVelocity;
//     }

//     public void spinIndexer(double speed) {
//         indexer.set(speed);
//     }

//     public void stop() {
//         indexer.stopMotor();
//     }

//     @Override
//     public void periodic() {
//         SmartDashboard.putNumber("Spindexer/Current", getIndexerCurrent());
//         SmartDashboard.putNumber("Spindexer/Velocity", getIndexerVelocity());
//         SmartDashboard.putBoolean("Spindexer/Stalled", isStalled());
//     }
// }