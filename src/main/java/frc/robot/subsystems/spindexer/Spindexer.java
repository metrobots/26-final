package frc.robot.subsystems.spindexer;

import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.Constants;

public class Spindexer extends SubsystemBase {

    private final SparkFlex indexer = new SparkFlex(Constants.kIndexerCanId, MotorType.kBrushless);

    public Spindexer() {}

    public double getIndexerCurrent() {
        return indexer.getOutputCurrent();
    }

    // Define methods to move the motors forwards and backwards
    public void spinIndexer(double speed) {
        indexer.set(speed);
    }

    public void stop() {
        indexer.stopMotor();
    }
}