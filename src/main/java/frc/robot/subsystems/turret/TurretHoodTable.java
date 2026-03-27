package frc.robot.subsystems.turret;

import java.util.TreeMap;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class TurretHoodTable {

    public static class HoodData {
        public final double angle;
        public final double speed;

        public HoodData(double angle, double speed) {
            this.angle = angle;
            this.speed = speed;
        }

        @Override
        public String toString() {
            return "HoodData{angle=" + angle + ", speed=" + speed + "}";
        }
    }

    private static final String SPEED_SCALE_KEY = "Hood/SpeedScale";
    private static final String ANGLE_OFFSET_KEY = "Hood/AngleOffset";

    private final TreeMap<Double, HoodData> table = new TreeMap<>();

    public TurretHoodTable() {
        SmartDashboard.putNumber(SPEED_SCALE_KEY, 1.0);
        SmartDashboard.putNumber(ANGLE_OFFSET_KEY, 0.0);

        table.put(1.5,  new HoodData(0, 22)); //irrelevant
        table.put(1.75, new HoodData(0, 22)); //irrelevant
        table.put(2.0,  new HoodData(0, 23)); //good
        table.put(2.25, new HoodData(0, 24)); //good
        table.put(2.5,  new HoodData(0, 25)); //good
        table.put(2.75, new HoodData(0, 25.5)); //good
        table.put(3.0,  new HoodData(4, 25)); //good
        table.put(3.25, new HoodData(4, 26)); //good

        table.put(3.5,  new HoodData(4, 27.5));
        table.put(3.75, new HoodData(0, 23));
        table.put(4.0,  new HoodData(0, 23));
        table.put(4.25, new HoodData(0, 23));
    }

    public HoodData get(double distance) {
        var lower = table.floorEntry(distance);
        var upper = table.ceilingEntry(distance);

        if (lower == null) return applyScale(upper.getValue());
        if (upper == null) return applyScale(lower.getValue());

        if (lower.getKey().equals(upper.getKey()))
            return applyScale(lower.getValue());

        double ratio =
            (distance - lower.getKey()) /
            (upper.getKey() - lower.getKey());

        double angle =
            lower.getValue().angle +
            ratio * (upper.getValue().angle - lower.getValue().angle);

        double speed =
            lower.getValue().speed +
            ratio * (upper.getValue().speed - lower.getValue().speed);

        return applyScale(new HoodData(angle, speed));
    }

    private HoodData applyScale(HoodData raw) {
        double speedScale  = SmartDashboard.getNumber(SPEED_SCALE_KEY, 1.0);
        double angleOffset = SmartDashboard.getNumber(ANGLE_OFFSET_KEY, 0.0);
        return new HoodData(raw.angle + angleOffset, raw.speed * speedScale);
    }
}