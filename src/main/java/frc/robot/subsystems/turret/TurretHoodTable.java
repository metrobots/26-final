package frc.robot.subsystems.turret;

import java.util.TreeMap;

public class TurretHoodTable {

    public static class HoodData {
        public final double angle;   // degrees
        public final double speed;   // RPS

        public HoodData(double angle, double speed) {
            this.angle = angle;
            this.speed = speed;
        }

        @Override
        public String toString() {
            return "HoodData{angle=" + angle + ", speed=" + speed + "}";
        }
    }

    private final TreeMap<Double, HoodData> table = new TreeMap<>();

    public TurretHoodTable() {

        // distance (meters) → hood angle + shooter speed
        table.put(1.5, new HoodData(0, 18));
        table.put(1.75, new HoodData(0, 18));
        table.put(2.0, new HoodData(0, 18));
        table.put(2.25, new HoodData(0, 20.6));
        table.put(2.5, new HoodData(0, 21));
        table.put(2.75, new HoodData(0, 21.5));
        table.put(3.0, new HoodData(0, 22));
        table.put(3.25, new HoodData(0, 23));
        table.put(3.5, new HoodData(0, 25));
        table.put(3.75, new HoodData(0, 27));
        table.put(4.0, new HoodData(0, 28)); // NEED A HOOD ANGLE FOR THIS ONE
        table.put(4.25, new HoodData(0, 29));
        
    }

    public HoodData get(double distance) {

        var lower = table.floorEntry(distance);
        var upper = table.ceilingEntry(distance);

        if (lower == null) return upper.getValue();
        if (upper == null) return lower.getValue();

        if (lower.getKey().equals(upper.getKey()))
            return lower.getValue();

        double ratio =
            (distance - lower.getKey()) /
            (upper.getKey() - lower.getKey());

        double angle =
            lower.getValue().angle +
            ratio * (upper.getValue().angle - lower.getValue().angle);

        double speed =
            lower.getValue().speed +
            ratio * (upper.getValue().speed - lower.getValue().speed);

        return new HoodData(angle, speed);
    }
}