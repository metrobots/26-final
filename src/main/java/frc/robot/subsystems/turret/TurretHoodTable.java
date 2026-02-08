package frc.robot.subsystems.turret;

import java.util.TreeMap;

public class TurretHoodTable {

    public static class HoodData {
        public final double angle;  // degrees
        public final double speed;  // RPM/S

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
        // ---- TUNE THESE ON THE FIELD ----
        table.put(1.5, new HoodData(18.0, 1000.0));
        table.put(2.0, new HoodData(23.0, 1200.0));
        table.put(2.5, new HoodData(28.0, 1400.0));
        table.put(3.0, new HoodData(33.0, 1600.0));
        table.put(3.5, new HoodData(38.0, 1800.0));
    }

    public TreeMap<Double, HoodData> getTable() {
        return table;
    }
}
