package frc.robot.subsystems.turret;

import java.util.TreeMap;

public class TurretTable {

    public static class TurretData {
        public final double turretAngle; // degrees

        public TurretData(double turretAngle) {
            this.turretAngle = turretAngle;
        }

        @Override
        public String toString() {
            return "TurretData{turretAngle=" + turretAngle + "}";
        }
    }

    private final TreeMap<Double, TurretData> table = new TreeMap<>();

    public TurretTable() {

        // drivetrain angle (degrees) → turret angle (degrees)
        table.put(15.0, new TurretData(0.7765646576881409));
        table.put(30.0, new TurretData(0.6361038088798523));
        table.put(0.0, new TurretData(0.0));
        table.put(45.0, new TurretData(-45.0));
        table.put(90.0, new TurretData(-90.0));
    }

    public TurretData get(double drivetrainAngle) {

        var lower = table.floorEntry(drivetrainAngle);
        var upper = table.ceilingEntry(drivetrainAngle);

        if (lower == null) return upper.getValue();
        if (upper == null) return lower.getValue();

        if (lower.getKey().equals(upper.getKey()))
            return lower.getValue();

        double ratio =
            (drivetrainAngle - lower.getKey()) /
            (upper.getKey() - lower.getKey());

        double turretAngle =
            lower.getValue().turretAngle +
            ratio * (upper.getValue().turretAngle - lower.getValue().turretAngle);

        return new TurretData(turretAngle);
    }
}