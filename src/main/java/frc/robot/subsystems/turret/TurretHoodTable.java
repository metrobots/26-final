package frc.robot.subsystems.turret;

import java.util.LinkedHashMap;
import java.util.Map;
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

    // Global tuning knobs
    private static final String SPEED_SCALE_KEY  = "Hood/SpeedScale";
    private static final String ANGLE_OFFSET_KEY = "Hood/AngleOffset";

    // Default table data: distance (m) -> {angle, speed}
    private static final double[][] DEFAULTS = {
        { 1.50, 0, 27 },
        { 1.75, 0, 27 },
        { 2.00, 0, 27 },
        { 2.25, 1, 28 },
        { 2.50, 1, 29 },
        { 2.75, 1, 30 },
        { 3.00, 1, 31 },
        { 3.25, 1, 32 },
        { 3.50, 2, 33 },
        { 3.75, 2, 34 },
        { 4.00, 2, 35 },
        { 4.25, 2, 36 },
    };

    // Sorted distances used for interpolation
    private final double[] distances;

    public TurretHoodTable() {
        distances = new double[DEFAULTS.length];

        // Publish global scalars
        SmartDashboard.putNumber(SPEED_SCALE_KEY,  1.0);
        SmartDashboard.putNumber(ANGLE_OFFSET_KEY, 0.0);

        // Publish every row so they show up as editable fields
        for (int i = 0; i < DEFAULTS.length; i++) {
            double dist  = DEFAULTS[i][0];
            double angle = DEFAULTS[i][1];
            double speed = DEFAULTS[i][2];

            distances[i] = dist;
            SmartDashboard.putNumber(angleKey(dist), angle);
            SmartDashboard.putNumber(speedKey(dist), speed);
        }
    }

    /** Call this from your subsystem's periodic() to push current values to the dashboard. */
    public void updateDashboard() {
        // Values are already on the dashboard from putNumber in the constructor.
        // This method exists so you can add extra telemetry if needed later.
    }

    public HoodData get(double distance) {
        // Find the two surrounding breakpoints
        int lo = 0, hi = distances.length - 1;
        for (int i = 0; i < distances.length - 1; i++) {
            if (distances[i] <= distance && distance <= distances[i + 1]) {
                lo = i;
                hi = i + 1;
                break;
            }
        }

        // Clamp to table bounds
        if (distance <= distances[0])  return applyScale(readRow(0));
        if (distance >= distances[hi]) return applyScale(readRow(distances.length - 1));

        double ratio =
            (distance - distances[lo]) /
            (distances[hi] - distances[lo]);

        HoodData lower = readRow(lo);
        HoodData upper = readRow(hi);

        double angle = lower.angle + ratio * (upper.angle - lower.angle);
        double speed = lower.speed + ratio * (upper.speed - lower.speed);

        return applyScale(new HoodData(angle, speed));
    }

    // ── helpers ──────────────────────────────────────────────────────────────

    /** Read a row's current values live from SmartDashboard. */
    private HoodData readRow(int index) {
        double dist = distances[index];
        double angle = SmartDashboard.getNumber(angleKey(dist), DEFAULTS[index][1]);
        double speed = SmartDashboard.getNumber(speedKey(dist), DEFAULTS[index][2]);
        return new HoodData(angle, speed);
    }

    private HoodData applyScale(HoodData raw) {
        double speedScale  = SmartDashboard.getNumber(SPEED_SCALE_KEY,  1.0);
        double angleOffset = SmartDashboard.getNumber(ANGLE_OFFSET_KEY, 0.0);
        return new HoodData(raw.angle + angleOffset, raw.speed * speedScale);
    }

    private static String angleKey(double dist) {
        return String.format("Hood/Table/%.2fm/angle", dist);
    }

    private static String speedKey(double dist) {
        return String.format("Hood/Table/%.2fm/speed", dist);
    }
}