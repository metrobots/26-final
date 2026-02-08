package frc.robot.subsystems.turret.commands;

import java.util.Map;
import java.util.TreeMap;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.turret.Turret;
import frc.robot.subsystems.turret.TurretHoodTable;
import frc.robot.utils.LimelightLib;

public class AimTurret extends Command {

    private static final String LIMELIGHT_NAME = "limelight";
    private static final double ANGLE_TOLERANCE = 1.0; // degrees

    private final Turret turret;
    private final TreeMap<Double, TurretHoodTable.HoodData> hoodTable;

    public AimTurret(Turret turret, TurretHoodTable hoodTableProvider) {
        this.turret = turret;
        this.hoodTable = hoodTableProvider.getTable();
        addRequirements(turret);
    }

    @Override
    public void execute() {
        if (!LimelightLib.getTV(LIMELIGHT_NAME)) return;

        // 1) Aim turret
        turret.aimTurretWithTx();

        // 2) Get distance
        double distance = getDistanceMeters();

        // 3) Interpolate hood angle and flywheel speed
        TurretHoodTable.HoodData target = interpolateHoodData(distance);

        turret.setHoodAngle(target.angle);
        turret.setTargetFlywheelRPM(target.speed); // Store for ShootTurret
    }

    @Override
    public void end(boolean interrupted) {
        // Optionally stop the flywheel if we want AimTurret to also cancel spin
        // turret.setFlywheelRPM(0);
    }

    @Override
    public boolean isFinished() {
        // We never "finish" automatically; it runs until canceled
        return false;
    }

    // ---------------- Helpers ----------------

    private double getDistanceMeters() {
        double[] pose = LimelightLib.getTargetPose_CameraSpace(LIMELIGHT_NAME);
        if (pose.length < 3) return hoodTable.firstKey();
        return Math.hypot(pose[0], pose[1]);
    }

    private TurretHoodTable.HoodData interpolateHoodData(double distance) {
        Map.Entry<Double, TurretHoodTable.HoodData> lower = hoodTable.floorEntry(distance);
        Map.Entry<Double, TurretHoodTable.HoodData> upper = hoodTable.ceilingEntry(distance);

        if (lower == null) return hoodTable.firstEntry().getValue();
        if (upper == null) return hoodTable.lastEntry().getValue();
        if (lower.getKey().equals(upper.getKey())) return lower.getValue();

        double d0 = lower.getKey();
        double d1 = upper.getKey();
        double t = (distance - d0) / (d1 - d0);

        double angle = lower.getValue().angle + t * (upper.getValue().angle - lower.getValue().angle);
        double speed = lower.getValue().speed + t * (upper.getValue().speed - lower.getValue().speed);

        return new TurretHoodTable.HoodData(angle, speed);
    }
}
