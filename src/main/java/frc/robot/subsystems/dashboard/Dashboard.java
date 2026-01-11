package frc.robot.subsystems.dashboard;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Dashboard extends SubsystemBase {

    private boolean redHubActive = true;
    private boolean blueHubActive = true;
    private String activeHubDisplay = "BOTH";
    private double shiftCountdown = 0; // seconds until next shift

    public Dashboard() {
        SmartDashboard.putBoolean("Red Hub Active", true);
        SmartDashboard.putBoolean("Blue Hub Active", true);
        SmartDashboard.putString("Active Hub", "BOTH");
        SmartDashboard.putNumber("Time to Next Shift", 0);
    }

    @Override
    public void periodic() {
        updateHubStates();
        updateShiftCountdown();
        publishToDashboard();
    }

    private void updateHubStates() {
        double matchTime = DriverStation.getMatchTime();

        // Default state (AUTO, TRANSITION, ENDGAME, disabled)
        redHubActive = true;
        blueHubActive = true;

        if (matchTime < 0) {
            updateDisplayString();
            return;
        }

        boolean inShift = (matchTime <= 130 && matchTime > 30);
        if (!inShift) {
            updateDisplayString();
            return;
        }

        String gameData = DriverStation.getGameSpecificMessage();
        if (gameData.isEmpty()) {
            updateDisplayString();
            return;
        }

        boolean redFavored = gameData.charAt(0) == 'R';

        boolean favoredAllianceActive;
        if (matchTime <= 130 && matchTime > 105) {
            favoredAllianceActive = false; // Shift 1
        } else if (matchTime <= 105 && matchTime > 80) {
            favoredAllianceActive = true;  // Shift 2
        } else if (matchTime <= 80 && matchTime > 55) {
            favoredAllianceActive = false; // Shift 3
        } else if (matchTime <= 55 && matchTime > 30) {
            favoredAllianceActive = true;  // Shift 4
        } else {
            updateDisplayString();
            return;
        }

        if (redFavored) {
            redHubActive = favoredAllianceActive;
            blueHubActive = !favoredAllianceActive;
        } else {
            blueHubActive = favoredAllianceActive;
            redHubActive = !favoredAllianceActive;
        }

        updateDisplayString();
    }

    private void updateDisplayString() {
        if (redHubActive && blueHubActive) {
            activeHubDisplay = "BOTH";
        } else if (redHubActive) {
            activeHubDisplay = "RED";
        } else if (blueHubActive) {
            activeHubDisplay = "BLUE";
        } else {
            activeHubDisplay = "NONE"; // should never happen
        }
    }

    private void updateShiftCountdown() {
        double matchTime = DriverStation.getMatchTime();

        // Default: no countdown
        shiftCountdown = 0;

        if (matchTime < 0) {
            return; // pre-match
        }

        // Define shift start times in seconds
        double[] shiftTimes = {130, 105, 80, 55}; // Shifts 1–4 start times
        for (double shiftStart : shiftTimes) {
            if (matchTime > shiftStart) continue;
            shiftCountdown = matchStartDelta(matchTime, shiftStart);
            return;
        }
    }

    private double matchStartDelta(double matchTime, double shiftStart) {
        // Returns seconds until next shift
        double delta = shiftStart - matchTime;
        return delta > 0 ? delta : 0;
    }

    private void publishToDashboard() {
        SmartDashboard.putBoolean("Red Hub Active", redHubActive);
        SmartDashboard.putBoolean("Blue Hub Active", blueHubActive);
        SmartDashboard.putString("Active Hub", activeHubDisplay);
        SmartDashboard.putNumber("Time to Next Shift", shiftCountdown);
    }

    // Optional getters
    public boolean isRedHubActive() { return redHubActive; }
    public boolean isBlueHubActive() { return blueHubActive; }
    public String getActiveHubDisplay() { return activeHubDisplay; }
    public double getShiftCountdown() { return shiftCountdown; }
}
