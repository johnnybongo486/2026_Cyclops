package frc.robot.util;

import com.ctre.phoenix6.SignalLogger;
import edu.wpi.first.wpilibj.DriverStation;

public final class MatchLog {
    private MatchLog() {}

    /** Writes a lifecycle event marker to the active hoot log. */
    public static void event(String name) {
        SignalLogger.writeString("events/" + name, matchContext());
    }

    /** Writes a lifecycle event marker with an extra detail string. */
    public static void event(String category, String detail) {
        SignalLogger.writeString("events/" + category, detail + " | " + matchContext());
    }

    private static String matchContext() {
        double matchTime = DriverStation.getMatchTime();
        String mode;
        if (DriverStation.isAutonomous()) {
            mode = "auto";
        } else if (DriverStation.isTeleop()) {
            mode = "tele";
        } else {
            mode = "disabled";
        }
        return String.format("t=%.2f mode=%s", matchTime, mode);
    }
}
