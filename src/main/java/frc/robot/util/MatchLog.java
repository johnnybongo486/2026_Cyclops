package frc.robot.util;

import com.ctre.phoenix6.SignalLogger;
import edu.wpi.first.wpilibj.DriverStation;

/** Thin wrapper around SignalLogger for match events + scoring markers. */
public final class MatchLog {
  private MatchLog() {}

  /** Log a discrete event (auto start, scoring, intake pickup, etc.). */
  public static void event(String name) {
    SignalLogger.writeString("events/" + name, matchContext());
  }

  /** Log an event with extra context (e.g. "scored/L4", "amp"). */
  public static void event(String category, String detail) {
    SignalLogger.writeString("events/" + category, detail + " @ " + matchContext());
  }

  private static String matchContext() {
    return String.format("match=%.1fs mode=%s",
        DriverStation.getMatchTime(),
        DriverStation.isAutonomous() ? "auto" : DriverStation.isTeleop() ? "tele" : "disabled");
  }
}