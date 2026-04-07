package frc.robot;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.util.MatchLog;

import com.ctre.phoenix6.SignalLogger;
import edu.wpi.first.wpilibj.RobotController;

/**
 * The methods in this class are called automatically corresponding to each mode, as described in
 * the TimedRobot documentation. If you change the name of this class or the package after creating
 * this project, you must also update the Main.java file in the project.
 */
public class Robot extends TimedRobot {
  private Command m_autonomousCommand;

  public static RobotContainer robotContainer;

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  public Robot() {
    // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
    // autonomous chooser on the dashboard.
    robotContainer = new RobotContainer();

    // --- SignalLogger setup ---
    // Default path is "/media/sda1/ctre-logs" if a USB stick is present,
    // otherwise it falls back to the RIO's internal storage. Setting an
    // explicit path makes behavior predictable.
    SignalLogger.setPath("/media/sda1/");

    // Log build metadata so you can tie a log back to a specific commit.
    SignalLogger.writeString("metadata/gitSha", BuildConstants.GIT_SHA);
    SignalLogger.writeString("metadata/buildDate", BuildConstants.BUILD_DATE);
    SignalLogger.writeString("metadata/eventName",
        DriverStation.getEventName().isEmpty() ? "practice" : DriverStation.getEventName());

    SignalLogger.start();
    MatchLog.event("robotInit");
  }

  /**
   * This function is called every 20 ms, no matter the mode. Use this for items like diagnostics
   * that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    RobotContainer.intakeWrist.updateDashboard();
    RobotContainer.hood.updateDashboard();
    RobotContainer.shooter.updateDashboard();
    RobotContainer.poseEst.updatePose();
    RobotContainer.poseEst.updateDashboard();
    RobotContainer.intake.updateDashboard();
    RobotContainer.uptake.updateDashboard();
    RobotContainer.agitator.updateDashboard();

    // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
    // commands, running already-scheduled commands, removing finished or interrupted commands,
    // and running subsystem periodic() methods.  This must be called from the robot's periodic
    // block in order for anything in the Command-based framework to work.
    CommandScheduler.getInstance().run();
  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {
    MatchLog.event("disabled");
  }

  @Override
  public void disabledPeriodic() {
    //RobotContainer.drivetrain.seedFieldCentric(Rotation2d.kZero);
    RobotContainer.poseLimelight.updateLLPositions();
  }

  /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
  @Override
  public void autonomousInit() {
    MatchLog.event("autoStart/" + robotContainer.getSelectedAutoName());
    m_autonomousCommand = robotContainer.getAutonomousCommand();

    // schedule the autonomous command (example)
    if (m_autonomousCommand != null) {
      CommandScheduler.getInstance().schedule(m_autonomousCommand);
    }
    RobotContainer.poseLimelight.updateLLPositions();  // will this work?

  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {}

  @Override
  public void autonomousExit() {
    MatchLog.event("autoEnd");
  }

  @Override
  public void teleopInit() {
    MatchLog.event("teleopStart");

    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }

    // use only for practice
    //RobotContainer.drivetrain.seedFieldCentric(Rotation2d.kZero);

    RobotContainer.standardDeviation = 0.3;
  }

  @Override
  public void teleopExit() {
    MatchLog.event("teleopEnd");
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {}

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}

  /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {}

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {}
}
