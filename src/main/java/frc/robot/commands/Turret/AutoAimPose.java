package frc.robot.commands.Turret;

import static edu.wpi.first.units.Units.*;

import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.generated.TunerConstants;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

public class AutoAimPose extends Command {

    // PROTOTYPE MODE: MaxSpeed matches TeleopDrive so translation feel is identical.
    private static final double MAX_SPEED = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);

    private double theta;
    private double thetaPass;
    private double currentAngle;
    private double pointAngle = 0;
    private boolean isPassingMode = false;

    public AutoAimPose() {
        addRequirements(RobotContainer.turret);
        // PROTOTYPE MODE: also require drivetrain so this command can rotate the robot.
        // *** AUTO CONFLICT WARNING ***
        // This command is also registered as NamedCommand "AutoAim" and used in "StopPass".
        // PathPlanner paths require drivetrain too — running those named commands inside a
        // path WILL cause a requirement conflict and the path's drivetrain control will win,
        // interrupting this command. For prototype auto use, comment out the "AutoAim" and
        // "StopPass" NamedCommand registrations in RobotContainer.registerNamedCommands().
        addRequirements(RobotContainer.drivetrain);
    }

    public void initialize() {
    }

    @Override
    public void execute() {
        theta = RobotContainer.poseEst.aimToHub();
        thetaPass = RobotContainer.poseEst.aimToPass();
        isPassingMode = RobotContainer.poseEst.getPassingMode();
        currentAngle = RobotContainer.drivetrain.getState().Pose.getRotation().getDegrees();
        double distance = RobotContainer.poseEst.distanceTarget();

        // Robot velocity in field frame (m/s)
        double vx = RobotContainer.turret.getXSpeed();
        double vy = RobotContainer.turret.getYSpeed();

        // Shooting Mode
        if (!isPassingMode) {
            if (DriverStation.getAlliance().get() == Alliance.Red) {
                pointAngle = (currentAngle > 0) ? -180 - theta + currentAngle
                                               :  180 - theta + currentAngle;
            } else {
                pointAngle = currentAngle - theta;
            }

            // Velocity compensation: lateral robot velocity causes angular drift
            double thetaRad = Math.toRadians(theta);
            double tof = distance / Constants.Shooter.ProjectileSpeed.metersPerSecond;
            double lateralSpeed = -vx * Math.sin(thetaRad) + vy * Math.cos(thetaRad);
            double angularRate = lateralSpeed / Math.max(distance, 0.01) * (180.0 / Math.PI);
            pointAngle -= angularRate * tof;
        }

        // Passing Mode
        else {
            if (DriverStation.getAlliance().get() == Alliance.Red) {
                pointAngle = -thetaPass + currentAngle;
            } else {
                pointAngle = (currentAngle > 0) ? -180 - thetaPass + currentAngle
                                               :  180 - thetaPass + currentAngle;
            }

            // Velocity compensation for passing
            double thetaPassRad = Math.toRadians(thetaPass);
            double tof = distance / Constants.Shooter.ProjectileSpeed.metersPerSecond;
            double lateralSpeed = -vx * Math.sin(thetaPassRad) + vy * Math.cos(thetaPassRad);
            double angularRate = lateralSpeed / Math.max(distance, 0.01) * (180.0 / Math.PI);
            pointAngle -= angularRate * tof;
        }

        SmartDashboard.putNumber("POINTANGLE", pointAngle);

        // --- Turret calls (no-ops in prototype mode, motor output suppressed) ---
        double targetPosition = pointAngle / 9.2571428;
        RobotContainer.turret.setTargetPosition(targetPosition);

        // PROTOTYPE MODE: convert turret setpoint to a field-relative robot heading and
        // send it to the drivetrain via FieldCentricFacingAngle.
        //
        // Math: targetHeading = currentAngle - pointAngle
        //   Blue:       currentAngle - (currentAngle - theta)          = theta
        //   Red (>0):   currentAngle - (-180 - theta + currentAngle)   = 180 + theta
        //   Red (<=0):  currentAngle - ( 180 - theta + currentAngle)   = theta - 180
        // This is equivalent to the heading the robot must face so the turret (if it existed)
        // would be at zero with the shooter pointed at the target.
        //
        // Driver left stick still provides translation — FieldCentricFacingAngle supports
        // simultaneous translation and heading control.
        double targetHeadingDeg = currentAngle - pointAngle;

        // When pose estimator says we're in an unsafe zone, hold current heading
        // (mirrors the turret's safe-zone behavior of zeroing the setpoint).
        if (!RobotContainer.poseEst.getIsSafe()) {
            targetHeadingDeg = currentAngle;
        }

        double driverVx = MathUtil.applyDeadband(
                -Robot.robotContainer.driverController.getRawAxis(XboxController.Axis.kLeftY.value), 0.05)
                * MAX_SPEED;
        double driverVy = MathUtil.applyDeadband(
                -Robot.robotContainer.driverController.getRawAxis(XboxController.Axis.kLeftX.value), 0.05)
                * MAX_SPEED;

        RobotContainer.drivetrain.setControl(
                RobotContainer.facingAngle
                        .withTargetDirection(Rotation2d.fromDegrees(targetHeadingDeg))
                        .withVelocityX(driverVx)
                        .withVelocityY(driverVy));

        if (RobotContainer.poseEst.getIsSafe()) {
            RobotContainer.turret.positionControl();
        } else {
            RobotContainer.turret.setTargetPosition(0);
            RobotContainer.turret.positionControl();
        }
    }

    public boolean isFinished() {
        return false;
    }

    protected void end() {
    }

    protected void interrupted() {
        end();
    }
}
