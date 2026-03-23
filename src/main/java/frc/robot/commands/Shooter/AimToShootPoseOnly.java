package frc.robot.commands.Shooter;

import static edu.wpi.first.units.Units.*;

import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.generated.TunerConstants;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

public class AimToShootPoseOnly extends Command {

    // PROTOTYPE MODE: MaxSpeed matches TeleopDrive so translation feel is identical.
    private static final double MAX_SPEED = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);

    private double tDistance;
    private double hoodPosition = 0;
    private double shooterSpeed = 0;
    private double newDistance = 0;
    private double newTheta = 0;
    private double newThetaPass = 0;
    private double currentAngle;
    private double pointAngle = 0;
    private boolean isPassingMode = false;
    private double shooterAdder = 0;

    public AimToShootPoseOnly() {
        addRequirements(RobotContainer.shooter);
        addRequirements(RobotContainer.hood);
        // PROTOTYPE MODE: also require drivetrain so this command can rotate the robot.
        // *** AUTO CONFLICT WARNING ***
        // This command is registered as NamedCommands "AimToShoot", "AimToShoot8/20/30/60".
        // PathPlanner paths require drivetrain too — running these named commands inside a
        // path WILL cause a requirement conflict and the path's drivetrain control will win,
        // interrupting this command. For prototype auto use, comment out those NamedCommand
        // registrations in RobotContainer.registerNamedCommands().
        addRequirements(RobotContainer.drivetrain);
    }

    public void initialize() {
    }

    @Override
    public void execute() {
        newDistance = RobotContainer.poseEst.distanceTarget();
        newTheta = RobotContainer.poseEst.aimToHub();
        newThetaPass = RobotContainer.poseEst.aimToPass();
        currentAngle = RobotContainer.drivetrain.getState().Pose.getRotation().getDegrees();
        isPassingMode = RobotContainer.poseEst.getPassingMode();

        // Robot velocity in field frame (m/s) — from swerve state, much cleaner than finite diff
        double vx = RobotContainer.drivetrain.getXSpeed();
        double vy = RobotContainer.drivetrain.getYSpeed();

        if (!isPassingMode) {
            // --- SHOOTING MODE ---

            // Base turret angle (robot-relative degrees)
            // if (DriverStation.getAlliance().get() == Alliance.Red) {
            //     pointAngle = (currentAngle > 0) ? -180 - newTheta + currentAngle
            //                                     :  180 - newTheta + currentAngle;
            // } else {
            pointAngle = currentAngle - newTheta;
            // }

            // Physics-based lead angle compensation.
            // newTheta is the angle of the (robot - hub) vector from the +X axis.
            // We decompose robot velocity into radial (toward/away) and lateral (perpendicular) components.
            double thetaRad = Math.toRadians(newTheta);
            double tof = newDistance / Constants.Shooter.ProjectileSpeed.metersPerSecond;

            // Lateral component (perpendicular to target line): causes angular drift → lead angle
            double lateralSpeed = -vx * Math.sin(thetaRad) + vy * Math.cos(thetaRad);
            double angularRate = -lateralSpeed / Math.max(newDistance, 0.01) * (180.0 / Math.PI);

            // Radial component (toward/away from hub): positive = toward hub → decreases distance
            double radialSpeed = (vx * Math.cos(thetaRad) + vy * Math.sin(thetaRad));

            pointAngle -= angularRate * tof;
            tDistance = Math.max(newDistance - radialSpeed * tof, 0.5);

            // Compute shooter/hood values based on effective shot distance
            hoodPosition = Constants.Shooter.HoodShooting.a * tDistance * tDistance
                         + Constants.Shooter.HoodShooting.b * tDistance
                         + Constants.Shooter.HoodShooting.c;
            shooterSpeed = Constants.Shooter.ShooterSpeed.a * tDistance * tDistance
                         + Constants.Shooter.ShooterSpeed.b * tDistance
                         + Constants.Shooter.ShooterSpeed.c;

            // Always pre-spin the shooter so the flywheel is ready when turret settles
            shooterAdder = RobotContainer.shooter.getShooterAdder();

            RobotContainer.shooter.setTargetVelocity(shooterSpeed + shooterAdder);
            RobotContainer.shooter.velocityControl();

            // PROTOTYPE MODE: rotate drivetrain to the heading the turret would have aimed at.
            // targetHeadingDeg = currentAngle - pointAngle (see AutoAimPose for full derivation).
            double targetHeadingDeg = currentAngle - pointAngle;

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
                RobotContainer.hood.setTargetPosition(hoodPosition);
                RobotContainer.hood.positionControl();
            }

            SmartDashboard.putNumber("SOTM_TOF", tof);
            SmartDashboard.putNumber("SOTM_AngularRate", angularRate);
            SmartDashboard.putNumber("SOTM_RadialSpeed", radialSpeed);
            SmartDashboard.putNumber("SOTM_tDistance", tDistance);
            SmartDashboard.putNumber("ShooterTargetSOTM", shooterSpeed);
            SmartDashboard.putNumber("ShooterAdderSOTM", shooterAdder);


        } else {
            // --- PASSING MODE ---
            Robot.robotContainer.stopRumble();

            // if (DriverStation.getAlliance().get() == Alliance.Red) {
            //     pointAngle = -newThetaPass + currentAngle;
            // } else {
            pointAngle = (currentAngle > 0) ? -180 - newThetaPass + currentAngle
                                            :  180 - newThetaPass + currentAngle;
            // }

            SmartDashboard.putNumber("POINTANGLESHOOT", pointAngle);

            // Physics-based velocity compensation for passing
            double thetaPassRad = Math.toRadians(newThetaPass);
            double tof = newDistance / Constants.Shooter.ProjectileSpeed.metersPerSecond;
            double lateralSpeed = -vx * Math.sin(thetaPassRad) + vy * Math.cos(thetaPassRad);
            double angularRate = lateralSpeed / Math.max(newDistance, 0.01) * (180.0 / Math.PI);
            double radialSpeed = -(vx * Math.cos(thetaPassRad) + vy * Math.sin(thetaPassRad));

            pointAngle -= angularRate * tof;
            tDistance = Math.max(newDistance - radialSpeed * tof, 0.5);

            // PROTOTYPE MODE: rotate drivetrain to the heading the turret would have aimed at.
            double targetHeadingDeg = currentAngle - pointAngle;

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

            hoodPosition = Constants.Shooter.HoodPassing.a * tDistance * tDistance
                         + Constants.Shooter.HoodPassing.b * tDistance
                         + Constants.Shooter.HoodPassing.c;
            shooterSpeed = Constants.Shooter.ShooterPassing.a * tDistance * tDistance
                         + Constants.Shooter.ShooterPassing.b * tDistance
                         + Constants.Shooter.ShooterPassing.c;

            RobotContainer.shooter.setTargetVelocity(shooterSpeed);
            RobotContainer.shooter.velocityControl();
            RobotContainer.hood.setTargetPosition(hoodPosition);
            RobotContainer.hood.positionControl();
            
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
