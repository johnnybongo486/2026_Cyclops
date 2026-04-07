package frc.robot.commands.Shooter;

import frc.robot.Constants;
import frc.robot.RobotContainer;
import edu.wpi.first.wpilibj2.command.Command;


public class ContinuousSetShooterAndHood extends Command {
    private double shooterSpeed = 0;

    private double tDistance;

    private double newDistance = 0;

	private boolean isPassingMode = false;


	public ContinuousSetShooterAndHood() {
		addRequirements(RobotContainer.shooter);
		addRequirements(RobotContainer.hood);
	}

	// Called just before this Command runs the first time
	public void initialize() {
		// RobotContainer.shooter.setTargetVelocity(0);
		// RobotContainer.hood.setTargetPosition(Constants.Shooter.Hood.StoreHoodPosition);
	}

	// Called repeatedly when this Command is scheduled to run
	public void execute() {
		isPassingMode = RobotContainer.poseEst.getPassingMode();
		newDistance = RobotContainer.poseEst.distanceTarget();

		tDistance = newDistance;

		if (isPassingMode == false) {
			shooterSpeed = (Constants.Shooter.ShooterSpeed.a*tDistance*tDistance) + (Constants.Shooter.ShooterSpeed.b*tDistance)+(Constants.Shooter.ShooterSpeed.c);
		} else {
			shooterSpeed = (Constants.Shooter.ShooterPassing.a*tDistance*tDistance) + (Constants.Shooter.ShooterPassing.b*tDistance)+(Constants.Shooter.ShooterPassing.c);
		}

		RobotContainer.shooter.setTargetVelocity(shooterSpeed);
		RobotContainer.shooter.velocityControl();

		RobotContainer.hood.setTargetPosition(Constants.Shooter.Hood.StoreHoodPosition);
		RobotContainer.hood.positionControl();

	}

	// Make this return true when this Command no longer needs to run execute()
	public boolean isFinished() {
			return false;
	}

	// Called once after isFinished returns true
	protected void end() {

	}

	// Called when another command which requires one or more of the same
	// subsystems is scheduled to run
	protected void interrupted() {

	}
}
