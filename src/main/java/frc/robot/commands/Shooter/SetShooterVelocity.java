package frc.robot.commands.Shooter;

import frc.robot.RobotContainer;
import edu.wpi.first.wpilibj2.command.Command;

public class SetShooterVelocity extends Command {
	private double shooterVelocity = 0;

	public SetShooterVelocity(double shootervelocity) {
		this.shooterVelocity = shootervelocity;

		addRequirements(RobotContainer.shooter);
	}

	// Called just before this Command runs the first time
	public void initialize() {
		RobotContainer.shooter.setTargetVelocity(shooterVelocity);
	}

	// Called repeatedly when this Command is scheduled to run
	public void execute() {
		RobotContainer.shooter.velocityControl();
	}

	// Make this return true when this Command no longer needs to run execute()
	public boolean isFinished() {
			return false; //RobotContainer.shooter.isAtVelocity(shooterVelocity);
	}

	// Called once after isFinished returns true
	protected void end() {

	}

	// Called when another command which requires one or more of the same
	// subsystems is scheduled to run
	protected void interrupted() {

	}
}
