package frc.robot.commands.Shooter;

import frc.robot.RobotContainer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.util.MatchLog;

public class ShooterAdderCommand extends Command {
	private double shooterVelocity = 0;

	public ShooterAdderCommand(double shootervelocity) {
		this.shooterVelocity = shootervelocity;
	}

	// Called just before this Command runs the first time
	public void initialize() {
		RobotContainer.shooter.setShooterAdder(shooterVelocity);
		if (shooterVelocity != 0) {
			MatchLog.event("cmd/shoot/fire",
				String.format("adder=%.2f baseVel=%.2f", shooterVelocity, RobotContainer.shooter.getTargetVelocity()));
		} else {
			MatchLog.event("cmd/shoot/adderCleared");
		}
	}

	// Called repeatedly when this Command is scheduled to run
	public void execute() {
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
