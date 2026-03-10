package frc.robot.commands.Turret;

import frc.robot.RobotContainer;
import edu.wpi.first.wpilibj2.command.Command;

public class ZeroTurret extends Command {

	public ZeroTurret() {

		addRequirements(RobotContainer.turret);
	}

	// Called just before this Command runs the first time
	public void initialize() {
		RobotContainer.turret.resetturretEncoder();
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
