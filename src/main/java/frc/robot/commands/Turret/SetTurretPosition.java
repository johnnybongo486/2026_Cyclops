package frc.robot.commands.Turret;

import frc.robot.RobotContainer;
import edu.wpi.first.wpilibj2.command.Command;

public class SetTurretPosition extends Command {
	private double turretPosition = 0;

	public SetTurretPosition(double turretPosition) {
		this.turretPosition = turretPosition;

		addRequirements(RobotContainer.turret);
	}

	// Called just before this Command runs the first time
	public void initialize() {
		RobotContainer.turret.setTargetPosition(turretPosition);
	}

	// Called repeatedly when this Command is scheduled to run
	public void execute() {
		RobotContainer.turret.positionControl();
	}

	// Make this return true when this Command no longer needs to run execute()
	public boolean isFinished() {
			return RobotContainer.turret.isInPosition(turretPosition);
	}

	// Called once after isFinished returns true
	protected void end() {

	}

	// Called when another command which requires one or more of the same
	// subsystems is scheduled to run
	protected void interrupted() {

	}
}
