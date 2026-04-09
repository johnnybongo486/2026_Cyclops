package frc.robot.commands.Hood;

import frc.robot.RobotContainer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.util.MatchLog;

public class SetHoodPosition extends Command {
	private double hoodPosition = 0;

	public SetHoodPosition(double hoodPosition) {
		this.hoodPosition = hoodPosition;

		addRequirements(RobotContainer.hood);
	}

	// Called just before this Command runs the first time
	public void initialize() {
		RobotContainer.hood.setTargetPosition(hoodPosition);
	}

	// Called repeatedly when this Command is scheduled to run
	public void execute() {
		RobotContainer.hood.positionControl();
	}

	// Make this return true when this Command no longer needs to run execute()
	public boolean isFinished() {
			return RobotContainer.hood.isInPosition(hoodPosition);
	}

	// Called once after isFinished returns true
	protected void end() {
		MatchLog.event("cmd/hood/inPosition",
			String.format("target=%.3f actual=%.3f rot",
				hoodPosition, RobotContainer.hood.getCurrentPosition()));
	}

	// Called when another command which requires one or more of the same
	// subsystems is scheduled to run
	protected void interrupted() {
		MatchLog.event("cmd/hood/interrupted",
			String.format("target=%.3f actual=%.3f rot",
				hoodPosition, RobotContainer.hood.getCurrentPosition()));
	}
}
