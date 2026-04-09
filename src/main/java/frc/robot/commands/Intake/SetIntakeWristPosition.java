package frc.robot.commands.Intake;

import frc.robot.RobotContainer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.util.MatchLog;

public class SetIntakeWristPosition extends Command {
	private double intakeWristPosition = 0;

	public SetIntakeWristPosition(double intakeWristPosition) {
		this.intakeWristPosition = intakeWristPosition;

		addRequirements(RobotContainer.intakeWrist);
	}

	// Called just before this Command runs the first time
	public void initialize() {
		RobotContainer.intakeWrist.setTargetPosition(intakeWristPosition);
	}

	// Called repeatedly when this Command is scheduled to run
	public void execute() {
		RobotContainer.intakeWrist.positionControl();
	}

	// Make this return true when this Command no longer needs to run execute()
	public boolean isFinished() {
			return RobotContainer.intakeWrist.isInPosition(intakeWristPosition);
	}

	// Called once after isFinished returns true
	protected void end() {
		MatchLog.event("cmd/intakeWrist/inPosition",
			String.format("target=%.3f actual=%.3f rot",
				intakeWristPosition, RobotContainer.intakeWrist.getCurrentPosition()));
	}

	// Called when another command which requires one or more of the same
	// subsystems is scheduled to run
	protected void interrupted() {
		MatchLog.event("cmd/intakeWrist/interrupted",
			String.format("target=%.3f actual=%.3f rot",
				intakeWristPosition, RobotContainer.intakeWrist.getCurrentPosition()));
	}
}
