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

	// Hold velocity for as long as the command is scheduled. Ending on reaching
	// the setpoint causes the parallel group (used in fixed-shot bindings) to
	// finish, after which Phoenix 6 control packets time out and the shooter
	// neutralizes mid-shot. Reaching the setpoint is "ready", not "done".
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
