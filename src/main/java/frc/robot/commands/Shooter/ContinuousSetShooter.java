package frc.robot.commands.Shooter;

import frc.robot.Constants;
import frc.robot.RobotContainer;
import edu.wpi.first.wpilibj2.command.Command;


public class ContinuousSetShooter extends Command {
    private double shooterSpeed = 0;

    private double tDistance;

    private double newDistance = 0;

	private boolean isPassingMode = false;


	public ContinuousSetShooter() {
		addRequirements(RobotContainer.shooter);
	}

	// Called just before this Command runs the first time
	public void initialize() {
		// RobotContainer.shooter.setTargetVelocity(0);
	}

	// Called repeatedly when this Command is scheduled to run
	public void execute() {
		isPassingMode = RobotContainer.poseEst.getPassingMode();
		newDistance = RobotContainer.poseEst.distanceTarget();

		tDistance = newDistance;

		if(isPassingMode == false) {
			// set the shooter speed shooting
			shooterSpeed = (Constants.Shooter.ShooterSpeed.a*tDistance*tDistance) + (Constants.Shooter.ShooterSpeed.b*tDistance)+(Constants.Shooter.ShooterSpeed.c); 

		} else {
			// set the shooter speed for passing
			shooterSpeed = (Constants.Shooter.ShooterPassing.a*tDistance*tDistance) + (Constants.Shooter.ShooterPassing.b*tDistance)+(Constants.Shooter.ShooterPassing.c); 
		}

		//Send Values
		RobotContainer.shooter.setTargetVelocity(shooterSpeed);

		//Move Subsystems
		RobotContainer.shooter.velocityControl();		
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
