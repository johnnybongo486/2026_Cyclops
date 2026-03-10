package frc.robot.commands.Serializer;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.Constants;

public class RunAgitator extends Command {
    
    public RunAgitator() {
        addRequirements(RobotContainer.agitator);
    }

	// Called just before this Command runs the first time
	public void initialize() {

	}

	// Called repeatedly when this Command is scheduled to run
	public void execute() {
        RobotContainer.agitator.setSpeed(Constants.Serializer.Agitator.AgitatorRunSpeed);
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
