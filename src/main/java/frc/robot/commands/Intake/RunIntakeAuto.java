package frc.robot.commands.Intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.Constants;

public class RunIntakeAuto extends Command {
    
    public RunIntakeAuto() {
        addRequirements(RobotContainer.intake);
    }

	// Called just before this Command runs the first time
	public void initialize() {

	}

	// Called repeatedly when this Command is scheduled to run
	public void execute() {
		if (RobotContainer.drivetrain.getState().Speeds.vxMetersPerSecond < -0.1) {
			if (RobotContainer.poseEst.getIsSafeIntake() == true){
				RobotContainer.intake.setSpeed(Constants.Intake.IntakeRoller.IntakeRunSpeed);
			}

			else {
        		RobotContainer.intake.setSpeed(0.4);
			}
		}

		else {
			RobotContainer.intake.setSpeed(Constants.Intake.IntakeRoller.IntakeStopSpeed);
		}
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
