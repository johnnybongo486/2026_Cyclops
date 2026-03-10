package frc.robot.commands.Turret;

import frc.robot.RobotContainer;
import frc.robot.subsystems.ShooterLimelight;

import edu.wpi.first.wpilibj2.command.Command;

public class AutoAim extends Command {      

    private double tx;

    private double positionIncrement = 0.015;

    private ShooterLimelight limelight; 

    public AutoAim() {
        this.limelight = RobotContainer.shooterLimelight;
        addRequirements(RobotContainer.shooterLimelight);
        addRequirements(RobotContainer.turret);
        ;
    }

    public void initialize() {
    }
    
    @Override
    public void execute() {
        // If we don't see a target, don't do anything
        if (!limelight.ifValidTag()) {
            RobotContainer.turret.incrementTargetPosition((double)(0.2));
        } 
        
        else {

            // find target location
            tx = limelight.getX();

            //TX affects the turret rotation
            RobotContainer.turret.incrementTargetPosition((double) (tx*positionIncrement));

            //Move Subsystems
            RobotContainer.turret.positionControl();
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
        end(); 
	}

} 
