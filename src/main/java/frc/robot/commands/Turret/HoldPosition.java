package frc.robot.commands.Turret;

import frc.robot.RobotContainer;
import edu.wpi.first.wpilibj2.command.Command;

public class HoldPosition extends Command {      

    private double targetAngle = 0;
    private double chassisAngle = 0;
    private double angleDifference = 0;
    private double turretPosition = 0;

    public HoldPosition(double targetAngle) {
        this.targetAngle = targetAngle;
        addRequirements(RobotContainer.turret);
    }

    public void initialize() {

    }
    
    @Override
    public void execute() {
        chassisAngle = RobotContainer.turret.getYaw().getDegrees();
        chassisAngle = chassisAngle%360;
        angleDifference = chassisAngle - targetAngle;
        turretPosition = angleDifference/9;
        RobotContainer.turret.setTargetPosition(turretPosition);
        RobotContainer.turret.positionControl();
    }

    // Make this return true when this Command no longer needs to run execute()
	public boolean isFinished() {
        // If all 3 PIDs are at their target, we're done
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
