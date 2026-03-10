package frc.robot.commands.Turret;

import frc.robot.RobotContainer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

public class AutoAimPose extends Command {      

    private double theta;
    private double thetaPass;
    private double deltaThetaPass = 0;
    private double oldThetaPass = 0;
    private double currentAngle;
    private double pointAngle = 0;

    private boolean isPassingMode = false;

    public AutoAimPose() {
       addRequirements(RobotContainer.turret);

    }

    public void initialize() {
        oldThetaPass = RobotContainer.poseEst.aimToPass(); 
    }
    
    @Override
    public void execute() {
        // If we don't see a target, don't do anything
        theta = RobotContainer.poseEst.aimToHub();
        thetaPass = RobotContainer.poseEst.aimToPass();

        deltaThetaPass = thetaPass - oldThetaPass;
        double speedTP = deltaThetaPass / 0.02;        

        isPassingMode = RobotContainer.poseEst.getPassingMode();

        currentAngle = RobotContainer.drivetrain.getState().Pose.getRotation().getDegrees();

        // Shooting Mode
        if (isPassingMode == false) {
            if (DriverStation.getAlliance().get() == Alliance.Red) {
                if (currentAngle > 0) {
                    pointAngle = -180 - theta + currentAngle;
                }
                else {
                    pointAngle = 180 - theta + currentAngle;
                }
            }

            else {
                pointAngle = currentAngle - theta;
            }
        }

        //Passing Mode
        else {
            if (DriverStation.getAlliance().get() == Alliance.Red) {
                if (currentAngle > 0) {
                    pointAngle = - thetaPass + currentAngle;  // was -180-
                }
                else {
                    pointAngle = - thetaPass + currentAngle;    // was 180-
                }
            }

            else {
                if (currentAngle > 0) {
                    pointAngle = -180 - thetaPass + currentAngle;  // was -180-
                }
                else {
                    pointAngle = 180 - thetaPass + currentAngle;    // was 180-
                }
            }

            pointAngle = pointAngle - speedTP * 1.0; // 0.9 the target pose based on relative x and y speed of the robot
        }

        SmartDashboard.putNumber("POINTANGLE", pointAngle);
        
        // Convert Angle to Ticks
        // double targetPosition = pointAngle / 9.2571428;
        
        // Set Target Position
        RobotContainer.turret.setTargetPosition(0.0);

        if(RobotContainer.poseEst.getIsSafe() == true) {
			RobotContainer.turret.positionControl();
		} else {
			RobotContainer.turret.setTargetPosition(0);
			RobotContainer.turret.positionControl();
		}

        oldThetaPass = thetaPass;
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
