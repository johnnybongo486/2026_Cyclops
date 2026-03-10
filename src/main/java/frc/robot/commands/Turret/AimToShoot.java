package frc.robot.commands.Turret; //holds the code for the command group, says that this code is in turret command group

import frc.lib.LimelightHelpers;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.subsystems.ShooterLimelight;

import edu.wpi.first.wpilibj2.command.Command; //imports establish what will need to be called in the code later

public class AimToShoot extends Command {  //the code within the class AimToShoot decides 
                                            //the numbers necessary to make the robot function as it should

    private double tx;
    private double tDistance;

    private double hoodPosition = 0;
    private double shooterSpeed = 0;

    private double positionIncrement = 0.015;  // 0.015

    private double ySpeed = 0;
    private double xSpeed = 0;

    private ShooterLimelight limelight; 

    public AimToShoot() {
        this.limelight = RobotContainer.shooterLimelight; //the limelights are cameras, this section states 
        addRequirements(RobotContainer.shooterLimelight); //the requirements, which are also numbers, to make
        addRequirements(RobotContainer.shooter); //the subsystems function correctly alongside the limelights
        addRequirements(RobotContainer.hood);
        addRequirements(RobotContainer.turret);
        ;
    }

    public void initialize() {
    }
    
    @Override
    public void execute() {
        // If we don't see a target, scan
        boolean onTarget = true;
        onTarget = RobotContainer.turret.getOnTarget();
        
        if (!limelight.ifValidTag() || onTarget == false) {
            Robot.robotContainer.doubleRumble();  // rumble joystick
        } 

        else {
            Robot.robotContainer.stopRumble();

            tx = limelight.getX();
            tDistance = limelight.gettz() - 0.65;

            // Look for robot speed
            xSpeed = RobotContainer.turret.getXSpeed();
            ySpeed = RobotContainer.turret.getYSpeed();

            double tagID = LimelightHelpers.getFiducialID("limelight-shooter");

            //Distance affects Hood Angle and Shooter Speed
            hoodPosition = (-1.39455*tDistance*tDistance) + (8.71414*tDistance)+(-4.88011);
            shooterSpeed = (.740361*tDistance*tDistance) + (-1.05658*tDistance)+(27.0);

            //TX affects the turret rotation
            RobotContainer.turret.incrementTargetPosition((double) (tx*positionIncrement));

            //Send Values
            RobotContainer.shooter.setTargetVelocity(shooterSpeed);
            RobotContainer.hood.setTargetPosition(hoodPosition);

            //Move Subsystems
            RobotContainer.turret.positionControl();
            RobotContainer.shooter.velocityControl();
            RobotContainer.hood.positionControl();
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
   