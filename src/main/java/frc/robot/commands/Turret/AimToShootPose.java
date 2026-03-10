package frc.robot.commands.Turret; //holds the code for the command group, says that this code is in turret command group

import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.subsystems.ShooterLimelight;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command; //imports establish what will need to be called in the code later

public class AimToShootPose extends Command {  //the code within the class AimToShoot decides 
                                            //the numbers necessary to make the robot function as it should

    private double tx;
    private double tDistance;

    private double hoodPosition = 0;
    private double shooterSpeed = 0;

    private double positionIncrement = 0.015;  // 0.015

    private double deltaDistance = 0;
    private double oldDistance = 0;
    private double newDistance = 0;

    private double deltaTheta = 0;
    private double oldTheta = 0;
    private double newTheta = 0;

    private double deltaThetaPass = 0;
    private double oldThetaPass = 0;
    private double newThetaPass = 0;

    private double currentAngle;
    private double pointAngle = 0;

    private boolean isPassingMode = false;


    private ShooterLimelight limelight; 

    public AimToShootPose() {
        this.limelight = RobotContainer.shooterLimelight; //the limelights are cameras, this section states 
        addRequirements(RobotContainer.shooterLimelight); //the requirements, which are also numbers, to make
        addRequirements(RobotContainer.shooter); //the subsystems function correctly alongside the limelights
        addRequirements(RobotContainer.hood);
        addRequirements(RobotContainer.turret);
    }

    public void initialize() {
        oldDistance = RobotContainer.poseEst.distanceTarget();
        oldTheta = RobotContainer.poseEst.aimToHub();
        oldThetaPass = RobotContainer.poseEst.aimToPass();

    }
    
    @Override
    public void execute() {

        // If we don't see a target, scan
        boolean onTarget = RobotContainer.turret.getOnTarget();

        newDistance = RobotContainer.poseEst.distanceTarget();
        newTheta = RobotContainer.poseEst.aimToHub();
        newThetaPass = RobotContainer.poseEst.aimToPass();


        deltaDistance = oldDistance - newDistance;
        double speedD = deltaDistance / 0.02;

        deltaTheta = newTheta - oldTheta;
        double speedT = deltaTheta / 0.02;

        deltaThetaPass = newThetaPass - oldThetaPass;
        double speedTP = deltaThetaPass / 0.02;

        currentAngle = RobotContainer.drivetrain.getState().Pose.getRotation().getDegrees();

        isPassingMode = RobotContainer.poseEst.getPassingMode();      
        
            // Shooting Mode
            if (isPassingMode == false) {

                // if (!limelight.ifValidTag() || onTarget == false) {
                if (onTarget == false) {
                    Robot.robotContainer.doubleRumble();  // rumble joystick
                } 
                
                else {  
                    Robot.robotContainer.stopRumble();

                    tx = limelight.getX();
                    // tx = RobotContainer.drivetrain.getState().Pose.getX();
                    tDistance = limelight.gettz() + 0.65;
                    // tDistance = RobotContainer.poseEst.distanceTarget();

                    tx = tx - speedT*1.2;//the target pose based on relative x and y speed of the robot
                    tDistance = tDistance - speedD*1.2;
                    

                    //Distance affects Hood Angle and Shooter Speed
                    hoodPosition = (-1.39455*tDistance*tDistance) + (8.71414*tDistance)+(-4.88011);
                    shooterSpeed = (.740361*tDistance*tDistance) + (-1.05658*tDistance)+(27.0); // 26.67761
                    shooterSpeed = shooterSpeed*1.33333333333;

                    
                    //TX affects the turret rotation
                    RobotContainer.turret.incrementTargetPosition((double) (tx*positionIncrement));

                    //Send Values
                    RobotContainer.shooter.setTargetVelocity(shooterSpeed);
                    RobotContainer.hood.setTargetPosition(hoodPosition);

                    //Move Subsystems
                    RobotContainer.turret.positionControl();
                    RobotContainer.shooter.velocityControl();

                    // check if the hood is in a safe place before moving it
                    if(RobotContainer.poseEst.getIsSafe() == true) {
                        RobotContainer.hood.positionControl();
                    } else {
                        RobotContainer.hood.setTargetPosition(Constants.Shooter.Hood.StoreHoodPosition);
                        RobotContainer.hood.positionControl();
                    }
                }

            }

            // PASSING MODE
            else {  
                Robot.robotContainer.stopRumble();

                tDistance = newDistance;

                if (DriverStation.getAlliance().get() == Alliance.Red) {
                    if (currentAngle > 0) {
                        pointAngle = - newThetaPass + currentAngle;  // was -180-
                    }
                    else {
                        pointAngle = - newThetaPass + currentAngle;    // was 180-
                    }
                }

                else {
                    if (currentAngle > 0) {
                        pointAngle = -180 - newThetaPass + currentAngle;  // was -180-
                    }
                    else {
                        pointAngle = 180 - newThetaPass + currentAngle;    // was 180-
                    }
                }

                SmartDashboard.putNumber("POINTANGLESHOOT", pointAngle);


                pointAngle = pointAngle - speedTP*1.2;//the target pose based on relative x and y speed of the robot
                tDistance = tDistance - speedD*1.2;

                
                // Convert Angle to Ticks
                double targetPosition = pointAngle / 9.2571428;
                    

                //Distance affects Hood Angle and Shooter Speed
                hoodPosition = (-0.269210*tDistance*tDistance) + (4.12897*tDistance)+(-7.24743); // -8.24743
                shooterSpeed = (-0.5112327*tDistance*tDistance) + (11.38087*tDistance) + (7.48918);

                    
                // Set Target Position
                RobotContainer.turret.setTargetPosition(targetPosition);

                //Send Values
                RobotContainer.shooter.setTargetVelocity(shooterSpeed);
                RobotContainer.hood.setTargetPosition(hoodPosition);

                //Move Subsystems
                RobotContainer.turret.positionControl();
                RobotContainer.shooter.velocityControl();
                
                // check if the hood is in a safe place before moving it
                if(RobotContainer.poseEst.getIsSafe() == true) {
                    RobotContainer.hood.positionControl();
                } else {
                    RobotContainer.hood.setTargetPosition(Constants.Shooter.Hood.StoreHoodPosition);
                    RobotContainer.hood.positionControl();
                }
            }
        oldDistance = newDistance;
        oldTheta = newTheta;
        oldThetaPass = newThetaPass;  
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
