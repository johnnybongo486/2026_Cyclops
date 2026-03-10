package frc.robot.commands.Turret; //holds the code for the command group, says that this code is in turret command group

import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.commands.Swerve.TeleopDrive;
import frc.robot.subsystems.CommandSwerveDrivetrain;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command; //imports establish what will need to be called in the code later

public class AimToShootPoseOnly extends Command {  //the code within the class AimToShoot decides 
                                            //the numbers necessary to make the robot function as it should

    private SwerveRequest.FieldCentric poseDrive;
    
    private double tDistance;
    private final PIDController angleController = new PIDController(0.0065, 0, 0.0001);
    private double rotationVal;

    private double hoodPosition = 0;
    private double shooterSpeed = 0;

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

    public AimToShootPoseOnly() {
        addRequirements(RobotContainer.shooter); //the subsystems function correctly alongside the limelights
        addRequirements(RobotContainer.hood);
        addRequirements(RobotContainer.turret);
    }

    public void initialize() {
        oldDistance = RobotContainer.poseEst.distanceTarget();
        oldTheta = RobotContainer.poseEst.aimToHub();
        oldThetaPass = RobotContainer.poseEst.aimToPass();
        angleController.setTolerance(0.05);  // needs to be tuned

    }
    
    @Override
    public void execute() {
        boolean onTarget = RobotContainer.turret.getOnTarget();

        newDistance = RobotContainer.poseEst.distanceTarget();
        newTheta = RobotContainer.poseEst.aimToHub(); // angle from 0 to the goal
        newThetaPass = RobotContainer.poseEst.aimToPass(); // angle from 0 to the passing target


        deltaDistance = oldDistance - newDistance; // change in distance
        double speedD = deltaDistance / 0.02; // how fast we're moving towards or away

        deltaTheta = newTheta - oldTheta; // change in angle to hub
        double speedT = deltaTheta / 0.02; // rate of theta change to hub

        deltaThetaPass = newThetaPass - oldThetaPass; // change in angle to passing target
        double speedTP = deltaThetaPass / 0.02; // rate of theta change to passing target

        currentAngle = RobotContainer.drivetrain.getState().Pose.getRotation().getDegrees();

        isPassingMode = RobotContainer.poseEst.getPassingMode();

        
        
            // Shooting Mode
            if (isPassingMode == false) {
                // calculate point angle and target distance
                if (DriverStation.getAlliance().get() == Alliance.Red) {
                    if (currentAngle > 0) {
                        pointAngle = -180 - newTheta; // + currentAngle;
                    }   
                    else {
                        pointAngle = 180 - newTheta; // + currentAngle;
                    }
                }

                else {
                    pointAngle = newTheta; // currentAngle - newTheta;
                }
                
                
                if (Math.abs(speedD) > 0 || Math.abs(speedT) > 0) { // tune this value for what is considered moving
                    double distanceMultiplier = newDistance*1;  // tune this value to determine how much distance affects overaim
                    pointAngle = (pointAngle - speedT*0.9);//*distanceMultiplier);  // tune this value to see how much speed affects overaim L/R
                    tDistance = (newDistance - speedD);//*distanceMultiplier);  // tune this value to see how much speed affects overaim F/B
                }

                else {
                    pointAngle = pointAngle;
                    tDistance = newDistance;
                }

                // Convert Angle to Ticks
                // double targetPosition = pointAngle / 9.2571428;

                // Set Target Position
                RobotContainer.turret.setTargetPosition(0.0);

                if (onTarget == false) {
                  Robot.robotContainer.doubleRumble();  // rumble joystick
                } 
                
                else {  
                    Robot.robotContainer.stopRumble();

                    //Distance affects Hood Angle and Shooter Speed
                    hoodPosition = (Constants.Shooter.HoodShooting.a*tDistance*tDistance) + (Constants.Shooter.HoodShooting.b*tDistance)+(Constants.Shooter.HoodShooting.c);
                    shooterSpeed = (Constants.Shooter.ShooterSpeed.a*tDistance*tDistance) + (Constants.Shooter.ShooterSpeed.b*tDistance)+(Constants.Shooter.ShooterSpeed.c); 

                    //Send Values
                    RobotContainer.shooter.setTargetVelocity(shooterSpeed);
                    RobotContainer.hood.setTargetPosition(hoodPosition);

                    //Move Subsystems
                    RobotContainer.turret.positionControl();
                    RobotContainer.shooter.velocityControl();
                    
                    RobotContainer.hood.positionControl();

                    }
                }

            

            // PASSING MODE
            else {  
                Robot.robotContainer.stopRumble();

                tDistance = newDistance;

                if (DriverStation.getAlliance().get() == Alliance.Red) {
                    if (currentAngle > 0) {
                        pointAngle = - newThetaPass;// + currentAngle;  // was -180-
                    }
                    else {
                        pointAngle = - newThetaPass; // + currentAngle;    // was 180-
                    }
                }

                else {
                    if (currentAngle > 0) {
                        pointAngle = -180 - newThetaPass; // + currentAngle;  // was -180-
                    }
                    else {
                        pointAngle = 180 - newThetaPass; // + currentAngle;    // was 180-
                    }
                }

                SmartDashboard.putNumber("POINTANGLESHOOT", pointAngle);


                pointAngle = pointAngle - speedTP*1.2;//the target pose based on relative x and y speed of the robot
                tDistance = tDistance - speedD*1.2;

                rotationVal = angleController.calculate(currentAngle,pointAngle);

                // Convert Angle to Ticks
                // double targetPosition = pointAngle / 9.2571428;                    

                //Distance affects Hood Angle and Shooter Speed
                hoodPosition = (Constants.Shooter.HoodPassing.a*tDistance*tDistance) + (Constants.Shooter.HoodPassing.b*tDistance)+(Constants.Shooter.HoodPassing.c);
                shooterSpeed = (Constants.Shooter.ShooterPassing.a*tDistance*tDistance) + (Constants.Shooter.ShooterPassing.b*tDistance)+(Constants.Shooter.ShooterPassing.c); 
      
                // Set Target Position
                RobotContainer.drivetrain.setControl(poseDrive
                    .withVelocityX(distanceVal * 5.2) // Drive forward with negative Y (forward)
                    .withVelocityY(strafeVal * 5.2) // Drive left with negative X (left)
                    .withRotationalRate(pointAngle * 0.75)
                );

                RobotContainer.turret.setTargetPosition(0.0);

                //Send Values
                RobotContainer.shooter.setTargetVelocity(shooterSpeed);
                RobotContainer.hood.setTargetPosition(hoodPosition);

                //Move Subsystems
                RobotContainer.turret.positionControl();
                RobotContainer.shooter.velocityControl();
                
                // check if the hood is in a safe place before moving it
                RobotContainer.hood.positionControl();

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
