package frc.robot.subsystems;

import java.util.Optional;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.LimelightHelpers;
import frc.robot.RobotContainer;

public class PoseEst extends SubsystemBase{

    public boolean doRejectUpdateLeft = false;
    public boolean doRejectUpdateRight = false;
    public boolean doRejectUpdateShooter = false;

    public boolean isPresent = false;
    private double theta = 0.0;
    private double robotAngle = 0.0;

    private Pose2d targetHubPoseRed = new Pose2d(11.901424,4.021328, Rotation2d.fromDegrees(0));
    private Pose2d targetHubPoseBlue = new Pose2d(4.611624,4.021328, Rotation2d.fromDegrees(0));

    private Pose2d targetHPSPassPoseBlue = new Pose2d(2,1.5, Rotation2d.fromDegrees(0));
    private Pose2d targetHPSPassPoseRed = new Pose2d(14.5,6.5, Rotation2d.fromDegrees(0));

    private Pose2d targetDepotPassPoseBlue = new Pose2d(2,6.5, Rotation2d.fromDegrees(0));
    private Pose2d targetDepotPassPoseRed = new Pose2d(14.5,1.5, Rotation2d.fromDegrees(0));



    private double tX = 0;
    private double tY = 0;
    private double xH = 0;
    private double yH = 0;
    private double prevHoodPos = 0;

    private boolean passingMode = true;
    private boolean isSafe = true;
    private GenericEntry startingPose;
    private boolean isShooting = false;
    private String shoot = "DO NOTHING";
    private String timing ="""
            2:10
            1:45
            1:20
            0:55
            0:30
            """;
    private double time;
    private double timeLeft;
    private boolean rejectLL;
    private double safeMin;
    private double safeMax;


    // Refreshed each loop in updatePose() — do not cache at construction time since FMS
    // may not have set the alliance yet when robot code first starts.
    public Optional<Alliance> alliance = Optional.empty();

    public PoseEst(){
        RobotContainer.drivetrain.getPigeon2().setYaw(0);

        startingPose = Shuffleboard.getTab("Robot Facing Blue Driver Station?")
            .add("Robot Facing Blue Driver Station?", false) // Initial value
            .withWidget("Toggle Button") // Makes it interactive
            .getEntry();
    }

    public void updatePose() {
        // Refresh alliance every loop so it's never stale from pre-FMS-connect startup
        alliance = DriverStation.getAlliance();

        //Send data to LL
        LimelightHelpers.SetRobotOrientation("limelight-right", RobotContainer.drivetrain.getPigeon2().getYaw().getValueAsDouble(), 0, 0, 0, 0, 0);
        LimelightHelpers.SetRobotOrientation("limelight-left", RobotContainer.drivetrain.getPigeon2().getYaw().getValueAsDouble(), 0, 0, 0, 0, 0);
        LimelightHelpers.SetRobotOrientation("limelight-shooter", RobotContainer.drivetrain.getPigeon2().getYaw().getValueAsDouble(), 0, 0, 0, 0, 0);
        
        //Pull relative tag location
        LimelightHelpers.PoseEstimate mt2LeftBlue = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("limelight-left");
        LimelightHelpers.PoseEstimate mt2RightBlue = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("limelight-right");
        LimelightHelpers.PoseEstimate mt2ShooterBlue = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("limelight-shooter");


        //init rejects
        doRejectUpdateLeft = true; // falses
        doRejectUpdateRight = true; 
        doRejectUpdateShooter = false;

        if (alliance.isPresent()) {

            if(/*mt2LeftBlue != null && mt2RightBlue != null &&*/ mt2ShooterBlue != null) { // make sure we have the camera        
                if(Math.abs(RobotContainer.drivetrain.getPigeon2().getAngularVelocityZDevice().getValueAsDouble()) > 720) {// if our angular velocity is greater than 720 degrees per second, ignore vision updates
                    doRejectUpdateLeft = true;
                    doRejectUpdateRight = true;
                    doRejectUpdateShooter = true;
                }
                /* 
                if(mt2LeftBlue.tagCount == 0) {
                    doRejectUpdateLeft = true;
                }

                else{}

                if(mt2RightBlue.tagCount == 0) {
                    doRejectUpdateRight = true;
                }

                else{}
                */

                if(mt2ShooterBlue.tagCount == 0) {
                    doRejectUpdateShooter = true;
                }
                else{}
                
                // use this if one LL works
                /*if( mt2LeftBlue.tagCount != 0 && mt2RightBlue.tagCount != 0 && Robot.robotContainer.driverController.leftTrigger().getAsBoolean() == true) {
                //if (Robot.robotContainer.driverController.leftTrigger().getAsBoolean() == true) {  // use this if no limelights
                    doRejectUpdateLeft = true;
                    //doRejectUpdateRight = true;
                    rejectLL = true;
                }
                
                else {
                    rejectLL = false;
                }
                    */

                if(!doRejectUpdateLeft) {
                    RobotContainer.drivetrain.addVisionMeasurement(mt2LeftBlue.pose, mt2LeftBlue.timestampSeconds, VecBuilder.fill(0.7,0.7,99999)); // n1: 0.7
                }

                if(!doRejectUpdateRight) {
                    RobotContainer.drivetrain.addVisionMeasurement(mt2RightBlue.pose, mt2RightBlue.timestampSeconds, VecBuilder.fill(0.7,0.7,99999)); // n1: 0.7
                }

                if(!doRejectUpdateShooter) {
                    RobotContainer.drivetrain.addVisionMeasurement(mt2ShooterBlue.pose, mt2ShooterBlue.timestampSeconds, VecBuilder.fill(0.7,0.7,99999)); // n1: 0.7
                }
            } 
            
            else {
                System.out.println("PoseEst.java: mt2LeftBlue is null and mt2RightBlue is null");
            }  
        }

        else{
            
        }
    }  
    
    public double aimToHub() {
        
        Pose2d currentPose = RobotContainer.drivetrain.getState().Pose;

        if (DriverStation.getAlliance().isPresent() == true) {

            if (alliance.get() == Alliance.Red) {
                robotAngle = currentPose.getRotation().getDegrees();
                tX = currentPose.getX() - targetHubPoseRed.getX();
                tY = currentPose.getY() - targetHubPoseRed.getY();
                theta = (Math.toDegrees(Math.atan(tY/tX)));  // was 180+

            }

            else {
                robotAngle = currentPose.getRotation().getDegrees();
                tX = currentPose.getX() - targetHubPoseBlue.getX();
                tY = currentPose.getY() - targetHubPoseBlue.getY();
                theta = Math.toDegrees(Math.atan(tY/tX));
            }

            return theta;
        }

        else {
            return 0;
        }
        
    }

    public double aimToPass() {
        
        Pose2d currentPose = RobotContainer.drivetrain.getState().Pose;

        if (DriverStation.getAlliance().isPresent() == true) {

            if (alliance.get() == Alliance.Red) {
                robotAngle = currentPose.getRotation().getDegrees();

                // aim based on where we are on the field
                if (currentPose.getY() >= 4) {
                    tX = currentPose.getX() - targetHPSPassPoseRed.getX();
                    tY = currentPose.getY() - targetHPSPassPoseRed.getY();
                } else {
                    tX = currentPose.getX() - targetDepotPassPoseRed.getX();
                    tY = currentPose.getY() - targetDepotPassPoseRed.getY();
                }

                theta = (Math.toDegrees(Math.atan(tY/tX)));  // was 180+

            }

            else {
                robotAngle = currentPose.getRotation().getDegrees();

                // aim based on where we are on the field
                if (currentPose.getY() <= 4) {
                    tX = currentPose.getX() - targetHPSPassPoseBlue.getX();
                    tY = currentPose.getY() - targetHPSPassPoseBlue.getY();
                } else {
                    tX = currentPose.getX() - targetDepotPassPoseBlue.getX();
                    tY = currentPose.getY() - targetDepotPassPoseBlue.getY();                    
                }

                theta = Math.toDegrees(Math.atan(tY/tX));
            }

            return theta;
        }

        else {
            return 0;
        }
        
    }

    public boolean getPassingMode(){

        Pose2d currentPose = RobotContainer.drivetrain.getState().Pose;

        if (DriverStation.getAlliance().isPresent() == true) {

            if (alliance.get() == Alliance.Red) {
                if (currentPose.getX() >= 11.9) {
                    passingMode = false;
                }
                else {
                    passingMode = true;
                }
            }

            else {
                if (currentPose.getX() <= 4.61) {
                    passingMode = false;
                }
                else {
                    passingMode = true;
                } 
            }
            return passingMode;   
        }

        else {
            return false;
        }
    }

    public boolean getIsSafe(){

        Pose2d currentPose = RobotContainer.drivetrain.getState().Pose;
        double currentVelocity = RobotContainer.drivetrain.getXSpeed();

        if (DriverStation.getAlliance().isPresent() == true) {

            if (alliance.get() == Alliance.Red) {
                
                // Check current velocity and set safe zone
                if (Math.abs(currentVelocity) > 3) {
                    safeMin = 10.1;
                    safeMax = 13.5;
                }

                else if (Math.abs(currentVelocity) > 1 && Math.abs(currentVelocity) <= 3 ) {
                    safeMin = 10.6;
                    safeMax = 13;
                }

                else if (Math.abs(currentVelocity) >= 0 && Math.abs(currentVelocity) <= 1 ) {
                    safeMin = 11.1;
                    safeMax = 12.5;
                }

                else{

                }

                // Set Min Max based on velocity
                if (currentPose.getX() >= safeMin && currentPose.getX() <= safeMax) {
                    isSafe = false;

                    // save where the hood was prior to entering the danger zone
                    prevHoodPos = RobotContainer.hood.getCurrentPosition();
                }

                else {
                    if(isSafe == false) {
                        // this is transition from unsafe to safe; reset the hood postion to how it was
                        // prior to entering the danger zone
                        RobotContainer.hood.setTargetPosition(prevHoodPos);
			            RobotContainer.hood.positionControl();
                    }

                    isSafe = true;
                }
            }

            else {
                // Check current velocity and set safe zone
                if (Math.abs(currentVelocity) > 3) {
                    safeMin = 2.8;
                    safeMax = 6.8;
                }

                else if (Math.abs(currentVelocity) > 1 && Math.abs(currentVelocity) <= 3 ) {
                    safeMin = 3.3;
                    safeMax = 6.3;
                }

                else if (Math.abs(currentVelocity) >= 0 && Math.abs(currentVelocity) <= 1 ) {
                    safeMin = 4.0;
                    safeMax = 5.3;
                }

                else{

                }

                // Set Min Max based on velocity
                if (currentPose.getX() >= safeMin && currentPose.getX() <= safeMax) { // 4.0, 5.3
                    isSafe = false;

                    // save where the hood was prior to entering the danger zone
                    prevHoodPos = RobotContainer.hood.getCurrentPosition();
                }
                else {
                    if(isSafe == false) {
                        // this is transition from unsafe to safe; reset the hood postion to how it was
                        // prior to entering the danger zone
                        RobotContainer.hood.setTargetPosition(prevHoodPos);
			            RobotContainer.hood.positionControl();
                    }

                    isSafe = true;
                } 
            }
            return isSafe;   
        }

        else {
            return false;
        }
    }

    public double distanceTarget() {
        Pose2d currentPose = RobotContainer.drivetrain.getState().Pose;

        double xR = currentPose.getX();
        double yR = currentPose.getY();

        // Read passing mode fresh here — avoids depending on the caller having called
        // getPassingMode() first to update the stale passingMode field.
        boolean currentlyPassing = getPassingMode();

        if (DriverStation.getAlliance().isPresent() == true) {

            if (DriverStation.getAlliance().get() == Alliance.Red) {
                if (!currentlyPassing) {
                    xH = targetHubPoseRed.getX();
                    yH = targetHubPoseRed.getY();
                } else if (currentPose.getY() >= 4) {
                    xH = targetHPSPassPoseRed.getX();
                    yH = targetHPSPassPoseRed.getY();
                } else {
                    xH = targetDepotPassPoseRed.getX();
                    yH = targetDepotPassPoseRed.getY();
                }
            }
            else {
                if (!currentlyPassing) {
                    xH = targetHubPoseBlue.getX();
                    yH = targetHubPoseBlue.getY();
                } else if (currentPose.getY() <= 4) {
                    xH = targetHPSPassPoseBlue.getX();
                    yH = targetHPSPassPoseBlue.getY();
                } else {
                    xH = targetDepotPassPoseBlue.getX();
                    yH = targetDepotPassPoseBlue.getY();
                }
            }

            double deltaX = xR - xH;
            double deltaY = yR - yH;

            double distance = Math.sqrt((deltaX*deltaX)+(deltaY*deltaY));

            return distance;
        }

        else {
            return 0;
        }
    }

    public double getTheta() {
        return theta;
    }

    public double getRobotAngle() {
        return robotAngle;
    }

    public boolean getStartingPose() {
        return startingPose.getBoolean(true);
    }

    public boolean allianceWinAuto() {
        String gameData = DriverStation.getGameSpecificMessage();
        if (gameData.length() > 0) {
            if (DriverStation.getAlliance().get() == Alliance.Red) {
                switch (gameData.charAt(0)) {
                    case 'B':
                        isShooting = false;
                        break;
                    case 'R':
                        isShooting = true;
                        break;
                    default:
                        break;
                }
            }
            else {
                switch (gameData.charAt(0)) {
                    case 'R':
                        isShooting = false;
                        break;
                    case 'B':
                        isShooting = true;
                        break;
                    default:
                        break;
                }
            }
        }
        else {

        }

        return isShooting;
    }

    public String getShootingWords() {
        if (allianceWinAuto() == false) {
            shoot = "SHOOT!";
        }
        else {
            shoot = "PASS!";
        }

        return shoot;
    }

    public double countdown() {

        time = Timer.getMatchTime();

        if (time < 140 && time > 130){
            timeLeft = time - 130;
        }

        else if (time < 130 && time > 105) {
            timeLeft = time - 105;
        }

        else if (time < 105 && time > 80) {
            timeLeft = time - 80;
        }

        else if (time < 80 && time > 55) {
            timeLeft = time - 55;
        }

        else if (time < 55 && time > 30) {
            timeLeft = time - 30;
        }

        else if (time < 30 && time > 0 && DriverStation.isTeleop()) {
            timeLeft = time - 0;
        }

        else {
            timeLeft = time - 0;
        }

        return timeLeft;
    }

    public void updateDashboard() {        
        aimToHub();
        double x = RobotContainer.drivetrain.getState().Pose.getX();
        double y = RobotContainer.drivetrain.getState().Pose.getY();
        double r = RobotContainer.drivetrain.getState().Pose.getRotation().getDegrees();
        SmartDashboard.putNumber("PoseX", x);
        SmartDashboard.putNumber("PoseY", y);
        SmartDashboard.putNumber("PoseR", r);
        SmartDashboard.putNumber("THETA", theta);
        SmartDashboard.putNumber("ROBOTANGLE", robotAngle); 
        SmartDashboard.putBoolean("Passing Mode", getPassingMode());
        SmartDashboard.putBoolean("Is Safe", getIsSafe());
        SmartDashboard.putNumber("Distance", distanceTarget());
        SmartDashboard.putBoolean("Robot Facing Blue Driver Station", getStartingPose());
        SmartDashboard.putNumber("Match Time", DriverStation.getMatchTime());
        SmartDashboard.putBoolean("Win Auto?", allianceWinAuto());
        SmartDashboard.putString("Shoot?", getShootingWords());
        SmartDashboard.putString("Match Timing", timing);
        SmartDashboard.putNumber("TimeLeft", countdown());
        SmartDashboard.putBoolean("REJECT LL", rejectLL);
    }
}