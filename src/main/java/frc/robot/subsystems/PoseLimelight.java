package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.LimelightHelpers;
import frc.robot.DeviceIds;
import frc.robot.RobotContainer;

public class PoseLimelight extends SubsystemBase {

   NetworkTable tableLeft;
   NetworkTable tableRight;

   NetworkTableInstance Inst;

   private double blueLLForwardLeft = -0.254;
   private double blueLLForwardRight = -0.254;
   private double blueLLRightLeft = -0.3261;
   private double blueLLRightRight = 0.3261;
   private double blueLLUpLeft = 0.3095;
   private double blueLLUpRight = 0.3095;
   private double blueLLRollLeft = 0;
   private double blueLLRollRight = 0;
   private double blueLLPitchLeft = 30;
   private double blueLLPitchRight = 30;
   private double blueLLYawLeft = 90;
   private double blueLLYawRight = -90;


   private double redLLForwardLeft = 0.254;
   private double redLLForwardRight = 0.254;
   private double redLLRightLeft = 0.3261;
   private double redLLRightRight = -0.3161;
   private double redLLUpLeft = 0.3095;
   private double redLLUpRight = 0.3095;
   private double redLLRollLeft = 0;
   private double redLLRollRight = 0;
   private double redLLPitchLeft = 30;
   private double redLLPitchRight = 30;
   private double redLLYawLeft = -90;
   private double redLLYawRight = 90;



   public PoseLimelight() {
      Inst = NetworkTableInstance.getDefault();
      tableLeft = Inst.getTable(DeviceIds.Limelight.LeftTableName);
      tableRight = Inst.getTable(DeviceIds.Limelight.RightTableName);
   }

   public void updateLLPositions(){
      boolean isFacingBlue = RobotContainer.poseEst.getStartingPose();

      if (DriverStation.getAlliance().isPresent() == true) {
         if (DriverStation.getAlliance().get() == Alliance.Blue) {
            if (isFacingBlue == true) {
               LimelightHelpers.setCameraPose_RobotSpace("limelight-left", redLLForwardLeft, redLLRightLeft, redLLUpLeft, redLLRollLeft, redLLPitchLeft, redLLYawLeft);
               LimelightHelpers.setCameraPose_RobotSpace("limelight-right", redLLForwardRight, redLLRightRight, redLLUpRight, redLLRollRight, redLLPitchRight, redLLYawRight);
            }
            else {
               LimelightHelpers.setCameraPose_RobotSpace("limelight-left", blueLLForwardLeft, blueLLRightLeft, blueLLUpLeft, blueLLRollLeft, blueLLPitchLeft, blueLLYawLeft);
               LimelightHelpers.setCameraPose_RobotSpace("limelight-right", blueLLForwardRight, blueLLRightRight, blueLLUpRight, blueLLRollRight, blueLLPitchRight, blueLLYawRight);
            }

         }

         else if (DriverStation.getAlliance().get() == Alliance.Red) {
            if (isFacingBlue == true) {
               LimelightHelpers.setCameraPose_RobotSpace("limelight-left", redLLForwardLeft, redLLRightLeft, redLLUpLeft, redLLRollLeft, redLLPitchLeft, redLLYawLeft);
               LimelightHelpers.setCameraPose_RobotSpace("limelight-right", redLLForwardRight, redLLRightRight, redLLUpRight, redLLRollRight, redLLPitchRight, redLLYawRight);
            }
            else {
               LimelightHelpers.setCameraPose_RobotSpace("limelight-left", blueLLForwardLeft, blueLLRightLeft, blueLLUpLeft, blueLLRollLeft, blueLLPitchLeft, blueLLYawLeft);
               LimelightHelpers.setCameraPose_RobotSpace("limelight-right", blueLLForwardRight, blueLLRightRight, blueLLUpRight, blueLLRollRight, blueLLPitchRight, blueLLYawRight);
            }
         }
      }

      else{

      }
   }

   public void enabled(){
      NetworkTableInstance.getDefault().getTable("limelight-Shooter").getEntry("ledMode").setNumber(3);
      NetworkTableInstance.getDefault().getTable("limelight-Shooter").getEntry("throttle_set").setNumber(0);

   }

   public void disabled(){
      NetworkTableInstance.getDefault().getTable("limelight-Shooter").getEntry("ledMode").setNumber(3);
      NetworkTableInstance.getDefault().getTable("limelight-Shooter").getEntry("throttle_set").setNumber(200);
   }

   public void updateDashboard() {

	}
}
