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
   NetworkTable tableShooter;

   NetworkTableInstance Inst;

   //  Blue Side
   private double blueLLForwardLeft = -0.127;
   private double blueLLForwardRight = -0.127;
   private double blueLLForwardShooter = 0.28575;

   private double blueLLRightLeft = -0.3302;
   private double blueLLRightRight = 0.3302;
   private double blueLLRightShooter = 0;

   private double blueLLUpLeft = 0.1556;
   private double blueLLUpRight = 0.1556;
   private double blueLLUpShooter = 0.254;

   private double blueLLRollLeft = 0;
   private double blueLLRollRight = 0;
   private double blueLLRollShooter = 0;

   private double blueLLPitchLeft = 30;
   private double blueLLPitchRight = 30;
   private double blueLLPitchShooter = 30;

   private double blueLLYawLeft = 90;
   private double blueLLYawRight = -90;
   private double blueLLYawShooter = 0;

   // Red Side
   private double redLLForwardLeft = 0.127;
   private double redLLForwardRight = 0.127;
   private double redLLForwardShooter = -0.28575;

   private double redLLRightLeft = 0.3302;
   private double redLLRightRight = -0.3302;
   private double redLLRightShooter = 0;

   private double redLLUpLeft = 0.1556;
   private double redLLUpRight = 0.1556;
   private double redLLUpShooter = 0.254;

   private double redLLRollLeft = 0;
   private double redLLRollRight = 0;
   private double redLLRollShooter = 0;

   private double redLLPitchLeft = 30;
   private double redLLPitchRight = 30;
   private double redLLPitchShooter = 30;

   private double redLLYawLeft = -90;
   private double redLLYawRight = 90;
   private double redLLYawShooter = 180;

   public PoseLimelight() {
      Inst = NetworkTableInstance.getDefault();
      tableLeft = Inst.getTable(DeviceIds.Limelight.LeftTableName);
      tableRight = Inst.getTable(DeviceIds.Limelight.RightTableName);
      tableShooter = Inst.getTable(DeviceIds.Limelight.ShooterTableName);

   }

   public void updateLLPositions(){
      boolean isFacingBlue = RobotContainer.poseEst.getStartingPose();

      if (DriverStation.getAlliance().isPresent() == true) {
         if (DriverStation.getAlliance().get() == Alliance.Blue) {
            if (isFacingBlue == true) {
               LimelightHelpers.setCameraPose_RobotSpace("limelight-left", redLLForwardLeft, redLLRightLeft, redLLUpLeft, redLLRollLeft, redLLPitchLeft, redLLYawLeft);
               LimelightHelpers.setCameraPose_RobotSpace("limelight-right", redLLForwardRight, redLLRightRight, redLLUpRight, redLLRollRight, redLLPitchRight, redLLYawRight);
               LimelightHelpers.setCameraPose_RobotSpace("limelight-shooter", redLLForwardShooter, redLLRightShooter, redLLUpShooter, redLLRollShooter, redLLPitchShooter, redLLYawShooter);
            }
            else {
               LimelightHelpers.setCameraPose_RobotSpace("limelight-left", blueLLForwardLeft, blueLLRightLeft, blueLLUpLeft, blueLLRollLeft, blueLLPitchLeft, blueLLYawLeft);
               LimelightHelpers.setCameraPose_RobotSpace("limelight-right", blueLLForwardRight, blueLLRightRight, blueLLUpRight, blueLLRollRight, blueLLPitchRight, blueLLYawRight);
               LimelightHelpers.setCameraPose_RobotSpace("limelight-shooter", blueLLForwardShooter, blueLLRightShooter, blueLLUpShooter, blueLLRollShooter, blueLLPitchShooter, blueLLYawShooter);
            }

         }

         else if (DriverStation.getAlliance().get() == Alliance.Red) {
            if (isFacingBlue == true) {
               LimelightHelpers.setCameraPose_RobotSpace("limelight-left", redLLForwardLeft, redLLRightLeft, redLLUpLeft, redLLRollLeft, redLLPitchLeft, redLLYawLeft);
               LimelightHelpers.setCameraPose_RobotSpace("limelight-right", redLLForwardRight, redLLRightRight, redLLUpRight, redLLRollRight, redLLPitchRight, redLLYawRight);
               LimelightHelpers.setCameraPose_RobotSpace("limelight-shooter", redLLForwardShooter, redLLRightShooter, redLLUpShooter, redLLRollShooter, redLLPitchShooter, redLLYawShooter);
            }
            else {
               LimelightHelpers.setCameraPose_RobotSpace("limelight-left", blueLLForwardLeft, blueLLRightLeft, blueLLUpLeft, blueLLRollLeft, blueLLPitchLeft, blueLLYawLeft);
               LimelightHelpers.setCameraPose_RobotSpace("limelight-right", blueLLForwardRight, blueLLRightRight, blueLLUpRight, blueLLRollRight, blueLLPitchRight, blueLLYawRight);
               LimelightHelpers.setCameraPose_RobotSpace("limelight-shooter", blueLLForwardShooter, blueLLRightShooter, blueLLUpShooter, blueLLRollShooter, blueLLPitchShooter, blueLLYawShooter);
            }
         }
      }

      else{

      }
   }

   public void updateDashboard() {

	}
}
