package frc.robot.subsystems;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.DeviceIds;
import frc.robot.config.AutoAimConfig;

public class ShooterLimelight extends SubsystemBase {

   private double ta;
   private double tx;
   private double ty;
   private double[] tAng;
   private int tv;

   NetworkTableEntry prelimtx;
   NetworkTableEntry prelimty;
   NetworkTableEntry prelimta;
   NetworkTableEntry prelimtl;
   NetworkTableEntry prelimts;
   NetworkTableEntry prelimtAng;
   NetworkTableEntry prelimtv;
   NetworkTableEntry prelimCamtran;
   NetworkTable table;
   NetworkTableInstance Inst;

   public final PIDController angleController = new PIDController(AutoAimConfig.AnglePID.P, AutoAimConfig.AnglePID.I,AutoAimConfig.AnglePID.D);
   public final PIDController strafeController = new PIDController(AutoAimConfig.StrafePID.P,AutoAimConfig.StrafePID.I,AutoAimConfig.StrafePID.D);
   public final PIDController distanceController = new PIDController(AutoAimConfig.DistancePID.P, AutoAimConfig.DistancePID.I,AutoAimConfig.DistancePID.D);

   public ShooterLimelight() {
      Inst = NetworkTableInstance.getDefault();
      table = Inst.getTable(DeviceIds.Limelight.ShooterTableName);
      prelimta = table.getEntry("ta");
      prelimtx = table.getEntry("tx");
      prelimty = table.getEntry("ty");
      prelimtAng = table.getEntry("targetpose_robotspace");
      prelimtv = table.getEntry("tv");

      angleController.setTolerance(AutoAimConfig.AngleTolerance);  // needs to be tuned
      strafeController.setTolerance(AutoAimConfig.StrafeTolerance);
      distanceController.setTolerance(AutoAimConfig.DistanceTolerance);
   }

   public void updateGameState(){
      ta = prelimta.getDouble(ta);
      tx = prelimtx.getDouble(0);
      ty = prelimty.getDouble(ty);
      tv = (int) prelimtv.getInteger(tv);
      tAng = prelimtAng.getDoubleArray(new double[6]);

   }

   public double getArea(){
      ta = prelimta.getDouble(ta);
      return ta;
   }

   public double getX(){
      tx = prelimtx.getDouble(tx);
      return tx;
   }

   public double getY(){
      ty = prelimty.getDouble(ty);
      return ty;
   }

   public boolean ifValidTag() {
      tv = (int) prelimtv.getInteger(tv);
      if (tv == 1) {
         return true;   
      }
      else {
         return false;
      }
      
   }

   public double gettPitch() {
      tAng = prelimtAng.getDoubleArray(new double[6]);
      double actAng = tAng[3];
      return actAng;
   }

   public double gettYaw() {
      tAng = prelimtAng.getDoubleArray(new double[6]);
      double actAng = tAng[4];
      return actAng;
   }

   public double gettRoll() {
      tAng = prelimtAng.getDoubleArray(new double[6]);
      double actAng = tAng[5];
      return actAng;
   }

   public double gettx() {
      tAng = prelimtAng.getDoubleArray(new double[6]);
      double actX = tAng[0];
      return actX;
   }

   public double getty() {
      tAng = prelimtAng.getDoubleArray(new double[6]);
      double actY = tAng[1];
      return actY;
   }

   public double gettz() {
      tAng = prelimtAng.getDoubleArray(new double[6]);
      double actZ = tAng[2];
      return actZ;
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
	   SmartDashboard.putNumber("Shooter ta", getArea());
      SmartDashboard.putNumber("Shooter tx", getX());
      SmartDashboard.putNumber("Shooter ty", getY());
      SmartDashboard.putNumber("Shooter tPitch", gettPitch());
      SmartDashboard.putNumber("Shooter tYaw", gettYaw());      
      SmartDashboard.putNumber("Shooter tRoll", gettRoll());
      SmartDashboard.putNumber("Shooter tX", gettx());
      SmartDashboard.putNumber("Shooter tY", getty());
      SmartDashboard.putNumber("Shooter tZ", gettz());
      SmartDashboard.putBoolean("Shooter tv", ifValidTag());
	}
}
