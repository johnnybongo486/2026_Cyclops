// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0; // 0
    public static final int kOperatorControllerPort = 1;  //1
  }

  public static class Serializer{
    public static class Uptake{
        public static final double UptakeRunSpeed = 0.8;  // 1.0 pbot 0.7 real
        public static final double UptakeStopSpeed = 0.0;
    }
      public static class Drum{
        public static final double DrumRunSpeed = 1.0; // was 0.8 real  // -0.55 pbot
        public static final double DrumStopSpeed = 0.0;
    }
      public static class Agitator{
        public static final double AgitatorRunSpeed = 0.2;  // was .5
        public static final double AgitatorSlowSpeed = 0.1;
        public static final double AgitatorStopSpeed = 0.0;
    }
  }

  public static class Intake{
    public static class IntakeRoller{
      public static final double IntakeRunSpeed = 0.6;  // was 0.7
      public static final double IntakeReverseSpeed = -0.5;
      public static final double IntakeStopSpeed = 0.0;
    }
    public static class IntakeWrist{
      public static double StoreIntakePosition = 0.0;
      public static double RunIntakePosition = 5.15;  // todo
    }
  }

  public static class Shooter{
    public static class Hood {
      public static double StoreHoodPosition = 0.0;
      public static double FullUpPosition = 12.8;  //todo
    }
    public static class HoodShooting {
      public static double a = -0.26400351354472845;
      public static double b = 3.8883426484925661;
      public static double c = -2.8753718603820908;
    }
    public static class HoodPassing {
      public static double a = -0.269210;
      public static double b = 4.12897;
      public static double c = -10.24743;
    }
    public static class Turret {
      public static double MinimumTurretPosition = -21.2;
      public static double MaximumTurretPosition = 12.2;  //todo
    }
    public static class ShooterSpeed {
      public static double a = 0.46691434209640875;
      public static double b = -0.54929686119468091;
      public static double c = 35.651718606278294; //36.151718606278294
    }
    public static class ShooterPassing {
      public static double a = -0.5112327;
      public static double b = 11.38087;
      public static double c = 5; //7.48918  // 2
    }
    // Estimated projectile speed in m/s — used to compute time-of-flight for lead angle.
    // Tune by shooting from a known distance at known robot speed and adjusting until shots land.
    public static class ProjectileSpeed {
      public static double metersPerSecond = 3.0; // 15
    }
  }

}
