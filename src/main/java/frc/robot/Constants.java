package frc.robot;

public final class Constants {
  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0; 
    public static final int kOperatorControllerPort = 1;  
  }

  public static class Serializer{
    public static class Uptake{
        public static final double UptakeRunSpeed = 0.8;  // was 1.0
        public static final double UptakeStopSpeed = 0.0;
    }
      public static class Agitator{
        public static final double AgitatorRunSpeed = 0.8;  // was 1.0
        public static final double AgitatorSlowSpeed = 0.1;
        public static final double AgitatorStopSpeed = 0.0;
    }
  }

  public static class Intake{
    public static class IntakeRoller{
      public static final double IntakeRunSpeed = 0.8;  // 1.0
      public static final double IntakeReverseSpeed = -0.5;
      public static final double IntakeStopSpeed = 0.0;
    }
    public static class IntakeWrist{
      public static double StoreIntakePosition = 0.0;
      public static double RunIntakePosition = 7;  // todo
    }
  }

  public static class Shooter{
    public static class Hood {
      public static double StoreHoodPosition = 0.0;
      public static double FullUpPosition = 12;
    }
    public static class HoodShooting {
      public static double a = -0.6273265054261399;
      public static double b = 5.8552271086199248;
      public static double c = -6.7824905374981768;
    }
    public static class HoodPassing {
      public static double a = -0.25882771334772930;
      public static double b = 4.3610212596379370;
      public static double c = -6.2957507741913759;
    }
    public static class ShooterSpeed {
      public static double a = 0.82423647247415344;
      public static double b = -1.2246146581263617;
      public static double c = 40.475097446874969; 
    }
    public static class ShooterPassing {
      public static double a = 0.083255767043861117;
      public static double b = 2.8169346870502308;
      public static double c = 27.691668594903540;
    }
    // Estimated projectile speed in m/s — used to compute time-of-flight for lead angle.
    // Tune by shooting from a known distance at known robot speed and adjusting until shots land.
    public static class ProjectileSpeed {
      public static double metersPerSecond = 2.5;
    }

    // Fixed Position Hood Positions
    public static class FixedShootHood {
      public static double bumpers = 0.0;
      public static double ladder = 5.06;
      public static double side = 5.06;
    }

    // Fixed Position Shooter Speeds
    public static class FixedShootSpeed {
      public static double bumpers = 38.0;
      public static double ladder = 43.5;
      public static double side = 46;
    }
  }

}
