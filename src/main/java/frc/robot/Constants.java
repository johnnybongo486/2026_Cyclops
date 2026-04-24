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
      public static final double IntakeRunSpeed = 1.0;  // 1.0
      public static final double IntakeReverseSpeed = -1;
      public static final double IntakeStopSpeed = 0.0;
    }
    public static class IntakeWrist{
      public static final double StoreIntakePosition = 0.0;
      public static final double RunIntakePosition = 6.25;  // todo
      public static final double squeeze = 2;

    }
  }

  public static class Shooter{
    public static class Hood {
      public static final double StoreHoodPosition = 0.0;
      public static final double FullUpPosition = 12;
    }
    public static class HoodShooting {
      public static double a = -0.41557407878887015;
      public static double b = 5.9101087329561057;
      public static double c = -6.8077637829396078;
    }
    public static class HoodPassing {
      public static double a = -0.25882771334772930;
      public static double b = 4.3610212596379370;
      public static double c = -5.8957507741913759;  // -6.29
    }
    public static class ShooterSpeed {
      public static double a = 0.31429744138369126;
      public static double b = 0.99819972029013271;
      public static double c = 37.404054363263093;  // 37.404054363263093
      public static double ShooterAdder = 2.25;
    }
    public static class ShooterPassing {
      public static double a = 0.083255767043861117;
      public static double b = 2.8169346870502308;
      public static double c = 28.691668594903540;  // 27.69
    }
    // Estimated projectile speed in m/s — used to compute time-of-flight for lead angle.
    // Tune by shooting from a known distance at known robot speed and adjusting until shots land.
    public static class ProjectileSpeed {
      public static double metersPerSecond = 2.3;
    }

    // Fixed Position Hood Positions
    public static class FixedShootHood {
      public static double bumpers = 0.49;
      public static double ladder = 8.15;
      public static double side = 5.2;
    }

    // Fixed Position Shooter Speeds
    public static class FixedShootSpeed {
      public static double bumpers = 39.0;
      public static double ladder = 43.5;
      public static double side = 41;
    }
  }

}
