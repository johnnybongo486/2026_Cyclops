package frc.robot.config;

public class AutoAimConfig {

    //Configurations for DISTANCE component
    public static class DistancePID{
        public static double P = 0.4;
        public static double I = 0.0;
        public static double D = 0.0;
    }

    public static double DistanceTarget = 0.40;  
    public static double DistanceTolerance = 0.02;  

    //Configurations for STRAFE component
    public static class StrafePID{
        public static double P = 0.5;
        public static double I = 0.0;
        public static double D = 0.0;
    }
    public static double StrafeTarget = 0.17; 
    public static double StrafeTolerance = 0.02;  

    //Configurations for ANGLE component
    public static class AnglePID{
        public static double P = 0.1;
        public static double I = 0.0;
        public static double D = 0.0;
    }

    public static double AngleTarget = 0;
    public static double AngleTolerance = 0.5;

}