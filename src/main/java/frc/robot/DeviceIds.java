package frc.robot;

public final class DeviceIds {
    public static final class Serializer {
        public static final int UptakeMotorId = 21;
        public static final int AgitatorMotorId = 22;
    }

    public static final class Intake {
        public static final int WristMotorId = 20;
        public static final int IntakeMotorId = 18;
        public static final int FollowerMotorId = 19;
    }

    public static final class Shooter {
        public static final int LeadLeftMotorId = 13;  // Top is the lead
        public static final int FollowerLeftMotorId = 14;
        public static final int LeadRightMotorId = 23;  // Top is the lead
        public static final int FollowerRightMotorId = 24;
        public static final int HoodMotorId = 15;
    }

    public static final class CANdle {
        public static final int CANdleId = 17;
    }

    public static final class Limelight {
        public static final String ShooterTableName = "limelight-shooter";
        public static final String LeftTableName = "limelight-left";
        public static final String RightTableName = "limelight-right";

    }
}
