package frc.robot;

public final class DeviceIds {
    public static final class Serializer {
        public static final int UptakeMotorId = 20;
        public static final int DrumMotorId = 21;
        public static final int AgitatorMotorId = 22;

    }

    public static final class Intake {
        public static final int WristMotorId = 19;
        public static final int IntakeMotorId = 18;
        public static final int FollowerMotorId = 23;
    }

    public static final class Shooter {
        public static final int LeadMotorId = 13;
        public static final int FollowerMotorId = 14;
        public static final int HoodMotorId = 15;
        public static final int TurretMotorId = 16;
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
