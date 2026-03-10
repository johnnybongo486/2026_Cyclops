package frc.robot.dashboard;
import edu.wpi.first.networktables.DoubleEntry;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.RobotContainer;
import frc.robot.config.AutoAimConfig;

public class AutoAimDashboard {
    
    private static DoubleEntry StrafeTargetEntry;
    private static DoubleEntry StrafeToleranceEntry;
    private static DoubleEntry AngleTargetEntry;
    private static DoubleEntry AngleToleranceEntry;
    private static DoubleEntry DistanceTargetEntry;
    private static DoubleEntry DistanceToleranceEntry;

    private static NetworkTableInstance nti = NetworkTableInstance.getDefault();
    private static NetworkTable aslTable = nti.getTable("Auto Shooter");

    public static void AddDashboard() {

        //STRAFE CONFIG
        SmartDashboard.putData("AutoAim Strafe PID",RobotContainer.shooterLimelight.strafeController);
        StrafeTargetEntry = addEntryWithValue("Strafe Target", AutoAimConfig.StrafeTarget);
        StrafeToleranceEntry = addEntryWithValue("Strafe Tolerance", AutoAimConfig.StrafeTolerance);

        //ANGLE CONFIG
        SmartDashboard.putData("AutoAim Angle PID", RobotContainer.shooterLimelight.angleController);
        AngleTargetEntry = addEntryWithValue("Angle Target",AutoAimConfig.AngleTarget);
        AngleToleranceEntry = addEntryWithValue("Angle Tolerance", AutoAimConfig.AngleTolerance);

        //DISTANCE CONFIG
        SmartDashboard.putData("AutoAim Distance PID",RobotContainer.shooterLimelight.distanceController);
        DistanceTargetEntry = addEntryWithValue("Distance Target", AutoAimConfig.DistanceTarget);
        DistanceToleranceEntry = addEntryWithValue("Distance Tolerance", AutoAimConfig.DistanceTolerance);
    }

    public static void syncDashboard() {
        AutoAimConfig.StrafeTarget = StrafeTargetEntry.get(AutoAimConfig.StrafeTarget);
        AutoAimConfig.StrafeTolerance = StrafeToleranceEntry.get(AutoAimConfig.StrafeTolerance);
        AutoAimConfig.AngleTarget = AngleTargetEntry.get(AutoAimConfig.AngleTarget);
        AutoAimConfig.AngleTolerance = AngleToleranceEntry.get(AutoAimConfig.AngleTolerance);
        AutoAimConfig.DistanceTarget = DistanceTargetEntry.get(AutoAimConfig.DistanceTarget);
        AutoAimConfig.DistanceTolerance = DistanceToleranceEntry.get(AutoAimConfig.DistanceTolerance);
        return;

    }


    private static DoubleEntry addEntryWithValue(String name, double defaultValue) {
        DoubleEntry newEntry = aslTable.getDoubleTopic(name).getEntry(defaultValue);
        newEntry.set(defaultValue);
        return newEntry;
    }

    
}