package frc.robot.commands.Auto;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.commands.Intake.SetIntakeWristPosition;

public class AutoRaiseIntake extends SequentialCommandGroup {   
    
    public AutoRaiseIntake() {
        addCommands(
            new SetIntakeWristPosition(Constants.Intake.IntakeWrist.StoreIntakePosition));
    }
}
