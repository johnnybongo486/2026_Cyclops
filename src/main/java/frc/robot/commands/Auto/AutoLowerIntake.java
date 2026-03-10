package frc.robot.commands.Auto;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.commands.Intake.SetIntakeWristPosition;

public class AutoLowerIntake extends SequentialCommandGroup {   
    
    public AutoLowerIntake() {
        addCommands(
            new SetIntakeWristPosition(Constants.Intake.IntakeWrist.RunIntakePosition));
    }
}
