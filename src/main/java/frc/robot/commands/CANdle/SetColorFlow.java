package frc.robot.commands.CANdle;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;

public class SetColorFlow extends Command {
    
    public SetColorFlow() {
        addRequirements(RobotContainer.caNdleSubsystem);
    }

    public void initialize() {
        RobotContainer.caNdleSubsystem.setAnimate("ColorFlow");
    }

    public void execute() {
        RobotContainer.caNdleSubsystem.setAnimate("ColorFlow");
        
    }

    public boolean isFinished() {
        return false;
    }

    protected void end() {
        
    }

    protected void interrupted() {
        end();
    }
}