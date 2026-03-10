package frc.robot.commands.CANdle;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;

public class SetPurple extends Command {
    
    public SetPurple() {
        addRequirements(RobotContainer.caNdleSubsystem);
    }

    public void initialize() {
        RobotContainer.caNdleSubsystem.setAnimate("Purple");
    }

    public void execute() {
        RobotContainer.caNdleSubsystem.setAnimate("Purple");
        
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