package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.DeviceIds;

public class Agitator extends SubsystemBase {

	private TalonFX agitatorKraken = new TalonFX(DeviceIds.Serializer.AgitatorMotorId, "canivore");
    private TalonFXConfiguration agitatorFXConfig = new TalonFXConfiguration();
    private TorqueCurrentFOC torqueDutyCycle = new TorqueCurrentFOC(0);

	public Agitator() {
        /** Shooter Motor Configuration */
        /* Motor Inverts and Neutral Mode */
		agitatorFXConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        agitatorFXConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        /* Current Limiting */
        agitatorFXConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        agitatorFXConfig.CurrentLimits.StatorCurrentLimit = 20;

        /* PID Config */
        agitatorFXConfig.Slot0.kP = 0.2;
        agitatorFXConfig.Slot0.kI = 0;
        agitatorFXConfig.Slot0.kD = 0;

        /* Open and Closed Loop Ramping */
        agitatorFXConfig.OpenLoopRamps.DutyCycleOpenLoopRampPeriod = 0.1;
        agitatorFXConfig.OpenLoopRamps.VoltageOpenLoopRampPeriod = 0.1;

        agitatorFXConfig.ClosedLoopRamps.DutyCycleClosedLoopRampPeriod = 0.1;
        agitatorFXConfig.ClosedLoopRamps.VoltageClosedLoopRampPeriod = 0.1;

        // Config Motor
        agitatorKraken.getConfigurator().apply(agitatorFXConfig);
        agitatorKraken.getConfigurator().setPosition(0.0);

	}

	public void setSpeed(double speed) {
        //this.agitatorKraken.set(speed);
        torqueDutyCycle.withOutput(30).withDeadband(1).withMaxAbsDutyCycle(speed);
        this.agitatorKraken.setControl(torqueDutyCycle);
	}

	public double getCurrentDrawLeader() {
		return this.agitatorKraken.getSupplyCurrent().getValueAsDouble();
	}

	public void resetAgitatorEncoder() {
        try {
			agitatorKraken.getConfigurator().setPosition(0.0);
        }
        catch (Exception e) {
            DriverStation.reportError("Agitator.resetAgitatorEncoders exception.  You're Screwed! : " + e.toString(), false);
        }
	}

	public void updateDashboard() {
		SmartDashboard.putNumber("Agitator Current", this.getCurrentDrawLeader());

	}
}
