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

public class Drum extends SubsystemBase {

	private TalonFX drumKraken = new TalonFX(DeviceIds.Serializer.DrumMotorId, "canivore");  // rio on pbot
    private TalonFXConfiguration drumFXConfig = new TalonFXConfiguration();

    private TorqueCurrentFOC torqueDutyCycle = new TorqueCurrentFOC(0);


	public Drum() {
        /** Shooter Motor Configuration */
        /* Motor Inverts and Neutral Mode */
		drumFXConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        drumFXConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;

        /* Current Limiting */
        drumFXConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        drumFXConfig.CurrentLimits.StatorCurrentLimit = 40;

        /* PID Config */
        drumFXConfig.Slot0.kP = 0.2;
        drumFXConfig.Slot0.kI = 0;
        drumFXConfig.Slot0.kD = 0;

        /* Open and Closed Loop Ramping */
        drumFXConfig.OpenLoopRamps.DutyCycleOpenLoopRampPeriod = 0.1;
        drumFXConfig.OpenLoopRamps.VoltageOpenLoopRampPeriod = 0.1;

        drumFXConfig.ClosedLoopRamps.DutyCycleClosedLoopRampPeriod = 0.1;
        drumFXConfig.ClosedLoopRamps.VoltageClosedLoopRampPeriod = 0.1;

        // Config Motor
        drumKraken.getConfigurator().apply(drumFXConfig);
        drumKraken.getConfigurator().setPosition(0.0);

	}

	public void setSpeed(double speed) {
        //this.drumKraken.set(speed);
        torqueDutyCycle.withOutput(40).withDeadband(1).withMaxAbsDutyCycle(speed);
        this.drumKraken.setControl(torqueDutyCycle);
	}

	public double getCurrentDrawLeader() {
		return this.drumKraken.getSupplyCurrent().getValueAsDouble();
	}

	public void resetDrumEncoder() {
        try {
			drumKraken.getConfigurator().setPosition(0.0);
        }
        catch (Exception e) {
            DriverStation.reportError("Drum.resetDrumEncoders exception.  You're Screwed! : " + e.toString(), false);
        }
	}

	public void updateDashboard() {
		SmartDashboard.putNumber("Drum Current", this.getCurrentDrawLeader());

	}
}
