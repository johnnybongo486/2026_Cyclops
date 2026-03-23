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

public class Uptake extends SubsystemBase {

	private TalonFX uptakeKraken = new TalonFX(DeviceIds.Serializer.UptakeMotorId, "canivore");
    private TalonFXConfiguration uptakeFXConfig = new TalonFXConfiguration();
    private TorqueCurrentFOC torqueDutyCycle = new TorqueCurrentFOC(0);

	public Uptake() {
        /** Shooter Motor Configuration */
        /* Motor Inverts and Neutral Mode */
		uptakeFXConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        uptakeFXConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;

        /* Current Limiting */
        uptakeFXConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        uptakeFXConfig.CurrentLimits.StatorCurrentLimit = 60;

        /* PID Config */
        uptakeFXConfig.Slot0.kP = 1;
        uptakeFXConfig.Slot0.kI = 0;
        uptakeFXConfig.Slot0.kD = 0;

        /* Open and Closed Loop Ramping */
        uptakeFXConfig.OpenLoopRamps.DutyCycleOpenLoopRampPeriod = 0.1;
        uptakeFXConfig.OpenLoopRamps.VoltageOpenLoopRampPeriod = 0.1;

        uptakeFXConfig.ClosedLoopRamps.DutyCycleClosedLoopRampPeriod = 0.1;
        uptakeFXConfig.ClosedLoopRamps.VoltageClosedLoopRampPeriod = 0.1;

        // Config Motor
        uptakeKraken.getConfigurator().apply(uptakeFXConfig);
        uptakeKraken.getConfigurator().setPosition(0.0);
	}

	public void setSpeed(double speed) {
        torqueDutyCycle.withOutput(60).withDeadband(1).withMaxAbsDutyCycle(speed);
        this.uptakeKraken.setControl(torqueDutyCycle);
	}

	public double getCurrentDrawLeader() {
		return this.uptakeKraken.getSupplyCurrent().getValueAsDouble();
	}

	public void resetUptakeEncoder() {
        try {
			uptakeKraken.getConfigurator().setPosition(0.0);
        }
        catch (Exception e) {
            DriverStation.reportError("Uptake.resetUptakeEncoders exception.  You're Screwed! : " + e.toString(), false);
        }
	}

	public void updateDashboard() {
		SmartDashboard.putNumber("Uptake Current", this.getCurrentDrawLeader());

	}
}
