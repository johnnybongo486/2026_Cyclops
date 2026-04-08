package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.DeviceIds;

public class Intake extends SubsystemBase {

	private TalonFX intakeKraken = new TalonFX(DeviceIds.Intake.IntakeMotorId, "canivore");
    private TalonFXConfiguration intakeFXConfig = new TalonFXConfiguration();
    private TalonFX intakeKrakenFollower = new TalonFX(DeviceIds.Intake.FollowerMotorId, "canivore");
    private TorqueCurrentFOC torqueDutyCycle = new TorqueCurrentFOC(0);


	public Intake() {
        // Set Follower
        this.intakeKrakenFollower.setControl(new Follower(DeviceIds.Intake.IntakeMotorId, MotorAlignmentValue.Opposed));

        /** Shooter Motor Configuration */
        /* Motor Inverts and Neutral Mode */
		intakeFXConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        intakeFXConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;

        /* Current Limiting */
        intakeFXConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        intakeFXConfig.CurrentLimits.StatorCurrentLimit = 40;

        /* PID Config */
        intakeFXConfig.Slot0.kP = 0.2;
        intakeFXConfig.Slot0.kI = 0;
        intakeFXConfig.Slot0.kD = 0;

        /* Open and Closed Loop Ramping */
        intakeFXConfig.OpenLoopRamps.DutyCycleOpenLoopRampPeriod = 0.25;
        intakeFXConfig.OpenLoopRamps.VoltageOpenLoopRampPeriod = 0.25;

        intakeFXConfig.ClosedLoopRamps.DutyCycleClosedLoopRampPeriod = 0.25;
        intakeFXConfig.ClosedLoopRamps.VoltageClosedLoopRampPeriod = 0.25;

        // Config Motor
        intakeKraken.getConfigurator().apply(intakeFXConfig);
        intakeKraken.getConfigurator().setPosition(0.0);
        intakeKrakenFollower.getConfigurator().setPosition(0);
	}

	public void setSpeed(double speed) {
        //this.intakeKraken.set(speed);

        if (speed > 0) {
            torqueDutyCycle.withOutput(40).withDeadband(1).withMaxAbsDutyCycle(speed);
        }
        else {
            torqueDutyCycle.withOutput(-40).withDeadband(1).withMaxAbsDutyCycle(-speed);
        }
        
        this.intakeKraken.setControl(torqueDutyCycle);
	}

	public double getCurrentDrawLeader() {
		return this.intakeKraken.getSupplyCurrent().getValueAsDouble();
	}

	public void resetIntakeEncoder() {
        try {
			intakeKraken.getConfigurator().setPosition(0.0);
        }
        catch (Exception e) {
            DriverStation.reportError("Intake.resetIntakeEncoders exception.  You're Screwed! : " + e.toString(), false);
        }
	}

	public void updateDashboard() {
		SmartDashboard.putNumber("Intake Current", this.getCurrentDrawLeader());

	}
}
