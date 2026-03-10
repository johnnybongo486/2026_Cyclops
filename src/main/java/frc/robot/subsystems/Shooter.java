package frc.robot.subsystems;

import frc.robot.DeviceIds;
import frc.robot.Robot;
import frc.lib.models.*;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityDutyCycle;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Shooter extends SubsystemBase implements IVelocityControlledSubsystem {

	private boolean isHoldingVelocity = false;

	// Set Different Speeds
	private double conversionFactor = 4096 / 600;
	private double zeroVelocity = 0*conversionFactor;
	private double maxVelocity = 5000*conversionFactor;

	public final static int Shooter_PIDX = 0;

	public double maxVelocityLimit = maxVelocity;
	public double lowVelocityLimit = 0;
	private VelocityDutyCycle targetVelocityDutyCycle = new VelocityDutyCycle(0);
    public double targetVelocity = 0;
	private double arbitraryFeedForward = 0.0;

	private final static double onTargetThreshold = 0.05;

	public TalonFX shooterKraken = new TalonFX(DeviceIds.Shooter.LeadMotorId, "canivore");
    public TalonFX shooterKrakenFollower = new TalonFX(DeviceIds.Shooter.FollowerMotorId, "canivore");
	public TalonFXConfiguration shooterFXConfig = new TalonFXConfiguration();

	public Shooter() {
        // Clear Sticky Faults
		this.shooterKraken.clearStickyFaults();
		this.shooterKrakenFollower.clearStickyFaults();
		
        // Set Followers
		this.shooterKrakenFollower.setControl(new Follower(DeviceIds.Shooter.LeadMotorId, MotorAlignmentValue.Opposed));
		
		/** Shooter Motor Configuration */
        /* Motor Inverts and Neutral Mode */
		shooterFXConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        shooterFXConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;

		// Current Limiting
		shooterFXConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        shooterFXConfig.CurrentLimits.StatorCurrentLimit = 40;

        /* PID Config */
        shooterFXConfig.Slot0.kP = 0.15; //0.38
        shooterFXConfig.Slot0.kI = 0.01; // 0.05
        shooterFXConfig.Slot0.kD = 0.15; // 0.18

        /* Open and Closed Loop Ramping */
        shooterFXConfig.OpenLoopRamps.DutyCycleOpenLoopRampPeriod = 0.5;
        shooterFXConfig.OpenLoopRamps.VoltageOpenLoopRampPeriod = 0.5;

        shooterFXConfig.ClosedLoopRamps.DutyCycleClosedLoopRampPeriod = 0.5;
        shooterFXConfig.ClosedLoopRamps.VoltageClosedLoopRampPeriod = 0.5;

		// Velocity Feed Forward
		shooterFXConfig.Slot0.kV = .0113;
		
		// Enable FOC
		targetVelocityDutyCycle.withEnableFOC(true);
		
		// Set Peak Torque Current
		shooterFXConfig.TorqueCurrent.PeakForwardTorqueCurrent = 40;
		shooterFXConfig.TorqueCurrent.PeakReverseTorqueCurrent = 40;

        // Config Motor
        shooterKraken.getConfigurator().apply(shooterFXConfig);
        shooterKraken.getConfigurator().setPosition(0.0);
		shooterKrakenFollower.getConfigurator().setPosition(0);
	}

	public void velocityControl() {
		targetVelocityDutyCycle.withVelocity(targetVelocity);
		this.shooterKraken.setControl(targetVelocityDutyCycle);
	}

	public double getCurrentDraw() {
		return this.shooterKraken.getSupplyCurrent().getValueAsDouble();
	}

	public boolean isHoldingVelocity() {
		return this.isHoldingVelocity;
	}

	public void setIsHoldingVelocity(boolean isHoldingVelocity) {
		this.isHoldingVelocity = isHoldingVelocity;
	}

	public double getTargetVelocity() {
		return this.targetVelocity;
	}

	public boolean setTargetVelocity(double Velocity) {
		if (!isValidVelocity(Velocity)) {
			return false;
		} else {
			this.targetVelocity = Velocity;
			return true;
		}
	}

	public void forceSetTargetVelocity(double Velocity) {
		this.targetVelocity = Velocity;
	}

	public void incrementTargetVelocity(double increment) {
		double currentTargetVelocity = this.targetVelocity;
		double newTargetVelocity = currentTargetVelocity + increment;
		if (isValidVelocity(newTargetVelocity)) {
			this.targetVelocity = newTargetVelocity;
		}
	}

	public boolean isValidVelocity(double Velocity) {
		boolean withinBounds = Velocity <= maxVelocityLimit && Velocity >= lowVelocityLimit;
		return withinBounds;
	}

    // communicate with commands
	public double getZeroVelocity() {
		return this.zeroVelocity;
	}

	public double getMaxVelocity() {
		return this.maxVelocity;
	}

	public double getArbitraryFeedForward() {
		return this.arbitraryFeedForward;
	}

	public void resetShooterEncoder() {
        try {
			shooterKraken.getConfigurator().setPosition(0.0);
			shooterKrakenFollower.getConfigurator().setPosition(0);

        }
        catch (Exception e) {
            DriverStation.reportError("Shooter.resetShooterEncoders exception.  You're Screwed! : " + e.toString(), false);
        }
	}

	public double getVelocityError() {
		double currentVelocity = this.getCurrentVelocity();
		double targetVelocity = this.getTargetVelocity();
		double VelocityError = Math.abs(currentVelocity - targetVelocity);
		return VelocityError;
	}

	public double joystickShooter(){
		double value = 0;
		value = -Robot.robotContainer.getOperatorRightStickY();
		return value;
	}

	public void updateDashboard() {
		SmartDashboard.putNumber("Shooter Velocity", this.getCurrentVelocity());
		SmartDashboard.putNumber("Shooter Target Velocity", this.getTargetVelocity());
		SmartDashboard.putNumber("Shooter Velocity Error", this.getVelocityError());
		SmartDashboard.putNumber("Shooter Current", this.getCurrentDraw());
	}

	@Override
	public double getCurrentVelocity() {
		double currentVelocity = this.shooterKraken.getVelocity().getValueAsDouble();
		return currentVelocity;
	}

	@Override
	public boolean isAtVelocity(double targetVelocity) {
		double currentVelocity = this.getCurrentVelocity();
		double VelocityError = Math.abs(currentVelocity - targetVelocity);
		if (VelocityError < onTargetThreshold) {
			return true;
		} else {
			return false;
		}
	}
}
