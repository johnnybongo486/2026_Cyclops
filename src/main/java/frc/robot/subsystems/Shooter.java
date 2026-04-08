package frc.robot.subsystems;

import frc.robot.DeviceIds;
import frc.robot.Robot;
import frc.lib.models.*;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.MatchLog;

public class Shooter extends SubsystemBase implements IVelocityControlledSubsystem {

	private boolean isHoldingVelocity = false;

	// Set Different Speeds
	//private double conversionFactor = 4096 / 600;
	private double zeroVelocity = 0;
	private double maxVelocity = 55;

	public final static int Shooter_PIDX = 0;

	public double maxVelocityLimit = maxVelocity;
	public double lowVelocityLimit = 0;
	private VelocityVoltage targetVelocityDutyCycle = new VelocityVoltage(0);

    public double targetVelocity = 0;
	private double arbitraryFeedForward = 0.0;

	private final static double onTargetThreshold = 0.01;

	public double shooterAdder = 0;

	public TalonFX leftShooterKraken = new TalonFX(DeviceIds.Shooter.LeadLeftMotorId, "canivore");
    public TalonFX leftShooterKrakenFollower = new TalonFX(DeviceIds.Shooter.FollowerLeftMotorId, "canivore");
	public TalonFX rightShooterKraken = new TalonFX(DeviceIds.Shooter.LeadRightMotorId, "canivore");
    public TalonFX rightShooterKrakenFollower = new TalonFX(DeviceIds.Shooter.FollowerRightMotorId, "canivore");
	public TalonFXConfiguration shooterFXConfig = new TalonFXConfiguration();

	public Shooter() {
        // Clear Sticky Faults
		this.leftShooterKraken.clearStickyFaults();
		this.leftShooterKrakenFollower.clearStickyFaults();
		this.rightShooterKraken.clearStickyFaults();
		this.rightShooterKrakenFollower.clearStickyFaults();
		
        // Set Followers
		this.leftShooterKrakenFollower.setControl(new Follower(DeviceIds.Shooter.LeadLeftMotorId, MotorAlignmentValue.Aligned));
		this.rightShooterKraken.setControl(new Follower(DeviceIds.Shooter.LeadLeftMotorId, MotorAlignmentValue.Opposed));
		this.rightShooterKrakenFollower.setControl(new Follower(DeviceIds.Shooter.LeadLeftMotorId, MotorAlignmentValue.Opposed));
		
		/** Shooter Motor Configuration */
        /* Motor Inverts and Neutral Mode */
		shooterFXConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        shooterFXConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;

		// Current Limiting
		shooterFXConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        shooterFXConfig.CurrentLimits.StatorCurrentLimit = 40;

        /* PID Config */
        shooterFXConfig.Slot0.kP = 0.8; // 0.7
        shooterFXConfig.Slot0.kI = 0.0; // 0.0
        shooterFXConfig.Slot0.kD = 0.0; // 0.5

        /* Open and Closed Loop Ramping */
        shooterFXConfig.OpenLoopRamps.DutyCycleOpenLoopRampPeriod = 0.5;
        shooterFXConfig.OpenLoopRamps.VoltageOpenLoopRampPeriod = 0.5;

        shooterFXConfig.ClosedLoopRamps.DutyCycleClosedLoopRampPeriod = 0.225;
        shooterFXConfig.ClosedLoopRamps.VoltageClosedLoopRampPeriod = 0.225;

		// Velocity Feed Forward
		shooterFXConfig.Slot0.kV = 0.125*0.95; // 0.01062
		shooterFXConfig.Slot0.kA = 0.0;		// 0.0
		shooterFXConfig.Slot0.kS = 0.25;	// 0.0018;
		
		// Enable FOC
		targetVelocityDutyCycle.withEnableFOC(true);
		
		// Set Peak Torque Current
		shooterFXConfig.TorqueCurrent.PeakForwardTorqueCurrent = 40;
		shooterFXConfig.TorqueCurrent.PeakReverseTorqueCurrent = 40;

        // Config Motor
        leftShooterKraken.getConfigurator().apply(shooterFXConfig);
        leftShooterKraken.getConfigurator().setPosition(0.0);
		leftShooterKrakenFollower.getConfigurator().setPosition(0);
		rightShooterKraken.getConfigurator().setPosition(0.0);
		rightShooterKrakenFollower.getConfigurator().setPosition(0);
	}

	public void velocityControl() {
		targetVelocityDutyCycle.withVelocity(targetVelocity);
		this.leftShooterKraken.setControl(targetVelocityDutyCycle);
	}

	public double getCurrentDraw() {
		return this.leftShooterKraken.getSupplyCurrent().getValueAsDouble();
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
			if (Velocity != this.targetVelocity) {
				MatchLog.event("shooter/targetVelocity",
					String.format("%.2f -> %.2f rot/s", this.targetVelocity, Velocity));
			}
			this.targetVelocity = Velocity;
			return true;
		}
	}

	public void forceSetTargetVelocity(double Velocity) {
		if (Velocity != this.targetVelocity) {
			MatchLog.event("shooter/targetVelocity",
				String.format("%.2f -> %.2f rot/s (forced)", this.targetVelocity, Velocity));
		}
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
			leftShooterKraken.getConfigurator().setPosition(0.0);
			leftShooterKrakenFollower.getConfigurator().setPosition(0);
			rightShooterKraken.getConfigurator().setPosition(0.0);
			rightShooterKrakenFollower.getConfigurator().setPosition(0);
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
		double currentVelocity = this.leftShooterKraken.getVelocity().getValueAsDouble();
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

	public void setShooterAdder(double adder) {
		this.shooterAdder = adder;
	}

	public double getShooterAdder(){
		return shooterAdder;
	}
}
