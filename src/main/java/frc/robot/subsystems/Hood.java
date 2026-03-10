package frc.robot.subsystems;

import frc.lib.models.*;
import frc.robot.Constants;
import frc.robot.DeviceIds;
import frc.robot.Robot;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicDutyCycle;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Hood extends SubsystemBase implements IPositionControlledSubsystem {

	private boolean isHoldingPosition = false;

    // Set Different Heights
	private double homePosition = Constants.Shooter.Hood.StoreHoodPosition;
	private double maxUpTravelPosition = Constants.Shooter.Hood.FullUpPosition;

	public double upPositionLimit = maxUpTravelPosition;
	public double downPositionLimit = homePosition;
	private double targetPosition = 0;
    private MotionMagicDutyCycle targetPositionDutyCycle = new MotionMagicDutyCycle(0);
	private double feedForward = 0.0;
	public double shooterAddValue;

	private final static double onTargetThreshold = 0.1;
		
	private TalonFX hoodKraken = new TalonFX(DeviceIds.Shooter.HoodMotorId, "canivore");

    private TalonFXConfiguration hoodFXConfig = new TalonFXConfiguration();

	public Hood() {
		// Clear Sticky Faults
		this.hoodKraken.clearStickyFaults();
		
        /** Shooter Motor Configuration */
        /* Motor Inverts and Neutral Mode */
		hoodFXConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        hoodFXConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        /* Current Limiting */
		hoodFXConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        hoodFXConfig.CurrentLimits.StatorCurrentLimit = 20;

        /* PID Config */
        hoodFXConfig.Slot0.kP = 0.2;
        hoodFXConfig.Slot0.kI = 0;
        hoodFXConfig.Slot0.kD = 0.01;

        /* Open and Closed Loop Ramping */
        hoodFXConfig.OpenLoopRamps.DutyCycleOpenLoopRampPeriod = 0.25;
        hoodFXConfig.OpenLoopRamps.VoltageOpenLoopRampPeriod = 0.25;

        hoodFXConfig.ClosedLoopRamps.DutyCycleClosedLoopRampPeriod = 0;
        hoodFXConfig.ClosedLoopRamps.VoltageClosedLoopRampPeriod = 0;

        //Config Acceleration and Velocity
        hoodFXConfig.MotionMagic.withMotionMagicAcceleration(300);
        hoodFXConfig.MotionMagic.withMotionMagicCruiseVelocity(300);

        // Config Motor
        hoodKraken.getConfigurator().apply(hoodFXConfig);
        hoodKraken.getConfigurator().setPosition(0.0);
		
		// Enable FOC
		targetPositionDutyCycle.withEnableFOC(true);

		resethoodEncoder();
	}

	public void positionControl() {
		this.manageMotion(targetPosition);
        targetPositionDutyCycle.withPosition(targetPosition);
        targetPositionDutyCycle.withFeedForward(feedForward);
		this.hoodKraken.setControl(targetPositionDutyCycle);
	}

	public double getCurrentPosition() {
		return this.hoodKraken.getRotorPosition().getValueAsDouble();
	}

	public double getCurrentDraw() {
		return this.hoodKraken.getSupplyCurrent().getValueAsDouble();
	}

	public boolean isHoldingPosition() {
		return this.isHoldingPosition;
	}

	public void setIsHoldingPosition(boolean isHoldingPosition) {
		this.isHoldingPosition = isHoldingPosition;
	}

	public double getTargetPosition() {
		return this.targetPosition;
	}

	public boolean setTargetPosition(double position) {
		if (!isValidPosition(position)) {
			return false;
		} else {
			this.targetPosition = position;
			return true;
		}
	}

	public void forceSetTargetPosition(double position) {
		this.targetPosition = position;
	}

	public void incrementTargetPosition(double increment) {
		double currentTargetPosition = this.targetPosition;
		double newTargetPosition = currentTargetPosition + increment;
		if (isValidPosition(newTargetPosition)) {		// && ishoodSafe(newTargetPosition) check for other subsystems
			this.targetPosition = newTargetPosition;
		}
	}

	public boolean isValidPosition(double position) {
		boolean withinBounds = position <= upPositionLimit && position >= downPositionLimit;
		return withinBounds;
	}

    // communicate with commands
	public double getHomePosition() {
		return this.homePosition;
	}

	public double getMaxUpTravelPosition() {
		return this.maxUpTravelPosition;
	}

	public double getFeedForward() {
		return this.feedForward;
	}

	public void resethoodEncoder() {
        try {
			hoodKraken.getConfigurator().setPosition(0.0);
        }
        catch (Exception e) {
            DriverStation.reportError("Hood.resetHoodEncoders exception.  You're Screwed! : " + e.toString(), false);
        }
	}

	public double joystickHood(){
		double value = 0;
		value = -Robot.robotContainer.getOperatorLeftStickY();
		return value;
	}

	public double getPositionError() {
		double currentPosition = this.getCurrentPosition();
		double targetPosition = this.getTargetPosition();
		double positionError = Math.abs(currentPosition - targetPosition);
		return positionError;
	}

	public void manageMotion(double targetPosition) {
		double currentPosition = getCurrentPosition();
		if (currentPosition < targetPosition) {
				// set based on gravity
		}
		else {
				//set based on gravity
		}
	}

	public void zeroTarget() {
		targetPosition = 0;
	}

	public void updateDashboard() {
		SmartDashboard.putNumber("Hood Position", this.getCurrentPosition());
		SmartDashboard.putNumber("Hood Target Position", this.getTargetPosition());
		SmartDashboard.putNumber("Hood Position Error", this.getPositionError());
		SmartDashboard.putNumber("Hood Velocity", this.getCurrentVelocity());
		SmartDashboard.putNumber("Hood Current", this.getCurrentDraw());
	}

	@Override
	public double getCurrentVelocity() {
		double currentVelocity = this.hoodKraken.getVelocity().getValueAsDouble();
		return currentVelocity;
	}

	@Override
	public boolean isInPosition(double targetPosition) {
		double currentPosition = this.getCurrentPosition();
		double positionError = Math.abs(currentPosition - targetPosition);
		if (positionError < onTargetThreshold) {
			return true;
		} else {
			return false;
		}
	}
}   
