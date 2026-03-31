package frc.robot.commands.Swerve;

import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import static edu.wpi.first.units.Units.*;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;

public class TeleopDrive extends Command {
	// Computed once at class load — these are constants; recomputing each loop created garbage
	private static final double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);
	private static final double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond);

	private CommandSwerveDrivetrain drivetrain;
	private SwerveRequest.FieldCentric drive;
	private DoubleSupplier translationSup;
    private DoubleSupplier strafeSup;
    private DoubleSupplier rotationSup;

	public TeleopDrive(CommandSwerveDrivetrain drivetrain, SwerveRequest.FieldCentric drive, DoubleSupplier translationSup, DoubleSupplier strafeSup, DoubleSupplier rotationSup) {
		this.drivetrain = drivetrain;
		addRequirements(drivetrain);
		this.drive = drive;
		this.translationSup = translationSup;
        this.strafeSup = strafeSup;
        this.rotationSup = rotationSup;
	}

	public void execute() {
		double translationVal = MathUtil.applyDeadband(translationSup.getAsDouble(), 0.05);
        double strafeVal = MathUtil.applyDeadband(strafeSup.getAsDouble(), 0.05);
        double rotationVal = MathUtil.applyDeadband(rotationSup.getAsDouble(), 0.05);

		drivetrain.setControl(
			drive.withVelocityX(translationVal * MaxSpeed)
			.withVelocityY(strafeVal * MaxSpeed)
			.withRotationalRate(rotationVal * MaxAngularRate)
		);
	}

	public boolean isFinished() {
		return false;
	}

	protected void end() {
	}

	protected void interrupted() {
	}
}
