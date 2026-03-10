package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.Auto.AutoLowerIntake;
import frc.robot.commands.Auto.AutoRaiseIntake;
import frc.robot.commands.CANdle.SetColorFlow;
import frc.robot.commands.Hood.JoystickHood;
import frc.robot.commands.Hood.SetHoodPosition;
import frc.robot.commands.Serializer.StopUptake;
import frc.robot.commands.Serializer.RunAgitator;
import frc.robot.commands.Serializer.RunDrum;
import frc.robot.commands.Serializer.RunUptake;
import frc.robot.commands.Serializer.SlowAgitator;
import frc.robot.commands.Serializer.StopAgitator;
import frc.robot.commands.Serializer.StopDrum;
import frc.robot.commands.Shooter.JoystickShooter;
import frc.robot.commands.Shooter.SetShooterVelocity;
import frc.robot.commands.Swerve.TeleopDrive;
import frc.robot.commands.Turret.AimToShoot;
import frc.robot.commands.Turret.AimToShootPose;
import frc.robot.commands.Turret.AimToShootPoseOnly;
import frc.robot.commands.Turret.AutoAim;
import frc.robot.commands.Turret.AutoAimPose;
import frc.robot.commands.Turret.ContinuousSetShooterAndHood;
import frc.robot.commands.Turret.JoystickTurret;
import frc.robot.commands.Turret.SetTurretPosition;
import frc.robot.commands.Turret.ZeroTurret;
import frc.robot.dashboard.AutoAimDashboard;
import frc.robot.generated.TunerConstants;
import frc.robot.commands.Intake.JoystickIntakeWrist;
import frc.robot.commands.Intake.ReverseIntake;
import frc.robot.commands.Intake.RunIntake;
import frc.robot.commands.Intake.SetIntakeWristPosition;
import frc.robot.commands.Intake.StopIntake;
import frc.robot.subsystems.Hood;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.ShooterLimelight;
import frc.robot.subsystems.Turret;
import frc.robot.subsystems.Uptake;
import frc.robot.subsystems.Drum;
import frc.robot.subsystems.Agitator;
import frc.robot.subsystems.CANdleSubsystem;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.IntakeWrist;
import frc.robot.subsystems.PoseEst;
import frc.robot.subsystems.PoseLimelight;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  private double MaxSpeed = 1.0 * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
  private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

  private final int translationAxis = XboxController.Axis.kLeftY.value;
  private final int strafeAxis = XboxController.Axis.kLeftX.value;
  private final int rotationAxis = XboxController.Axis.kRightX.value;

  /* Setting up bindings for necessary control of the swerve drive platform */
  private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
    .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
    .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
  private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
  private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

  private final Telemetry logger = new Telemetry(MaxSpeed);

  public static CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();

  // The robot's subsystems and commands are defined here...
  public static Shooter shooter = new Shooter(); 
  public static Hood hood = new Hood();
  public static Turret turret = new Turret();
  public static Uptake uptake = new Uptake();
  public static Drum drum = new Drum();
  public static Agitator agitator = new Agitator();
  public static IntakeWrist intakeWrist = new IntakeWrist();
  public static Intake intake = new Intake();
  public static ShooterLimelight shooterLimelight = new ShooterLimelight();
  public static PoseEst poseEst = new PoseEst();
  public static PoseLimelight poseLimelight = new PoseLimelight();
  public static CANdleSubsystem caNdleSubsystem = new CANdleSubsystem();

  /* Path follower */
  private SendableChooser<Command> autoChooser = new SendableChooser<>();

  // Establish Controllers
  public final CommandXboxController driverController =
      new CommandXboxController(OperatorConstants.kDriverControllerPort);

        private final CommandXboxController operatorController =
      new CommandXboxController(OperatorConstants.kOperatorControllerPort);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    registerNamedCommands();

    ShuffleboardTab autoTab = Shuffleboard.getTab("Auto settings");
    autoChooser.addOption("MidPassRight", new PathPlannerAuto("MidPassRight"));
    autoChooser.addOption("MidPassLeft", new PathPlannerAuto("MidPassLeft"));
    autoChooser.addOption("DoubleShotRight", new PathPlannerAuto("DoubleShotRight"));
    autoChooser.addOption("DoubleShotRightSteal", new PathPlannerAuto("DoubleShotRightSteal"));
    autoChooser.addOption("DoubleShotLeft", new PathPlannerAuto("DoubleShotLeft"));
    autoChooser.addOption("DoubleShotLeftDepot", new PathPlannerAuto("DoubleShotLeftDepot"));
    autoChooser.addOption("Depot", new PathPlannerAuto("Depot"));
    
    autoTab.add("Mode", autoChooser);
    
    // Set Default Commands
    uptake.setDefaultCommand(new StopUptake());
    drum.setDefaultCommand(new StopDrum());
    //agitator.setDefaultCommand(new StopAgitator());
    //intake.setDefaultCommand(new StopIntake());

    // Choose a default turret command
    turret.setDefaultCommand(new AutoAimPose().alongWith(new ContinuousSetShooterAndHood()));  // use only for matches
    //turret.setDefaultCommand(new SetTurretPosition(0));  // stop turret when starting 
    //turret.setDefaultCommand(new JoystickTurret());

    drivetrain.setDefaultCommand(
      new TeleopDrive(drivetrain,
      drive,
      () -> -driverController.getRawAxis(translationAxis),
      () -> -driverController.getRawAxis(strafeAxis),
      () -> -driverController.getRawAxis(rotationAxis)  
      )
    );
    
    // Joysticks Used for Tuning
    //hood.setDefaultCommand(new JoystickHood());
    //shooter.setDefaultCommand(new JoystickShooter());
    //intakeWrist.setDefaultCommand(new JoystickIntakeWrist());

    // Configure the trigger bindings
    configureBindings();

    AutoAimDashboard.AddDashboard();
      
    }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    // Idle while the robot is disabled. This ensures the configured
    // neutral mode is applied to the drive motors while disabled.

    // Shoot
    driverController.rightBumper().whileTrue(new RunAgitator().alongWith(new RunDrum().alongWith(new RunUptake().alongWith(new RunIntake().alongWith(new SetColorFlow())))));
    driverController.rightBumper().onFalse(new StopDrum().alongWith(new StopUptake().alongWith(new SlowAgitator())));

    //drop intake
    driverController.b().onTrue(new SetIntakeWristPosition(5.15));

    // pick up intake
    driverController.b().multiPress(2,1).onTrue(new SetIntakeWristPosition(0));

    // shoot using pose only
    driverController.leftTrigger().whileTrue(new AimToShootPoseOnly());
    // driverController.leftTrigger().onFalse(new AutoAimPose());
    driverController.leftTrigger().onFalse(new AutoAimPose().alongWith(new ContinuousSetShooterAndHood()));

    // shoot using turret camera; use in case the pose is sad
    driverController.rightTrigger().whileTrue(new AimToShootPose());
    // driverController.rightTrigger().onFalse(new AutoAimPose());
    driverController.rightTrigger().onFalse(new AutoAimPose().alongWith(new SetHoodPosition(0)));

    //intake
    driverController.leftBumper().onTrue(new RunIntake().alongWith(new SlowAgitator()));
    driverController.leftBumper().multiPress(2, 1).onTrue(new StopIntake().alongWith(new StopAgitator()));
    //driverController.leftBumper().multiPress(2, 1).onTrue(new StopIntake().alongWith(new StopAgitator()).alongWith(new SetHoodPosition(Constants.Shooter.Hood.StoreHoodPosition)).alongWith(new SetShooterVelocity((0))));
  
    // Operator Take Control of Turret (also Emergency Disable Turret)
    operatorController.leftTrigger().whileTrue(new JoystickTurret());

    // Operator Rezero
    operatorController.leftTrigger().and(operatorController.a().onTrue(new ZeroTurret()));

    // Operator Emergency Fix Intake Belt
    operatorController.x().whileTrue(new ReverseIntake());
    operatorController.x().onFalse(new RunIntake());

  final var idle = new SwerveRequest.Idle();
    RobotModeTriggers.disabled().whileTrue(
      drivetrain.applyRequest(() -> idle).ignoringDisable(true)
    );
  }

  /* Sets Joystick Deadband */
    public static double stickDeadband(double value, double deadband, double center) {
        return (value < (center + deadband) && value > (center - deadband)) ? center : value;
    }

    /* Passes Along Joystick Values for Elevator and Wrist */
    public double getOperatorLeftStickY() {
       return stickDeadband(this.operatorController.getRawAxis(1), 0.05, 0.0);
    }

    public double getOperatorLeftStickX() {
        return stickDeadband(this.operatorController.getRawAxis(0), 0.05, 0.0);
    }

    public double getOperatorRightStickY() {
        return stickDeadband(this.operatorController.getRawAxis(5), 0.05, 0.0);
    }

    public void doubleRumble() {
        driverController.setRumble(GenericHID.RumbleType.kLeftRumble, 0.5);
    }

    public void stopRumble() {
        driverController.setRumble(GenericHID.RumbleType.kLeftRumble, 0.0);
    }

    // facing away from our alliance station wall (0 deg).
    //drivetrain.runOnce(() -> drivetrain.seedFieldCentric(Rotation2d.kZero)),

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }

    public void registerNamedCommands() {
        /* Command registration for PathPlanner */     
        NamedCommands.registerCommand("LowerIntake", new AutoLowerIntake());
        NamedCommands.registerCommand("RaiseIntake", new AutoRaiseIntake());
        NamedCommands.registerCommand("AutoAim", new AutoAimPose().alongWith(new ContinuousSetShooterAndHood()));
        NamedCommands.registerCommand("AimToShoot", new AimToShootPoseOnly());
        NamedCommands.registerCommand("AimToShoot8", new AimToShootPoseOnly().withTimeout(1.25));
        NamedCommands.registerCommand("AimToShoot20", new AimToShootPoseOnly().withTimeout(3));
        NamedCommands.registerCommand("AimToShoot30", new AimToShootPoseOnly().withTimeout(4));
        NamedCommands.registerCommand("AimToShoot60", new AimToShootPoseOnly().withTimeout(6));
        NamedCommands.registerCommand("ShootCommand", new RunAgitator().alongWith(new RunDrum().alongWith(new RunUptake().alongWith(new RunIntake()))));
        NamedCommands.registerCommand("RunIntake", new RunIntake().alongWith(new SlowAgitator()));
        NamedCommands.registerCommand("StopPass", new SetHoodPosition(0).andThen(new AutoAimPose()));
        NamedCommands.registerCommand("StopShoot", new StopDrum().alongWith(new StopUptake()));
    }
}
