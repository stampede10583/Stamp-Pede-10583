package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import edu.wpi.first.wpilibj.PS4Controller.Button;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.Constants.ScoringConstants;
//import frc.robot.commands.MoveToHeightCommand;
import frc.robot.commands.ShootCommand;

import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.MotorTestSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import java.util.List; 
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.util.PathPlannerLogging;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import frc.robot.commands.ShootCommand; 
import frc.robot.AlignmentSubsystem;

/*
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems
  private DriveSubsystem m_robotDrive;
  //private final ShooterSubsystem m_shooter = new ShooterSubsystem();
  //private final ElevatorSubsystem m_elevator = new ElevatorSubsystem();
  
  //private final ShootCommand m_ShootCommand = new ShootCommand(m_shooter);
  private SendableChooser<Command> autoChooser;

  private AlignmentSubsystem m_aligner;
  
  // The driver's controller
  CommandXboxController m_driverController = new CommandXboxController(OIConstants.kDriverControllerPort);

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure the button binding
    
    // NamedCommands.registerCommand("shooterShoot", m_shooter.shoot());
    // NamedCommands.registerCommand("shooterShootL1", m_shooter.shootL1());
    // NamedCommands.registerCommand("shooterIntake", m_shooter.intake());
    // NamedCommands.registerCommand("shooterStop", m_shooter.stop());
    // NamedCommands.registerCommand("elevatorUp", m_elevator.upCommand());
    // NamedCommands.registerCommand("elevatorDown", m_elevator.downCommand());
    // NamedCommands.registerCommand("elevatorStop", m_elevator.stop());

    m_robotDrive = new DriveSubsystem();

    autoChooser = AutoBuilder.buildAutoChooser("Back and Shoot 1");
    SmartDashboard.putData("Auto Mode", autoChooser);

    m_aligner = new AlignmentSubsystem(m_robotDrive);

    boolean InTelleop = true;
    
    
    

    // double AlignmentGetLeftY;
    // double AlignmentGetLeftX;
    // double AlignmentGetRightX;
    configureButtonBindings();
    m_robotDrive.resetEncoders();
    m_robotDrive.zeroHeading();
    //Configure default commands
    m_robotDrive.setDefaultCommand(
        // The left stick controls translation of the robot.
        // Turning is controlled by the X axis of the right stick.
        

        
        

        new RunCommand(
            () -> m_robotDrive.drive(
                -MathUtil.applyDeadband(m_driverController.getLeftY(), OIConstants.kDriveDeadband),
                -MathUtil.applyDeadband(m_driverController.getLeftX(), OIConstants.kDriveDeadband),
                -MathUtil.applyDeadband(m_driverController.getRightX(), OIConstants.kDriveDeadband),
                true),
            m_robotDrive));
            

    PathPlannerLogging.setLogCurrentPoseCallback((pose) -> {
            System.out.println(pose);
        });
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be
   * created by
   * instantiating a {@link edu.wpi.first.wpilibj.GenericHID} or one of its
   * subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then calling
   * passing it to a
   * {@link JoystickButton}.
   */
  private void configureButtonBindings() {
    // m_driverController.rightBumper()
    //     .whileTrue(new RunCommand(
    //         () -> m_robotDrive.setX(),
    //         m_robotDrive));

    // m_driverController.a()
    //   .onTrue(new MoveToHeightCommand(m_elevator, ScoringConstants.kElevatorLevel1));


    // m_driverController.b()
    //   .onTrue(new MoveToHeightCommand(m_elevator, ScoringConstants.kElevatorLevel2));
    
    // m_driverController.x()
    //   .onTrue(new MoveToHeightCommand(m_elevator, ScoringConstants.kElevatorLevel3));
  
    // m_driverController.y()
    //   .onTrue(new MoveToHeightCommand(m_elevator, ScoringConstants.kElevatorLevel4));


    // m_driverController.leftTrigger()
    // .onTrue(m_shooter.intake());
    // m_driverController.leftTrigger()
    // .onFalse(m_shooter.stop());
    //  m_driverController.rightTrigger()
    //    .onTrue(m_shooter.shoot());
    //  m_driverController.rightTrigger()
    //    .onFalse(m_shooter.stop());

    //    m_driverController.povUp()
    //    .onTrue(m_shooter.shootL1());
    //  m_driverController.povUp()
    //    .onFalse(m_shooter.stop());

    /*
     m_driverController.leftBumper()
        .onTrue(m_elevator.downCommand());
    m_driverController.leftBumper()
      .onFalse(m_elevator.stop());
    m_driverController.rightBumper()
       .onTrue(m_elevator.upCommand());
     m_driverController.rightBumper()
       .onFalse(m_elevator.stop());
    */


       m_driverController.rightStick()
        .onTrue(m_robotDrive.zeroCommand());


//ABSOLUTELY MUST CHANGE THE ON FALSE TO BE A TIMED COMMAND OTHERWISE WE SOLVED NOTHING
//ABSOLUTELY MUST CHANGE THE ON FALSE TO BE A TIMED COMMAND OTHERWISE WE SOLVED NOTHING
//ABSOLUTELY MUST CHANGE THE ON FALSE TO BE A TIMED COMMAND OTHERWISE WE SOLVED NOTHING
//ABSOLUTELY MUST CHANGE THE ON FALSE TO BE A TIMED COMMAND OTHERWISE WE SOLVED NOTHING
        m_driverController.povLeft()
        .onTrue(m_aligner.align());
      m_driverController.povLeft()
        .onFalse(m_aligner.alignLeft().andThen(m_aligner.stop()));
  
        
        
        //ABSOLUTELY MUST CHANGE THE ON FALSE TO BE A TIMED COMMAND OTHERWISE WE SOLVED NOTHING
        //ABSOLUTELY MUST CHANGE THE ON FALSE TO BE A TIMED COMMAND OTHERWISE WE SOLVED NOTHING
        //ABSOLUTELY MUST CHANGE THE ON FALSE TO BE A TIMED COMMAND OTHERWISE WE SOLVED NOTHING
        //ABSOLUTELY MUST CHANGE THE ON FALSE TO BE A TIMED COMMAND OTHERWISE WE SOLVED NOTHING
        m_driverController.povRight()
        .onTrue(m_aligner.align());
      m_driverController.povRight()
        .onFalse(m_aligner.alignRight().andThen(m_aligner.stop()));
        
    //  if(m_driverController.rightStick()){
    //   (m_robotDrive.zeroCommand());
    //  }
      
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
 /*  public Command getAutonomousCommand() {
    // Create config for trajectory
    TrajectoryConfig config = new TrajectoryConfig(
        AutoConstants.kMaxSpeedMetersPerSecond,
        AutoConstants.kMaxAccelerationMetersPerSecondSquared)
        // Add kinematics to ensure max speed is actually obeyed
        .setKinematics(DriveConstants.kDriveKinematics);

    // An example trajectory to follow. All units in meters.
    Trajectory exampleTrajectory = TrajectoryGenerator.generateTrajectory(
        // Start at the origin facing the +X direction
        new Pose2d(0, 0, new Rotation2d(0)),
        // Pass through these two interior waypoints, making an 's' curve path
        List.of(new Translation2d(1, 1), new Translation2d(2, -1)),
        // End 3 meters straight ahead of where we started, facing forward
        new Pose2d(3, 0, new Rotation2d(0)),
        config);

    var thetaController = new ProfiledPIDController(
        AutoConstants.kPThetaController, 0, 0, AutoConstants.kThetaControllerConstraints);
    thetaController.enableContinuousInput(-Math.PI, Math.PI);

    SwerveControllerCommand swerveControllerCommand = new SwerveControllerCommand(
        exampleTrajectory,
        m_robotDrive::getPose, // Functional interface to feed supplier
        DriveConstants.kDriveKinematics,

        // Position controllers
        new PIDController(AutoConstants.kPXController, 0, 0),
        new PIDController(AutoConstants.kPYController, 0, 0),
        thetaController,
        m_robotDrive::setModuleStates,
        m_robotDrive);

    // Reset odometry to the starting pose of the trajectory.
    m_robotDrive.resetOdometry(exampleTrajectory.getInitialPose());

    // Run path following command, then stop at the end.
    return swerveControllerCommand.andThen(() -> m_robotDrive.drive(0, 0, 0, false));
  } */


  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }
}