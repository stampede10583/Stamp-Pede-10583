package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  
  public static final class StolenConstants {
  public static final double X_REEF_ALIGNMENT_P = 1;
	public static final double Y_REEF_ALIGNMENT_P = 1;
	public static final double ROT_REEF_ALIGNMENT_P = 0.01;

	public static final double ROT_SETPOINT_REEF_ALIGNMENT = 0;  // Rotation
	public static final double ROT_TOLERANCE_REEF_ALIGNMENT = 1;
	public static final double X_SETPOINT_REEF_ALIGNMENT = -0.06;  // Vertical pose
	public static final double X_TOLERANCE_REEF_ALIGNMENT = 0.015;
	public static final double Y_SETPOINT_REEF_ALIGNMENT = 0.16;  // Horizontal pose
	public static final double Y_TOLERANCE_REEF_ALIGNMENT = 0.02;

	public static final double DONT_SEE_TAG_WAIT_TIME = 1;
	public static final double POSE_VALIDATION_TIME = 0.3;
  }
  
  
  public static final class DriveConstants {
    public static double driveMultiplier = 1;
    public static final double accelerationSlewRate = .5;  // Adjust for smoother starts
public static final double decelerationSlewRate = .5;  // Faster stopping


    // Driving Parameters - Note that these are not the maximum capable speeds of
    // the robot, rather the allowed maximum speeds
    public static final double kMaxSpeedMetersPerSecond = 4.8; //Fix this to 4.8
    public static final double kMaxAngularSpeed = 2*Math.PI; // radians per second fix this should be 2 pi

    // Chassis configuration
    public static final double kTrackWidth = Units.inchesToMeters(25.0625);
    // Distance between centers of right and left wheels on robot
    public static final double kWheelBase = Units.inchesToMeters(25.0625);
    // Distance between front and back wheels on robot
    public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
        new Translation2d(kWheelBase / 2, kTrackWidth / 2),
        new Translation2d(kWheelBase / 2, -kTrackWidth / 2),
        new Translation2d(-kWheelBase / 2, kTrackWidth / 2),
        new Translation2d(-kWheelBase / 2, -kTrackWidth / 2));

    // Angular offsets of the modules relative to the chassis in radians
    public static final double kFrontLeftChassisAngularOffset = -Math.PI / 2;
    public static final double kFrontRightChassisAngularOffset = 0;
    public static final double kBackLeftChassisAngularOffset = Math.PI;
    public static final double kBackRightChassisAngularOffset = Math.PI / 2;

    // SPARK MAX CAN IDs
    public static final int kFrontLeftDrivingCanId = 9;
    public static final int kRearLeftDrivingCanId = 7;
    public static final int kFrontRightDrivingCanId = 3;
    public static final int kRearRightDrivingCanId = 5;

    public static final int kFrontLeftTurningCanId = 8;
    public static final int kRearLeftTurningCanId = 6;
    public static final int kFrontRightTurningCanId = 2;
    public static final int kRearRightTurningCanId = 4;

    public static final boolean kGyroReversed = false;
  }

  public static final class ModuleConstants {
    // The MAXSwerve module can be configured with one of three pinion gears: 12T,
    // 13T, or 14T. This changes the drive speed of the module (a pinion gear with
    // more teeth will result in a robot that drives faster).
    public static final int kDrivingMotorPinionTeeth = 14;

    // Calculations required for driving motor conversion factors and feed forward
    public static final double kDrivingMotorFreeSpeedRps = NeoMotorConstants.kFreeSpeedRpm / 60;
    public static final double kWheelDiameterMeters = 0.0762;
    public static final double kWheelCircumferenceMeters = kWheelDiameterMeters * Math.PI;
    // 45 teeth on the wheel's bevel gear, 22 teeth on the first-stage spur gear, 15
    // teeth on the bevel pinion
    public static final double kDrivingMotorReduction = (45.0 * 22) / (kDrivingMotorPinionTeeth * 15);
    public static final double kDriveWheelFreeSpeedRps = (kDrivingMotorFreeSpeedRps * kWheelCircumferenceMeters)
        / kDrivingMotorReduction;
  }

  public static final class OIConstants {
    public static final int kDriverControllerPort = 0;
    public static final double kDriveDeadband = 0.1;
  }

  public static final class AutoConstants {
    public static final double kMaxSpeedMetersPerSecond = 4.8;
    public static final double kMaxAccelerationMetersPerSecondSquared = 3;
    public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI * 2;
    public static final double kMaxAngularSpeedRadiansPerSecondSquared = Math.PI / 2;

    public static final double kPXController = 1;
    public static final double kPYController = 1;
    public static final double kPThetaController = 1;

    // Constraint for the motion profiled robot angle controller
    public static final TrapezoidProfile.Constraints kThetaControllerConstraints = new TrapezoidProfile.Constraints(
        kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared);
  }

  public static final class NeoMotorConstants {
    public static final double kFreeSpeedRpm = 5676;
  }

  public static final class ScoringConstants  {
    public static final int kElevatorMotorLeftCanId = 11;
    public static final int kElevatorMotorRightCanId = 12;

    public static final int kShooterLeftCanId = 13;
    public static final int kShooterRightCanId = 14;


    public static final double kElevatorLevel0 = 46.0;
    public static final double kElevatorLevel1 = 46.0;
    public static final double kElevatorLevel2 = 81.0;
    public static final double kElevatorLevel3 = 121.0;
    public static final double kElevatorLevel4 = 183.0;

    //change when we actually have port
    public static final int kElevatorLimitSwitchPort = 15;


    //change when we actually have port
    public static final int kElevatorEncoderInputPort = 5;


    //pulse height equation?
    //input value
    // public static final double kElevatorSprocketDiameter = 21.23;
    // public static final int kEncoderPPR = 2048;
    // public static final double kElevatorSprocketCircumference = Math.PI*kElevatorSprocketDiameter;
    // public static final double kElevatorEncoderDistancePerPulse = kElevatorSprocketCircumference/kEncoderPPR;

//*********************************************************************************************************** */
    public static final double kElevatorHeightIncrement = 0; //ABSOLUTELY MUST CHANGE LATER
//*********************************************************************************************************** */
  //3500 normal speed
    public static final int kElevatorMotorSpeed = 3500;

    public static final double kElevatorSpeed = 0.95; //meters per second
}
}