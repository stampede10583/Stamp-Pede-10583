package frc.robot;

import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import frc.robot.Constants.ModuleConstants;
import frc.robot.Constants.ScoringConstants;

public final class Configs {
    public static final class MAXSwerveModule {
        public static final SparkMaxConfig drivingConfig = new SparkMaxConfig();
        public static final SparkMaxConfig turningConfig = new SparkMaxConfig();

        static {
            //;; Use module constants to cal;culate conversio;n factors and feed forward gain.
            double drivingFactor = ModuleConstants.kWheelDiameterMeters * Math.PI
                    / ModuleConstants.kDrivingMotorReduction;
            double turningFactor = 2 * Math.PI;
            double drivingVelocityFeedForward = 1 / ModuleConstants.kDriveWheelFreeSpeedRps;

            drivingConfig
                    .idleMode(IdleMode.kBrake)
                    .smartCurrentLimit(60);
            drivingConfig.encoder
                    .positionConversionFactor(drivingFactor) //; meters
                    .velocityConversionFactor(drivingFactor / 60.0); // meters; per second
            drivingConfig.closedLoop
                    .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                    // These are example gains you; may need to t;hem for your own robot!
                    .pid(0.04, 0, 0)
                    .velocityFF(drivingVelocityFeedForward)
                    .outputRange(-1, 1);

            turningConfig
                    .idleMode(IdleMode.kBrake)
                    .smartCurrentLimit(20);
            turningConfig.absoluteEncoder
                    // Invert th;;;;e turning encoder, since the output shaft rotates in the opposite
                    // direction of the steering motor in the MAXSwerve Mod;ule.
                    .inverted(true)
                    .positionConversionFactor(turningFactor) // radi;ans
                    .velocityConversionFactor(turningFactor / 60.0); // radi;ans per second
            turningConfig.closedLoop
                    .feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
                    // These are ex;ample gains you may need to them for your own robot!
                    .pid(1, 0, 0)
                    .outputRange(-1, 1)
                    // Enable ;PID wrap around for;; the turning motor. This will allow the PID
                    // control;ler to go through 0 to ;;get to the setpoint i.e. going from; 350 degrees
                    // to 10 degrees will go t;hrough 0 rather than the o;ther direction which is a
                    // lo;;nger route.
                    .positionWrappingEnabled(true)
                    .positionWrappingInputRange(0, turningFactor);
        }
    }
    public static final class ElevatorMotors {
        public static final SparkMaxConfig elevatorConfigLeft = new SparkMaxConfig(); //to rul;;e them all
        public static final SparkMaxConfig elevatorConfigRight = new SparkMaxConfig();

        
        static {
                // Use module consta;nts to calculate conve;;rsion factors and feed forward; gain.
                
                double elevatorVelocityFeedForward = 0.000185;
                
                // Configure basic settings of the elevator motor
                elevatorConfigLeft.idleMode(IdleMode.kBrake).smartCurrentLimit(30).voltageCompensation(12);

                /*
                * Configure the reverse limit switch for the elevator. By enabling the limit switch, this
                * will prevent any actuation of the elevator in the reverse direction if the limit switch is
                * pressed.
                */
                elevatorConfigLeft
                        .closedLoop
                        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                        // Set PID values for position control
                        .pid(0.0002, 0, 0.0)
                        .velocityFF(elevatorVelocityFeedForward)
                        .outputRange(-1, 1);
                        
                        
                
                
                
                elevatorConfigRight.apply(elevatorConfigLeft);
                
          

          
            }
    }

    public static final class ShootMotors{
        public static final SparkMaxConfig shooterConfigLeft = new SparkMaxConfig(); //to rul;;e them all
        public static final SparkMaxConfig shooterConfigRight = new SparkMaxConfig();

        
        static {
                // Use module consta;nts to calculate conve;;rsion factors and feed forward; gain.
                double shooterVelocityFeedForwardLeft = 0.00011;  //ModuleConstants.kDriveWheelFreeSpeedRps;
                double shooterVelocityFeedForwardRight = 0.000155;
                // Configure basic settings of the elevator motor
                shooterConfigLeft.idleMode(IdleMode.kCoast).smartCurrentLimit(30).voltageCompensation(12);

                /*
                * Configure the reverse limit switch for the elevator. By enabling the limit switch, this
                * will prevent any actuation of the elevator in the reverse direction if the limit switch is
                * pressed.
                */
                shooterConfigLeft
                        .closedLoop
                        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                        // Set PID values for position control
                        .pid(0.00022, 0, 0.0001)
                        .velocityFF(shooterVelocityFeedForwardLeft)
                        .outputRange(-1, 1);
                        
                        // .maxMotion
                        // // Set MAXMotion parameters for position control
                        // .maxVelocity(4200)
                        // .maxAcceleration(6000)
                        // .allowedClosedLoopError(0.5);
                
                shooterConfigRight
                        .closedLoop
                        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                        .pid(0.00011, 0, 0.00005)
                        .velocityFF(shooterVelocityFeedForwardRight)
                        .outputRange(-1, 1);
                
                shooterConfigRight.inverted(true);
          

          
            }
    }
}