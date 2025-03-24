package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Configs;
import frc.robot.Constants;
import frc.robot.Constants.ScoringConstants;
import edu.wpi.first.wpilibj.PWM;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;



public class ElevatorSubsystem extends SubsystemBase {
    private final SparkMax elevatorMotorLeft;
    private final SparkMax elevatorMotorRight;
    //private final PWM encoder;
    private final SparkClosedLoopController pidControllerLeft;
    private final SparkClosedLoopController pidControllerRight;
    // private final DigitalInput limitSwitch;
    private double height = ScoringConstants.kElevatorLevel0;
    private double targetHeight;
    private int targetLevel;
    private final RelativeEncoder encoder;
    private final double[] levelHeights = {ScoringConstants.kElevatorLevel1,ScoringConstants.kElevatorLevel2,ScoringConstants.kElevatorLevel3,ScoringConstants.kElevatorLevel4};
    

    // Preset Heights in Encoder Units (Modify as Needed)
    

    public ElevatorSubsystem() {
        elevatorMotorRight = new SparkMax(ScoringConstants.kElevatorMotorRightCanId, MotorType.kBrushless);
        elevatorMotorLeft = new SparkMax(ScoringConstants.kElevatorMotorLeftCanId, MotorType.kBrushless);

        elevatorMotorRight.configure(Configs.ElevatorMotors.elevatorConfigRight, ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);

        elevatorMotorLeft.configure(Configs.ElevatorMotors.elevatorConfigLeft, ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);

        //encoder = elevatorMotorRight.getEncoder();
        //encoder = new PWM(ScoringConstants.kElevatorEncoderInputPort);
        pidControllerLeft = elevatorMotorLeft.getClosedLoopController();
        pidControllerRight = elevatorMotorRight.getClosedLoopController();

        encoder = elevatorMotorLeft.getEncoder();
        targetLevel = 0;
        //limitSwitch = new DigitalInput(ScoringConstants.kElevatorLimitSwitchPort);

        // Configure PID Controller (Tune Gains as Needed)
        //pidController.setP(0.1);
        //pidController.setIAccum(0.0);
        //pidController.setD(0.0);
        //pidController.setFF(0.0);
        //pidController.setOutputRange(-1.0, 1.0);
    }

//FIX ALL OF THESE VALUES WHEN WE ACTUALLY NEED THEMN!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!! ahhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhh

    // public void setHeight(double targetHeight) {
    //     height = calculateHeight();
    //     if(height > targetHeight){
    //         runOnce(() -> this.downCommand());
    //     }
    //     if(height < targetHeight){
    //         runOnce(() -> this.upCommand());
    //     }
    //     this.periodic();
    //     this.targetHeight = targetHeight;
    // }

    // public double getCurrentHeight(double encoderCount) {
    //     height += encoderCount*ScoringConstants.kElevatorEncoderDistancePerPulse;
    //     return height;
    // }
    // public double getEncoderCount(){
    //     return encoder.getPosition();
    // }
    // public void resetEncoder() {
    //     encoder.setPosition(0);
    // }

    @Override
    public void periodic() {
        //SmartDashboard.putNumber("Elevator Position", getCurrentHeight(getEncoderCount()));
        // double heightAway = Math.abs(calculateHeight()-targetHeight);
        // if( heightAway < 2){
        //     runOnce(() -> this.stop());
        // }

        // if(calculateHeight() <= ScoringConstants.kElevatorLevel0){
        //     height = ScoringConstants.kElevatorLevel0;
        //     runOnce(() -> this.stop());
        //     height = ScoringConstants.kElevatorLevel0;
        // }

        // if(calculateHeight() >= ScoringConstants.kElevatorLevel4){
        //     height = ScoringConstants.kElevatorLevel4;
        //     runOnce(() -> this.stop());
        //     height = ScoringConstants.kElevatorLevel4;
        // }
    }

    public void setTargetLevel(int level) {
        if(level < 0 || level > 3) return;
        this.targetLevel = level;
        this.setTargetHeight(levelHeights[level]);
    }
    
    public double setTargetHeight(double height){
        this.targetHeight = height;
        
        return this.targetHeight;
    }

    public double getTargetHeight(){
        return this.targetHeight;
    }

    // public double calculateHeight(){
    //     height += getEncoderPosition() * ScoringConstants.kElevatorHeightIncrement;
        
    //     return height;
    // }

    // public double getHeight(){
    //     height = calculateHeight();
        
    //     return height;
    // }

    // public double getEncoderPosition(){
        
    //     if(getTargetHeight() < getHeight()){
    //         return encoder.getPosition()*-1;
    //     }

    //     return encoder.getPosition();
    // }


    public Command upCommand(){
        return runOnce(() -> {
            pidControllerLeft.setReference(ScoringConstants.kElevatorMotorSpeed, ControlType.kVelocity);
            pidControllerRight.setReference(-ScoringConstants.kElevatorMotorSpeed, ControlType.kVelocity);
    });
    }
    
    public Command downCommand(){
        return runOnce(() -> {
            pidControllerLeft.setReference(-ScoringConstants.kElevatorMotorSpeed, ControlType.kVelocity);
            pidControllerRight.setReference(ScoringConstants.kElevatorMotorSpeed, ControlType.kVelocity);
    });

    }

    public Command stop(){
        return runOnce(() ->{
            elevatorMotorLeft.stopMotor();
            elevatorMotorRight.stopMotor();
        });
    }
}
