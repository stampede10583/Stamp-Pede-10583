package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.Configs.MAXSwerveModule;
import frc.robot.Constants.ScoringConstants;

import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Configs;

public class ShooterSubsystem extends SubsystemBase{
    private SparkMax leftMotor;
    private SparkMax rightMotor;
    private final SparkClosedLoopController pidControllerLeft;
    private final SparkClosedLoopController pidControllerRight;

    public ShooterSubsystem(){
        leftMotor = new SparkMax(ScoringConstants.kShooterLeftCanId, MotorType.kBrushless);
        rightMotor = new SparkMax(ScoringConstants.kShooterRightCanId, MotorType.kBrushless);

        leftMotor.configure(Configs.ShootMotors.shooterConfigLeft, ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);

        rightMotor.configure(Configs.ShootMotors.shooterConfigRight, ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);

        pidControllerLeft = leftMotor.getClosedLoopController();
        pidControllerRight = rightMotor.getClosedLoopController();
        
    }

    public Command shoot(){
        return runOnce(() -> {
            pidControllerLeft.setReference(-1500, ControlType.kVelocity);
            pidControllerRight.setReference(-1500, ControlType.kVelocity);
    });
    }

    public Command shootL1(){
        return runOnce(() -> {
            pidControllerLeft.setReference(-300, ControlType.kVelocity);
            pidControllerRight.setReference(-1000, ControlType.kVelocity);
    });
}
    
    public Command intake(){
        return runOnce(() -> {
            pidControllerLeft.setReference(-800, ControlType.kVelocity);
            pidControllerRight.setReference(-800, ControlType.kVelocity);
    });
    
    }
    public Command stop(){
    return runOnce(() -> {
        System.out.println("Stopping");
        leftMotor.stopMotor();
        rightMotor.stopMotor(); 
    });
      
    }
}




