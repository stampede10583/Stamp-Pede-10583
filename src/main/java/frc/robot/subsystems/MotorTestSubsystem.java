package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.Configs.MAXSwerveModule;

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


import frc.robot.Configs;

public class MotorTestSubsystem extends SubsystemBase{

    private final SparkMax testMotor;
    private final RelativeEncoder encoder;
    private final SparkClosedLoopController pidController;
    // private final int motorType;
    
    public MotorTestSubsystem(int motorID){ //motorType 0 -> Neo, 1 -> 550
        // this.motorType = motorType;
        testMotor = new SparkMax(motorID, MotorType.kBrushless);

        // SparkMaxConfig config = new SparkMaxConfig();

        // config.closedLoop
        //     //neo config
        //     .p(0.04, ClosedLoopSlot.kSlot0)
        //     .i(0, ClosedLoopSlot.kSlot0)
        //     .d(0, ClosedLoopSlot.kSlot0)
        //     .velocityFF(1/917, ClosedLoopSlot.kSlot0)
        //     //550 config
        //     .p(0.04, ClosedLoopSlot.kSlot1)
        //     .i(0, ClosedLoopSlot.kSlot1)
        //     .d(0, ClosedLoopSlot.kSlot1)
        //     .velocityFF(1/413, ClosedLoopSlot.kSlot1);

        encoder = testMotor.getEncoder();
        pidController = testMotor.getClosedLoopController();


        // pidController.SetP(0.04)
        pidController.setIAccum(0.0);
        // pidController.setD(0.0);
        // pidController.setFF(1/917);
        //pidController.setOutputRange(-1.0, 1.0);
    }


    public Command test(double speed){
        // if(motorType == 1){
        //     System.out.println("Neo 1.1");
        //     return runOnce(() -> pidController.setReference(speed, SparkMax.ControlType.kVelocity, ClosedLoopSlot.kSlot0));
        // } 
        // System.out.println("Neo 550");
        // return runOnce(() -> pidController.setReference(speed, SparkMax.ControlType.kVelocity, ClosedLoopSlot.kSlot1));

        return runOnce(() -> pidController.setReference(speed, SparkMax.ControlType.kVelocity));


        
        
    }
    public Command stop(){
        return runOnce(() -> testMotor.stopMotor());
    }
}


