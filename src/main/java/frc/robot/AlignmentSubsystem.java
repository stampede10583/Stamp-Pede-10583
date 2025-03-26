package frc.robot;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.DriveSubsystem;

import com.revrobotics.spark.SparkBase.ControlType;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;

import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

import frc.robot.RobotContainer;

public class AlignmentSubsystem extends SubsystemBase {
    boolean hasTarget = LimelightHelpers.getTV("");
    DriveSubsystem drive;

    


    public AlignmentSubsystem(DriveSubsystem m_DriveSubsystem){
        this.drive = m_DriveSubsystem;
    }

    

    public Command align(){
        return runOnce(() -> {
            //drive.drive(0, 1, 0, false); 
            hasTarget = LimelightHelpers.getTV("");
            if(hasTarget){


                double targetYaw = updateYaw();
                driveRotated(targetYaw);
                targetYaw = updateYaw();
                
    
            }


    });
    }

    public double updateYaw(){
        return LimelightHelpers.getTX("");
    }


    public void driveRotated(double yaw){
        int rotation = 0;
        if(yaw < 0){
            rotation = -1;
            
        }
        else if(yaw > 0){
            rotation = 1; 
            
        }
        
        drive.drive(2, 0, rotation, false);
        while(!(Math.abs(yaw) > 5)){
            yaw = updateYaw();
        }

        drive.drive(2, 0, 0, false);
        
    }
    
    public Command adjustLeft(){
        return runOnce(() -> {
            drive.drive(0, 1, 0, false); 
            


    });
    }


    public Command adjustRight(){
        return runOnce(() -> {
            drive.drive(0, -1, 0, false);
            

        });
    }

    public Command alignLeft(){
        return new SequentialCommandGroup(
            this.adjustLeft().withTimeout(0.2), 
        new InstantCommand(() -> this.stop()) 
        );
    }

    public Command alignRight(){
        return new SequentialCommandGroup(
            this.adjustRight().withTimeout(0.2), 
        new InstantCommand(() -> this.stop()) 
        );
    }


    public Command stop(){
        return runOnce(() -> {
            drive.drive(0, 0, 0, true);
            

        });
    }
    

}
