package frc.robot;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.DriveSubsystem;

import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;

import com.revrobotics.spark.SparkBase.ControlType;

import edu.wpi.first.cscore.CvSink;
import edu.wpi.first.cscore.CvSource;
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

    

    // public Command align(){
    //     return runOnce(() -> {
    //         //drive.drive(0, 1, 0, false); 
    //         hasTarget = LimelightHelpers.getTV("");
    //         if(hasTarget){


    //             double targetYaw = updateYaw();
    //             driveRotated(targetYaw);
    //             targetYaw = updateYaw();
                
    
    //         }


    // });
    // }

    // public double updateYaw(){
    //     return LimelightHelpers.getTX("");
    // }


    // public void driveRotated(double yaw){
    //     int rotation = 0;
    //     if(yaw < 0){
    //         rotation = -1;
            
    //     }
    //     else if(yaw > 0){
    //         rotation = 1; 
            
    //     }
        
    //     drive.drive(-2, 0, -rotation, false);
    //     while(!(Math.abs(yaw) > 5)){
    //         yaw = updateYaw();
    //     }

    //     drive.drive(-2, 0, 0, false);
        
    // }
    
    // public Command adjustLeft(){
    //     return runOnce(() -> {
    //         drive.drive(0, 1, 0, false); 
            


    // });
    // }


    // public Command adjustRight(){
    //     return runOnce(() -> {
    //         drive.drive(0, -1, 0, false);
            

    //     });
    // }

    // public Command alignLeft(){
    //     return new SequentialCommandGroup(
    //         this.adjustLeft().withTimeout(0.2), 
    //     new InstantCommand(() -> this.stop()) 
    //     );
    // }

    // public Command alignRight(){
    //     return new SequentialCommandGroup(
    //         this.adjustRight().withTimeout(0.2), 
    //     new InstantCommand(() -> this.stop()) 
    //     );
    // }


    // public Command stop(){
    //     return runOnce(() -> {
    //         drive.drive(0, 0, 0, true);
            

    //     });
    // }
    
    public static Mat drawCrosshair(Mat frame) {
    int width = frame.width();
    int height = frame.height();

    // Calculate the center position for the crosshair
    int centerX = width / 2;
    int centerY = (int) (height * 0.75); // Three-quarters down the screen

    int crosshairSize = 20;
    Scalar color = new Scalar(0, 249, 0); // Orange colour
    int thickness = 2;

    // Draw horizontal line
    Imgproc.line(frame, new Point(centerX - crosshairSize, centerY),
            new Point(centerX + crosshairSize, centerY), color, thickness);
    // Draw vertical line
    Imgproc.line(frame, new Point(centerX, centerY - crosshairSize),
            new Point(centerX, centerY + crosshairSize), color, thickness);
    

    
    return frame;
    
    }


    
    
}
