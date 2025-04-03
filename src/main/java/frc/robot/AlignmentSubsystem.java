package frc.robot;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.DriveSubsystem;


import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LimelightHelpers;
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



import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LimelightHelpers;
import frc.robot.subsystems.DriveSubsystem;

public class AlignmentSubsystem extends SubsystemBase {
    private final DriveSubsystem driveSubsystem;
    private static NetworkTable limelightTable = 
        NetworkTableInstance.getDefault().getTable("limelight");


    public AlignmentSubsystem(DriveSubsystem driveSubsystem) {
        this.driveSubsystem = driveSubsystem;
    }

    public static Pose2d getAdjustedPose() {
        // Read botpose from the Limelight's NetworkTables
        double[] botpose = NetworkTableInstance.getDefault()
                            .getTable("limelight")
                            .getEntry("botpose").getDoubleArray(new double[6]);

        if (botpose.length < 6) {
            return new Pose2d(); // Return a default pose if data is invalid
        }

        // Extract original pose
        double x = botpose[0]; // X position in meters
        double y = botpose[1]; // Y position in meters
        double theta = botpose[5]; // Rotation in degrees

        // Apply offsets (adjust these values as needed)
        double xOffset = -0.3175;  // Shift in X direction (meters)
        double yOffset = -0.0127; // Shift in Y direction (meters)
        double angleOffset = 5; // Rotation offset in degrees

        // Create a new adjusted pose
        return new Pose2d(
            new Translation2d(x + xOffset, y + yOffset),
            Rotation2d.fromDegrees(theta + angleOffset)
        );
    }

    /** Gets the robot's pose relative to the field. */
    public Pose2d getRobotPose() {
        return driveSubsystem.getPose();
    }

    /** Gets the nearest AprilTag's pose. */
    public Pose2d getNearestTagPose() {
        LimelightHelpers.PoseEstimate limelightMeasurement = 
            LimelightHelpers.getBotPoseEstimate_wpiBlue("limelight");

        if (limelightMeasurement.tagCount >= 1) {
            return limelightMeasurement.pose;
        } else {
            return null; // No tag detected
        }
    }

    /** Rotates the robot to be flush with the nearest tag. */
    private Command rotateToTag() {
        return run(() -> {
            Pose2d tagPose = getNearestTagPose();
            if (tagPose != null) {
                double targetAngle = tagPose.getRotation().getDegrees();
                double currentAngle = driveSubsystem.getPose().getRotation().getDegrees();
                double error = targetAngle - currentAngle;
                double rotationSpeed = error * 0.02; // Simple P-controller

                driveSubsystem.drive(0, 0, rotationSpeed, false);
            }
        }).until(() -> {
            Pose2d tagPose = getNearestTagPose();
            if (tagPose != null) {
                double targetAngle = tagPose.getRotation().getDegrees();
                double currentAngle = driveSubsystem.getPose().getRotation().getDegrees();
                return Math.abs(targetAngle - currentAngle) < 2.0;
            }
            return false;
        });
    }

    /** Moves the robot 6.75 inches (0.17145 meters) to the left. */
    private Command strafeLeft() {
        return run(() -> driveSubsystem.drive(0, -0.5, 0, false))
            .withTimeout(0.5); // Adjust timing as needed
    }

    /** Moves the robot 6.75 inches (0.17145 meters) to the right. */
    private Command strafeRight() {
        return run(() -> driveSubsystem.drive(0, 0.5, 0, false))
            .withTimeout(0.5); // Adjust timing as needed
    }

    /** Drives forward until aligned with the tag. */
    private Command driveForwardToAlign() {
        return run(() -> {
            Pose2d tagPose = getNearestTagPose();
            if (tagPose != null) {
                Translation2d tagTranslation = tagPose.getTranslation();
                Translation2d robotTranslation = getAdjustedPose().getTranslation();
                double errorY = tagTranslation.getY() - robotTranslation.getY();

                driveSubsystem.drive(0.5, 0, 0, false);
            }
        }).until(() -> {
            Pose2d tagPose = getNearestTagPose();
            if (tagPose != null) {
                Translation2d tagTranslation = tagPose.getTranslation();
                Translation2d robotTranslation = getAdjustedPose().getTranslation();
                return Math.abs(tagTranslation.getY() - robotTranslation.getY()) < 0.05;
            }
            return false;
        });
    }

    /** Full command sequence: Rotate → Strafe Left → Drive Forward → Strafe Right */
    public Command leftAlighCommand() {
        return new SequentialCommandGroup(
            rotateToTag(),
            strafeLeft(),
            driveForwardToAlign()
            
        );
    }

    public Command rightAlignCommand() {
        return new SequentialCommandGroup(
            rotateToTag(),
            strafeRight(),
            driveForwardToAlign()
            
        );
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
