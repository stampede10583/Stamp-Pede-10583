
    // Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.StolenConstants;
import frc.robot.LimelightHelpers;
import frc.robot.subsystems.DriveSubsystem;

public class AlignToReefTagRelative extends Command {
  private PIDController xController, yController, rotController;
  private boolean isRightScore;
  private Timer dontSeeTagTimer, stopTimer;
  private DriveSubsystem drivebase;
  private double tagID = -1;

  public AlignToReefTagRelative(boolean isRightScore, DriveSubsystem drivebase) {
    xController = new PIDController(StolenConstants.X_REEF_ALIGNMENT_P, 0.0, 0);  // Vertical movement
    yController = new PIDController(StolenConstants.Y_REEF_ALIGNMENT_P, 0.0, 0);  // Horitontal movement
    rotController = new PIDController(StolenConstants.ROT_REEF_ALIGNMENT_P, 0, 0);  // Rotation
    this.isRightScore = isRightScore;
    this.drivebase = drivebase;
    addRequirements(drivebase);
  }

  @Override
  public void initialize() {
    this.stopTimer = new Timer();
    this.stopTimer.start();
    this.dontSeeTagTimer = new Timer();
    this.dontSeeTagTimer.start();

    rotController.setSetpoint(StolenConstants.ROT_SETPOINT_REEF_ALIGNMENT);
    rotController.setTolerance(StolenConstants.ROT_TOLERANCE_REEF_ALIGNMENT);

    xController.setSetpoint(StolenConstants.X_SETPOINT_REEF_ALIGNMENT);
    xController.setTolerance(StolenConstants.X_TOLERANCE_REEF_ALIGNMENT);

    //yController.setSetpoint(isRightScore ? StolenConstants.Y_SETPOINT_REEF_ALIGNMENT : -StolenConstants.Y_SETPOINT_REEF_ALIGNMENT);
    yController.setSetpoint(0);
    yController.setTolerance(StolenConstants.Y_TOLERANCE_REEF_ALIGNMENT);

    tagID = LimelightHelpers.getFiducialID("");
    System.out.println(tagID);
  }

  @Override
  public void execute() {
    System.out.println(tagID);
    if (LimelightHelpers.getTV("") && LimelightHelpers.getFiducialID("") == tagID) {
      this.dontSeeTagTimer.reset();

      double[] postions = LimelightHelpers.getBotPose_TargetSpace("");
      SmartDashboard.putNumber("x", postions[2]);

      double xSpeed = -xController.calculate(postions[2]);
      SmartDashboard.putNumber("xspeed", xSpeed);
      //maybe change to be negative
      double ySpeed = yController.calculate(postions[0]); //we revomed a negative
      double rotValue = -rotController.calculate(postions[4]);

      drivebase.drive(xSpeed, ySpeed, rotValue, false);

      if (!rotController.atSetpoint() ||
          !yController.atSetpoint() ||
          !xController.atSetpoint()) {
        stopTimer.reset();
      }
    } else {
      drivebase.drive(0, 0, 0, false);
    }

    SmartDashboard.putNumber("poseValidTimer", stopTimer.get());
  }

  @Override
  public void end(boolean interrupted) {
    drivebase.drive(0, 0, 0, false);
  }

  @Override
  public boolean isFinished() {
    // Requires the robot to stay in the correct position for 0.3 seconds, as long as it gets a tag in the camera
    return this.dontSeeTagTimer.hasElapsed(StolenConstants.DONT_SEE_TAG_WAIT_TIME) ||
        stopTimer.hasElapsed(StolenConstants.POSE_VALIDATION_TIME);
  }
}
