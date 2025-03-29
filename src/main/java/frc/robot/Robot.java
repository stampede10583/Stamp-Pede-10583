package frc.robot;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.CvSink;
import edu.wpi.first.cscore.CvSource;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.PS4Controller.Button;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.commands.ExampleCommand;
//import frc.robot.commands.MoveToHeightCommand;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import edu.wpi.first.cameraserver.CameraServer;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import frc.robot.AlignmentSubsystem;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  private Command m_autonomousCommand;

  private RobotContainer m_robotContainer;

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
    // autonomous chooser on the dashboard.
    UsbCamera camera = CameraServer.startAutomaticCapture();

     camera.setResolution(640, 480);
     camera.setFPS(30);


     CvSink cvSink = CameraServer.getVideo();


     CvSource outputStream = CameraServer.putVideo("Crosshair Camera", 640, 480);

     new Thread(() -> processCameraFeed(cvSink, outputStream)).start();





    m_robotContainer = new RobotContainer();
    
  }

  private void drawCrosshair(Mat frame) {
    int centerX = frame.width() / 2;
    int centerY = frame.height() / 2;
    int crosshairSize = 20;
    Scalar color = new Scalar(0, 255, 0); // Green color
    int thickness = 2;

    // Draw horizontal line
    Imgproc.line(frame, new Point(centerX - crosshairSize, centerY),
            new Point(centerX + crosshairSize, centerY), color, thickness);
    // Draw vertical line
    Imgproc.line(frame, new Point(centerX, centerY - crosshairSize),
            new Point(centerX, centerY + crosshairSize), color, thickness);
}

  /**
   * This function is called every 20 ms, no matter the mode. Use this for items like diagnostics
   * that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */

   


  @Override
  public void robotPeriodic() {
    // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
    // commands, running already-scheduled commands, removing finished or interrupted commands,
    // and running subsystem periodic() methods.  This must be called from the robot's periodic
    // block in order for anything in the Command-based framework to work.
    CommandScheduler.getInstance().run();
  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    /*
     * String autoSelected = SmartDashboard.getString("Auto Selector",
     * "Default"); switch(autoSelected) { case "My Auto": autonomousCommand
     * = new MyAutoCommand(); break; case "Default Auto": default:
     * autonomousCommand = new ExampleCommand(); break; }
     */

    // schedule the autonomous command (example)
    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopInit() {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
    
}

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
    
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {

    
    
  }



  private void processCameraFeed(CvSink cvSink, CvSource outputStream) {
    Mat mat = new Mat();

    while (!Thread.interrupted()) {
        if (cvSink.grabFrame(mat) == 0) {
            continue; // Skip the frame if there's an error
        }
        // Draw the crosshair on the frame
        mat = AlignmentSubsystem.drawCrosshair(mat);
        // Send the frame to the dashboard
        outputStream.putFrame(mat);
    }
}
}