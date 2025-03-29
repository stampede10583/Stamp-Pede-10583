package frc.robot.commands;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.ScoringConstants;

public class SlowDrive extends Command {
    
  public SlowDrive(){

  }
    
    @Override
  public void initialize() {
    RobotContainer.switchDrive();

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    RobotContainer.switchDrive();
    return true;
  }


  
}
