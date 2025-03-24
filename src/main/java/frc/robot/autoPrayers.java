
// package frc.robot;

// import com.revrobotics.spark.SparkBase.ControlType;

// import edu.wpi.first.wpilibj2.command.Command;
// import edu.wpi.first.wpilibj2.command.SubsystemBase;
// import edu.wpi.first.wpilibj2.command.WaitCommand;
// import frc.robot.subsystems.DriveSubsystem;
// import frc.robot.subsystems.ShooterSubsystem;


// public class autoPrayers extends SubsystemBase{
//     private DriveSubsystem m_robotDrive;
//     private ShooterSubsystem m_robotShoot;

//     public autoPrayers(DriveSubsystem m_robotDrive, ShooterSubsystem m_robotShoot){
//         this.m_robotDrive = m_robotDrive;
//         this.m_robotShoot = m_robotShoot;
//     }

//     public Command oracle(){
//         return runOnce(() -> {
//             m_robotDrive.drive(-2, 0, 0, true);
//             new WaitCommand(7);
//             m_robotShoot.shootL1();
//     });
//     }
// }
