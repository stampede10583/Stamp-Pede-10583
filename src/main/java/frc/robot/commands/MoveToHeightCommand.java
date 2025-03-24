// package frc.robot.commands;

// import edu.wpi.first.wpilibj2.command.Command;
// import frc.robot.subsystems.ElevatorSubsystem;


// public class MoveToHeightCommand extends Command {
    
//     private final ElevatorSubsystem elevator;
//     private double targetHeight;

//     public MoveToHeightCommand(ElevatorSubsystem elevator, double position) {
//         this.elevator = elevator;
//         this.targetHeight = position;
//         addRequirements(elevator);

//         //goUntilHeight(targetHeight);
//     }

//     @Override
//     public void initialize() {
//         elevator.setHeight(targetHeight);
//     }

//     /* 
//     @Override
//     public boolean isFinished() {
        
//         return Math.abs((elevator.getCurrentHeight(elevator.getEncoderCount())) - targetHeight) < 1.0; // Within 1 cm of target
//     }
//     */

//     @Override
//     public void end(boolean interrupted) {
//         if (interrupted) {
//             elevator.stop();
//         }
//     }


//     /*
//     //potentially make this a command with a Command.runUntil(() ->) function if we get problems
//     public void goUntilHeight(double height){
//         while(!isFinished()){
//             if(elevator.getCurrentHeight(-1*elevator.getEncoderCount()) > targetHeight){
//                 elevator.runDown();
//             }
//             else if(elevator.getCurrentHeight(elevator.getEncoderCount()) < targetHeight){
//                 elevator.runUp();
//             }
//         }
//         elevator.resetEncoder();
//     }
//     */
// }
