// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.

// package frc.robot.commands.climber;

// import edu.wpi.first.wpilibj2.command.CommandBase;

// import frc.robot.subsystems.climber.TelescopingTube;

// public class MoveTubeToPosition extends CommandBase {

//     // Save the tube that we're interacting with
//     TelescopingTube tube;

//     // The desired final length
//     double desiredFinalLength;

//     /**
//      * Creates a new MoveTubeToPosition.
//      *
//      * <p>WARNING: IF THE DESIRED DISTANCE IS OUT OF SCOPE, THE COMMAND WILL NEVER FINISH
//      */
//     public MoveTubeToPosition(TelescopingTube tube, double desiredDistance) {
//         // Use addRequirements() here to declare subsystem dependencies.
//         addRequirements(tube);

//         // Save the tube to interact with
//         this.tube = tube;

//         // Save the desired final length
//         this.desiredFinalLength = desiredDistance;
//     }

//     // Called when the command is initially scheduled.
//     @Override
//     public void initialize() {

//         // Set that the tube should move to the set position
//         tube.setDesiredDistance(desiredFinalLength);
//     }

//     // Called every time the scheduler runs while the command is scheduled.
//     @Override
//     public void execute() {}

//     // Called once the command ends or is interrupted.
//     @Override
//     public void end(boolean interrupted) {}

//     // Returns true when the command should end.
//     @Override
//     public boolean isFinished() {
//         return tube.isAtDesiredDistance();
//     }
// }
