// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.

// package frc.robot.commands.AMP;

// import edu.wpi.first.wpilibj2.command.Command;
// import frc.robot.Constants.AMPConstants.ArmMode;
// import frc.robot.subsystems.AMPSubsystem;

// public class ArmAuto extends Command {
//   private AMPSubsystem ampSubsystem;
//   private ArmMode armMode;

//   /** Creates a new ArmAuto. */
//   public ArmAuto(AMPSubsystem ampSubsystem, ArmMode armMode) {
//     // Use addRequirements() here to declare subsystem dependencies.
//     this.ampSubsystem = ampSubsystem;
//     this.armMode = armMode;

//     addRequirements(ampSubsystem);
//   }

//   // Called when the command is initially scheduled.
//   @Override
//   public void initialize() {
//     if (armMode == ArmMode.kAMP) {
//       ampSubsystem.setAMP();
//     } else {
//       ampSubsystem.setArmDefult();
//     }
//     // if(armMode == ArmMode.kDefult)
//     // ampSubsystem.setArmDefult();
//   }

//   // Called every time the scheduler runs while the command is scheduled.
//   @Override
//   public void execute() {
//   }

//   // Called once the command ends or is interrupted.
//   @Override
//   public void end(boolean interrupted) {
//   }

//   // Returns true when the command should end.
//   @Override
//   public boolean isFinished() {
//     return true;
//   }
// }
