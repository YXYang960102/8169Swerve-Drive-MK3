// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.

// package frc.robot.commands.Shooter;

// import java.util.function.BooleanSupplier;

// import edu.wpi.first.units.Time;
// import edu.wpi.first.wpilibj.Timer;
// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
// import edu.wpi.first.wpilibj2.command.Command;
// import frc.robot.Constants.ShooterConstants;
// import frc.robot.Constants.ShooterConstants.SpeedSet;

// import frc.robot.subsystems.AMPSubsystem;
// import frc.robot.subsystems.AngleSubsystem;
// import frc.robot.subsystems.IntakeSubsystem;
// import frc.robot.subsystems.ShooterSubsystem;

// public class ShooterAuto extends Command {
//   private ShooterSubsystem shooterSubsystem;
//   private AngleSubsystem angleSubsystem;

//   private BooleanSupplier Onstop;
//   private SpeedSet speed;
//   Timer time = new Timer();

//   /** Creates a new ShooterAuto. */
//   public ShooterAuto(ShooterSubsystem shooterSubsystem,
//       AngleSubsystem angleSubsystem,
//       BooleanSupplier OnStop,
//       SpeedSet speed) {
//     // Use addRequirements() here to declare subsystem dependencies.

//     this.shooterSubsystem = shooterSubsystem;
//     this.angleSubsystem = angleSubsystem;
//     this.Onstop = OnStop;
//     this.speed = speed;
//   }

//   // Called when the command is initially scheduled.
//   @Override
//   public void initialize() {
//     shooterSubsystem.setSpeed(speed.topSpeed, speed.bottomSpeed);
//     time.reset();
//   }

//   // Called every time the scheduler runs while the command is scheduled.
//   @Override
//   public void execute() {
//     if (speed == SpeedSet.kManual) {
//       int rpmT = (int) SmartDashboard.getNumber("Shooter set RPM T", ShooterConstants.kShooterMotorDefaultRPM);
//       int rpmB = (int) SmartDashboard.getNumber("Shooter Set RPM B", ShooterConstants.kShooterMotorDefaultRPM);
//       shooterSubsystem.setSpeed(rpmT, rpmB);
//       angleSubsystem.setSpeaker();
      
//     }

//   }

//   // Called once the command ends or is interrupted.
//   @Override
//   public void end(boolean interrupted) {
//     shooterSubsystem.stop();
//     shooterSubsystem.CrawlStop();
//     ampCrawlSubsystem.AMPCrawlStop();
//     intakeSubsystem.IntakeStop();
//   }

//   // Returns true when the command should end.
//   @Override
//   public boolean isFinished() {
//     return Onstop != null ? Onstop.getAsBoolean() : false;
//   }
// }
