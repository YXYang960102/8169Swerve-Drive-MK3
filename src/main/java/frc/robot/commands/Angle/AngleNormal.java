// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Angle;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.AngleSubsystem;

public class AngleNormal extends Command {
  private AngleSubsystem angleSubsystem;
  private boolean isUP;
  /** Creates a new AngleNormal. */
  public AngleNormal(AngleSubsystem angleSubsystem, boolean isUP) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.angleSubsystem = angleSubsystem;
    this.isUP = isUP;

    addRequirements(angleSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if(isUP){
      angleSubsystem.AngleUP();
    }else{
      angleSubsystem.AngleDown();
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    angleSubsystem.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
