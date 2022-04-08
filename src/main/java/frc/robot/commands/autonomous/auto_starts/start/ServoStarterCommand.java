// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autonomous.auto_starts.start;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IntakeOpeningSubsystem;

public class ServoStarterCommand extends CommandBase {
  private IntakeOpeningSubsystem intakeOpeningSubsystem;
  /** Creates a new ServoStarterCommand. */
  public ServoStarterCommand(IntakeOpeningSubsystem intakeOpeningSubsystem) {
    this.intakeOpeningSubsystem = intakeOpeningSubsystem;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(this.intakeOpeningSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    this.intakeOpeningSubsystem.setIntakeAngle();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    this.intakeOpeningSubsystem.setIntakeAngle();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
