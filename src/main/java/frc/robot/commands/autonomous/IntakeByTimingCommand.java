// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autonomous;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.IntakeConstants;
import frc.robot.subsystems.IntakeSubsystem;

public class IntakeByTimingCommand extends CommandBase {
  private final IntakeSubsystem intakeSubsystem;
  private double time;
  private double now; 
  /** Creates a new IntakeByRotationsCommand. */
  public IntakeByTimingCommand(IntakeSubsystem intakeSubsystem, double time) {
    this.intakeSubsystem = intakeSubsystem;
    this.time = time;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(this.intakeSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    this.now = Timer.getFPGATimestamp();
    // this.intakeSubsystem.setMode(1);
    this.intakeSubsystem.setIntakeSpeed(IntakeConstants.autonomousSpeed);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    this.intakeSubsystem.stopSystem();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (this.now > this.time);
  }
}
