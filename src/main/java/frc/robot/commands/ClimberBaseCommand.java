// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
// import frc.robot.Constants.ClimberConstants;
import frc.robot.subsystems.ClimberSubsystem;

public class ClimberBaseCommand extends CommandBase {
  private ClimberSubsystem climberSubsystem;
  private double setPoint = 0;
  /** Creates a new ClimberBaseCommand. */
  public ClimberBaseCommand(ClimberSubsystem climberSubsystem, double setPoint) {
    this.climberSubsystem = climberSubsystem;
    this.setPoint = setPoint;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(this.climberSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    this.climberSubsystem.setMotor((ClimberSubsystem.leftEncoder.getDistance() - this.setPoint) * 0.25);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    this.climberSubsystem.setMotor(0);
  }
}

  // Returns true when the command should end.
/*  @Override
  public boolean isFinished() {
    return Math.abs(ClimberSubsystem.leftEncoder.getDistance() - this.setPoint) < ClimberConstants.ones_movement_threshold;
  }
}*/
