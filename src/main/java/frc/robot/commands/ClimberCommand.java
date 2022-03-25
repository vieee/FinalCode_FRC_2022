// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ClimberSubsystem;

public class ClimberCommand extends CommandBase {
  private final ClimberSubsystem climberSubsystem;
  Supplier<Double> onesAxis, twosAxis;

  /** Creates a new ClimberCommand. */
  public ClimberCommand(ClimberSubsystem climberSubsystem, Supplier<Double> onesAxis, Supplier<Double> twosAxis) {
    this.climberSubsystem = climberSubsystem;
    this.onesAxis = onesAxis;
    this.twosAxis = twosAxis;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(this.climberSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double onesReading = onesAxis.get();
    double twosReading = twosAxis.get();

    SmartDashboard.putNumber("Left Reading", onesReading);

    this.climberSubsystem.moveRightMotor(twosReading);
    // this.climberSubsystem.moveLeftMotor(onesReading);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
