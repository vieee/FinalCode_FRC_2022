// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.teleop.intakeOpening;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.IntakeOpeningConstants;
import frc.robot.subsystems.IntakeOpeningSubsystem;

public class IntakeOpeningCommand extends CommandBase {
  private IntakeOpeningSubsystem intakeOpeningSubsystem;
  private Supplier<Double> intakeFwdAxis, intakeRevAxis;
  /** Creates a new IntakeOpeningCommand. */
  public IntakeOpeningCommand(IntakeOpeningSubsystem intakeOpeningSubsystem, Supplier<Double> intakeFwdAxis, Supplier<Double> intakeRevAxis) {
    this.intakeOpeningSubsystem = intakeOpeningSubsystem;
    this.intakeFwdAxis = intakeFwdAxis;
    this.intakeRevAxis = intakeRevAxis;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(this.intakeOpeningSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double intakeFwdSpeed = this.intakeFwdAxis.get();
    double intakeRevSpeed = this.intakeRevAxis.get();

    if (intakeFwdSpeed > IntakeOpeningConstants.deadband) {
      this.intakeOpeningSubsystem.setIntakeOpeningSpeed(-1 * intakeFwdSpeed);
    } else if (intakeRevSpeed > IntakeOpeningConstants.deadband) {
      this.intakeOpeningSubsystem.setIntakeOpeningSpeed(intakeRevSpeed);
    } else {
      this.intakeOpeningSubsystem.setIntakeOpeningSpeed(0.0);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
