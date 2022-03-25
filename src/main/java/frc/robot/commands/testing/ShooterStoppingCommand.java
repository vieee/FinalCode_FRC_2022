// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.testing;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.ShooterSubsystem;

public class ShooterStoppingCommand extends CommandBase {
  private final ShooterSubsystem shooterSubsystem;
  /** Creates a new ShooterStoppingCommand. */
  public ShooterStoppingCommand(ShooterSubsystem shooterSubsystem) {
    this.shooterSubsystem = shooterSubsystem;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(this.shooterSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    this.shooterSubsystem.stopShooter();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return !RobotContainer.joyD.getRawButton(OIConstants.shootStopping_B_ButtonNumber);
  }
}
