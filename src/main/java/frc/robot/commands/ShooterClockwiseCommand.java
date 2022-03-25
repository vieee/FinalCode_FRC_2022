// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.Constants.OIConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.ShooterSubsystem;

public class ShooterClockwiseCommand extends CommandBase {
  private final ShooterSubsystem shooterSubsystem;

  /** Creates a new ShooterCommand. */
  public ShooterClockwiseCommand(ShooterSubsystem shooterSubsystem) {
    this.shooterSubsystem = shooterSubsystem;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(shooterSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // System.out.println("Shooter AntiClockwise Started...");
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    SmartDashboard.putNumber("VELO", ShooterSubsystem.shootEncoder.getVelocity());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    this.shooterSubsystem.setSpeed(ShooterConstants.frontRollSpeed);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return !RobotContainer.joyD.getRawButton(OIConstants.shoot_RB_ButtonNumber);
  }
}
