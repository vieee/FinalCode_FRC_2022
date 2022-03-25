// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autonomous;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.ShooterSubsystem;

public class ShootByTimingCommand extends CommandBase {
  private final ShooterSubsystem shooterSubsystem;
  private double time, now;
  /** Creates a new ShootByTimingCommand. */
  public ShootByTimingCommand(ShooterSubsystem shooterSubsystem, double time) {
    this.shooterSubsystem = shooterSubsystem;
    this.time = time;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(this.shooterSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    this.now = Timer.getFPGATimestamp();
    this.shooterSubsystem.shootReference(ShooterConstants.autonomousShooterRPM);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    SmartDashboard.putNumber("execute shooter", this.now);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    this.shooterSubsystem.stopShooter();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (this.now > this.time);
  }
}
