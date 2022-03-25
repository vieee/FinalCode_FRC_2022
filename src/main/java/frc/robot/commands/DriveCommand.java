// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.Supplier;

// import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.DrivingConstants;
import frc.robot.subsystems.DriveSubsystem;

public class DriveCommand extends CommandBase {
  private final DriveSubsystem driveSubsystem;
  private Supplier<Double> speedAxis, turnAxis;
  // private final SlewRateLimiter speedLimiter, turnLimiter;

  /** Creates a new DriveCommand. */
  public DriveCommand(DriveSubsystem driveSubsystem, Supplier<Double> speedAxis, Supplier<Double> turnAxis) {
    this.driveSubsystem = driveSubsystem;
    this.speedAxis = speedAxis;
    this.turnAxis = turnAxis;
    // this.speedLimiter = new SlewRateLimiter(DrivingConstants.kTeleDriveMaxAccelerationUnitsPerSecond);
    // this.turnLimiter = new SlewRateLimiter(DrivingConstants.kTeleDriveMaxAccelerationUnitsPerSecond);
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(driveSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // get the speeds
    double realTimeSpeed = -1 * speedAxis.get();
    double realTimeTurn = turnAxis.get();
    
    // deadband consideration
    // realTimeSpeed = (Math.abs(realTimeSpeed) < DrivingConstants.motionStopThreshold) ? 0 : realTimeSpeed;
    // realTimeTurn = (Math.abs(realTimeTurn) < DrivingConstants.motionStopThreshold) ? 0 : realTimeTurn;

    // slew rate limiter
    // realTimeSpeed = this.speedLimiter.calculate(realTimeSpeed) * DrivingConstants.kTeleDriveMaxSpeedMetersPerSecond;
    // realTimeTurn = this.turnLimiter.calculate(realTimeTurn) * DrivingConstants.kTeleDriveMaxSpeedMetersPerSecond;
    // System.out.println(realTimeSpeed + "+" + realTimeTurn);
    if (Math.abs(realTimeTurn) > DrivingConstants.turnThreshold) {
      driveSubsystem.arcadeInbuilt(realTimeSpeed, realTimeTurn);
    } else {
      if (Math.abs(realTimeSpeed) > DrivingConstants.motionStopThreshold)
        driveSubsystem.arcadeInbuilt(realTimeSpeed, realTimeTurn);
      else
        driveSubsystem.drive(0, 0);
    }
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