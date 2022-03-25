// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.testing;

import java.util.function.Supplier;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ClimberSubsystem;

public class ClimberTestingCommand extends CommandBase {
  private final ClimberSubsystem climberSubsystem;
  private final PIDController pidController;

  /** Creates a new ClimberTestingCommand. */
  public ClimberTestingCommand(ClimberSubsystem climberSubsystem, Supplier<Double> setPoint) {
    this.climberSubsystem = climberSubsystem;
    this.pidController = new PIDController(0.001, 0.0001, 0.01);
    this.pidController.setSetpoint(5000);
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(this.climberSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    System.out.println("ElevatorPIDCmd started!");
    this.pidController.reset();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double speed = pidController.calculate(this.climberSubsystem.getEncoderMeters());
    this.climberSubsystem.setMotor(speed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    this.climberSubsystem.setMotor(0);
    System.out.println("ElevatorPIDCmd ended!");
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
