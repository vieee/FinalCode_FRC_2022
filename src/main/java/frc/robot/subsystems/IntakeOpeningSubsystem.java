// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeOpeningConstants;

public class IntakeOpeningSubsystem extends SubsystemBase {
  private WPI_TalonSRX intakePositonSwitching;
  /** Creates a new IntakeOpeningSubsystem. */
  public IntakeOpeningSubsystem() {
    this.intakePositonSwitching = new WPI_TalonSRX(IntakeOpeningConstants.intakeOpening_ID);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void setIntakeOpeningSpeed(double speed) {
    this.intakePositonSwitching.set(TalonSRXControlMode.PercentOutput, speed);
  }
}
