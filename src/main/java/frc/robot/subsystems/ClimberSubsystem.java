// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.CounterBase.EncodingType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ClimberSubsystem extends SubsystemBase {
  public static final String baseEncoder = null;
  private WPI_TalonSRX left_motor;
  private WPI_TalonSRX right_motor;
  public static Encoder leftEncoder;

  /** Creates a new ClimberSubsystem. */
  public ClimberSubsystem() {
    this.left_motor = new WPI_TalonSRX(Constants.left_ID);
    this.right_motor = new WPI_TalonSRX(Constants.right_ID);
    ClimberSubsystem.leftEncoder = new Encoder(Constants.left_encoder_port, Constants.second_encoder_port, false,
        EncodingType.k4X);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void moveRightMotor(double rotationSpeed) {
    if (rotationSpeed < Constants.deadband_speed) {
      this.right_motor.setNeutralMode(NeutralMode.Brake);
    } else {
      this.right_motor.setNeutralMode(NeutralMode.Coast);
      this.right_motor.set(rotationSpeed * Constants.climber_speed_limiter);
    }
  }

  public void moveLeftMotor(boolean anticlockwiseMotion) {
    ClimberSubsystem.leftEncoder.reset();
    this.left_motor.setNeutralMode(NeutralMode.Coast);
    double finalDistance = (anticlockwiseMotion ? 1 : -1) * Constants.left_motor_movement_metric;
    while ((Math.abs(finalDistance)
        - Math.abs(ClimberSubsystem.leftEncoder.getDistance())) < Constants.deadband_motion) {
      this.left_motor.set(Math.signum(finalDistance) * Constants.left_motor_movement_kp
          * (Math.abs(finalDistance) - Math.abs(ClimberSubsystem.leftEncoder.getDistance())));
    }

    this.left_motor.setNeutralMode(NeutralMode.Brake);
  }

  public void setMotor(double move) {
    this.left_motor.set(move);
  }

public double getEncoderMeters() {
    return 0;
}
}
