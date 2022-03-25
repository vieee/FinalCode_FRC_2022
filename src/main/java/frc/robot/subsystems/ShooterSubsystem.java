// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.SparkMaxRelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import frc.robot.Constants.ShooterConstants;
// import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ShooterSubsystem extends SubsystemBase {
  private final CANSparkMax shoot;
  private final SparkMaxPIDController shootPIDController;
  public static RelativeEncoder shootEncoder;

  /** Creates a new ShooterSubsystem. */
  public ShooterSubsystem() {
    this.shoot = new CANSparkMax(ShooterConstants.shoot_ID, MotorType.kBrushless);
    this.shoot.restoreFactoryDefaults();
    this.shootPIDController = this.shoot.getPIDController();
    ShooterSubsystem.shootEncoder = this.shoot.getEncoder(SparkMaxRelativeEncoder.Type.kHallSensor,
        ShooterConstants.neoCountsPerRevolution);

    this.shootPIDController.setP(ShooterConstants.kP);
    this.shootPIDController.setI(ShooterConstants.kI);
    this.shootPIDController.setD(ShooterConstants.kD);
    this.shootPIDController.setIZone(ShooterConstants.kIz);
    this.shootPIDController.setFF(ShooterConstants.kFF);
    this.shootPIDController.setOutputRange(ShooterConstants.kMinOutput, ShooterConstants.kMaxOutput);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void setSpeed(double speed) {
    double speedmotor = shootEncoder.getVelocity();
    if (Math.abs(speedmotor) < ShooterConstants.deadbandVelocity) {
      // double now = Timer.getFPGATimestamp();
      // while (now < 0.6) {
      // if (now == 0.2)
      // this.shoot.set(speed / 3);
      // if (now == 0.4)
      // this.shoot.set(speed / 2);
      // }
      // this.shoot.set(speed);
      this.shootPIDController.setReference(3500, CANSparkMax.ControlType.kVelocity);
      // SmartDashboard.putNumber("POSITION",
      // ShooterSubsystem.shootEncoder.getPosition());
      SmartDashboard.putNumber("VELOCITY", ShooterSubsystem.shootEncoder.getPosition());
      // SmartDashboard.putNumber("POSITION FACTOR",
      // ShooterSubsystem.shootEncoder.getPositionConversionFactor());
      // SmartDashboard.putNumber("CPR",
      // ShooterSubsystem.shootEncoder.getCountsPerRevolution());
      // System.out.println("Timer : " + now);
    } else
      this.shoot.set(0);
  }

  public void shootReference(double rpm) {
    SmartDashboard.putNumber("RPM", rpm);
    this.shootPIDController.setReference(rpm, CANSparkMax.ControlType.kVelocity);
  }

  public void stopShooter() {
    this.shoot.set(0);
  }
}
