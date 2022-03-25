// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
// import com.revrobotics.RelativeEncoder;
// import com.revrobotics.SparkMaxRelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.Constants.DrivingConstants;

public class DriveSubsystem extends SubsystemBase {
  private final CANSparkMax FR;
  private final CANSparkMax BR;
  // public RelativeEncoder FR_encoder;
  // public RelativeEncoder BR_encoder;
  private final MotorControllerGroup rightSide;

  private final CANSparkMax FL;
  private final CANSparkMax BL;
  //public RelativeEncoder FL_encoder;
  //public RelativeEncoder BL_encoder;
  private final MotorControllerGroup leftSide;

  private final DifferentialDrive driveTrain;

  /** Creates a new DriveSubsystem. */
  public DriveSubsystem() {
    this.FR = new CANSparkMax(DrivingConstants.FR_ID, MotorType.kBrushed);
    this.BR = new CANSparkMax(DrivingConstants.BR_ID, MotorType.kBrushed);
    //this.FR_encoder = this.FR.getEncoder(SparkMaxRelativeEncoder.Type.kHallSensor,
        //DrivingConstants.neoCountsPerRevolution);
    //this.BR_encoder = this.BR.getEncoder(SparkMaxRelativeEncoder.Type.kHallSensor,
        //DrivingConstants.neoCountsPerRevolution);
    this.rightSide = new MotorControllerGroup(this.FR, this.BR);

    this.FL = new CANSparkMax(DrivingConstants.FL_ID, MotorType.kBrushed);
    this.BL = new CANSparkMax(DrivingConstants.BL_ID, MotorType.kBrushed);
    //this.FL_encoder = this.FL.getEncoder(SparkMaxRelativeEncoder.Type.kHallSensor,
        //DrivingConstants.neoCountsPerRevolution);
    //this.BL_encoder = this.BL.getEncoder(SparkMaxRelativeEncoder.Type.kHallSensor,
        //DrivingConstants.neoCountsPerRevolution);
    this.leftSide = new MotorControllerGroup(this.FL, this.BL);

    this.driveTrain = new DifferentialDrive(this.leftSide, this.rightSide);

    // Set Encoders to 0 initially
    // this.FR_encoder.setPosition(0.0);
    // this.BR_encoder.setPosition(0.0);
    // this.FL_encoder.setPosition(0.0);
    // this.BL_encoder.setPosition(0.0);

    // Manually Invert the Right Side
    // this.rightSide.setInverted(true);
    // this.leftSide.setInverted(false);

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void drive(final double l, final double r) {
    this.rightSide.setInverted(true);

    this.FR.set(r);
    this.BR.set(r);
    this.FL.set(l);
    this.BL.set(l);
  }

  public void arcadeInbuilt(final double y, final double z) {
    // System.out.println("Speed: " + y + " " + z);
    this.rightSide.setInverted(false);

    this.driveTrain.arcadeDrive(y * DrivingConstants.maxSpeed, z * DrivingConstants.maxTurnSpeed);
  }

  double integralEncoder = 0.0;
  double previousErrorEncoder = 0.0;

 /* public void drivePIDEncoder(double yaxis) {

    // Calculate Error
    double leftRate = (this.FL_encoder.getPosition() + this.BL_encoder.getPosition()) / 2; // RobotContainer.enc_L.getRate();
    double rightRate = (this.FR_encoder.getPosition() + this.BR_encoder.getPosition()) / 2; // RobotContainer.enc_R.getRate();
    double error = leftRate + rightRate;
    SmartDashboard.putNumber("left rate", leftRate);
    SmartDashboard.putNumber("right rate", rightRate);
    // double navxYawRate = RobotContainer.navx.getRate();
    // double navxYaw = RobotContainer.navx.getYaw();
    // double navxAngle = RobotContainer.navx.getAngle();
    // SmartDashboard.putNumber("YAW RATE", navxYawRate);
    // SmartDashboard.putNumber("YAW", navxYaw);
    // SmartDashboard.putNumber("ANGLE", navxAngle);

    // Calculate Correction
    integralEncoder += error;
    if (yaxis < DrivingConstants.integralResetBound)
      integralEncoder = 0;

    double derivative = error - previousErrorEncoder;
    previousErrorEncoder = error;

    double correction = ((error * DrivingConstants.kPEncoder) + (integralEncoder * DrivingConstants.kIEncoder)
        + (derivative * DrivingConstants.kDEncoder));

    // Apply the Correction to Motor Voltages
    double leftSpeed = yaxis - correction;
    double rightSpeed = yaxis + correction;

    // System.out.println("leftSpeed: " + leftSpeed + " rightSpeed: " + rightSpeed);
    SmartDashboard.putNumber("Left PID", leftSpeed);
    SmartDashboard.putNumber("Right PID", rightSpeed);

    drive(leftSpeed * DrivingConstants.maxSpeed, rightSpeed * DrivingConstants.maxSpeed);

  }*/

  double integralNavX = 0;
  double previousErrorNavX = 0;

  public void drivePIDNavX(double yaxis) {

    // Calculate Error
    double navxYawRate = RobotContainer.navx.getRate();
    double error = navxYawRate;

    // Calculate Correction
    integralNavX += error;
    if (yaxis < DrivingConstants.integralResetBound)
      integralNavX = 0;

    double derivative = error - previousErrorNavX;
    previousErrorNavX = error;

    double correction = ((error * DrivingConstants.kPNavX) + (integralEncoder * DrivingConstants.kINavX)
        + (derivative * DrivingConstants.kDNavX));

    // Apply Correction to Motor Voltages
    double leftSpeed = yaxis - correction;
    double rightSpeed = yaxis + correction;

    drive(leftSpeed * DrivingConstants.maxSpeed, rightSpeed * DrivingConstants.maxSpeed);

  }

  /*public double getDistanceTravelled() {
    double totalDistance = ((Math.abs(FL_encoder.getPosition()) + Math.abs(BL_encoder.getPosition()) + Math.abs(FR_encoder.getPosition()) + Math.abs(BR_encoder.getPosition())) / DrivingConstants.neoCountsPerRevolution) * DrivingConstants.neoMotorWheelCircumference * DrivingConstants.motorGearBoxRatio; // RobotContainer.enc_L.getDistance();
    return ((totalDistance) / 4);
  }*/


}