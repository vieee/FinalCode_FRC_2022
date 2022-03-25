// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

// import edu.wpi.first.wpilibj.DoubleSolenoid;
// import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;

public class IntakeSubsystem extends SubsystemBase {
  private final CANSparkMax intake;
  // private final DoubleSolenoid poser;
  // private final CANSparkMax feeder;

  /** Creates a new IntakeSubsystem. */
  public IntakeSubsystem() {
    this.intake = new CANSparkMax(IntakeConstants.intake_ID, MotorType.kBrushless);
    // this.poser = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, IntakeConstants.poser_FID, IntakeConstants.poser_BID);
    //  this.feeder = new CANSparkMax(IntakeConstants.feeder_ID, MotorType.kBrushless);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  // 1 - forward, 2 - reverse, 3 - off
  // public void setMode(int mode) {
  //   switch (mode) {
  //     case 1:
  //       this.poser.set(DoubleSolenoid.Value.kForward);
  //       break;
  //     case 2:
  //       this.poser.set(DoubleSolenoid.Value.kReverse);
  //       break;
  //     case 3:
  //       this.poser.set(DoubleSolenoid.Value.kOff);
  //       break;
  //     default:
  //       this.poser.set(DoubleSolenoid.Value.kOff);
  //       break;
  //   }
  // }

  public void setIntakeSpeed(double speed) {
    double speedmotor = this.intake.get();
    if (Math.abs(speedmotor) < IntakeConstants.deadband) {
      double now = Timer.getFPGATimestamp();
      while (now < 0.6) {
        if (now == 0.2) this.intake.set(speed / 3);
        if (now == 0.4) this.intake.set(speed / 2);
      }
      this.intake.set(speed);
      // System.out.println("Timer : " + now);
    }
    else this.intake.set(0);
  }

  /*public void setFeederSpeed() {
    double speedmotor = this.feeder.get();
    if (Math.abs(speedmotor) < IntakeConstants.deadband) {
      double now = Timer.getFPGATimestamp();
      while (now < 0.6) {
        if (now == 0.2) this.feeder.set(IntakeConstants.feederSpeed / 3);
        if (now == 0.4) this.feeder.set(IntakeConstants.feederSpeed / 2);
      }
      this.feeder.set(IntakeConstants.feederSpeed);
      // System.out.println("Timer : " + now);
    }
    else this.feeder.set(0);
  }*/

  public void stopSystem() {
    this.intake.set(0);
    // this.poser.set(DoubleSolenoid.Value.kReverse);
  }
}
