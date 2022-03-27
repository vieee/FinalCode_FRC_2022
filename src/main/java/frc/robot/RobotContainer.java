// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.Constants.OIConstants;
// import frc.robot.Constants.VisionConstants;
import frc.robot.commands.DriveCommand;
// import frc.robot.commands.FeederRotateCommand;
import frc.robot.commands.IntakeForwardCommand;
import frc.robot.commands.IntakeReverseCommand;
// import frc.robot.commands.ShooterAnticlockwiseCommand;
import frc.robot.commands.ShooterClockwiseCommand;
import frc.robot.commands.testing.ShooterStoppingCommand;
// import frc.robot.commands.testing.ShooterTestingCommand;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

// import edu.wpi.first.networktables.NetworkTable;
// import edu.wpi.first.networktables.NetworkTableEntry;
// import edu.wpi.first.networktables.NetworkTableInstance;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final DriveSubsystem driveSubsystem;
  private final ShooterSubsystem shooterSubsystem;
  private final IntakeSubsystem intakeSubsystem;
  
  // IO Devices
  public final static Joystick joyD = new Joystick(OIConstants.kDriverJoystickPort);
  public final static AHRS navx = new AHRS(SPI.Port.kMXP);
  // static NetworkTable table = NetworkTableInstance.getDefault().getTable(VisionConstants.limelight);
  // static NetworkTableEntry tv = table.getEntry(VisionConstants.tv);
  // static NetworkTableEntry tx = table.getEntry(VisionConstants.tx);
  // NetworkTableEntry ty = table.getEntry(VisionConstants.ty);
  // NetworkTableEntry ta = table.getEntry(VisionConstants.ta);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    this.driveSubsystem = new DriveSubsystem();
    this.shooterSubsystem = new ShooterSubsystem();
    this.intakeSubsystem = new IntakeSubsystem();
    // Configure the button bindings
    configureButtonBindings();

    this.driveSubsystem.setDefaultCommand(new DriveCommand(this.driveSubsystem, //
        () -> joyD.getRawAxis(OIConstants.kArcadeDriveSpeedAxis),
        () -> -1 * joyD.getRawAxis(OIConstants.kArcadeDriveTurnAxis))//
    );

    this.intakeSubsystem.setDefaultCommand(new IntakeForwardCommand(this.intakeSubsystem));
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    // Shooter Clockwise
    new JoystickButton(joyD, OIConstants.shoot_RB_ButtonNumber)
        .whenActive(new ShooterClockwiseCommand(this.shooterSubsystem));

    // Shooter Anticlockwise
    new JoystickButton(joyD, OIConstants.shootTesting_LB_ButtonNumber)
        .whenPressed(new ShooterStoppingCommand(this.shooterSubsystem));

    // Feeder Rotation
    // new JoystickButton(joyD, OIConstants.feeder_X_ButtonNumber)
    //     .whenActive(new FeederRotateCommand(this.intakeSubsystem));

    // Intake Position Switch 
    new JoystickButton(joyD, OIConstants.intaker_Y_ButtonNumber)
        .toggleWhenPressed(new IntakeReverseCommand(this.intakeSubsystem));

    
  }

  // public static boolean getTarget() {
  //   return tv.getBoolean(false);
  // }

  // public double getDistanceToGoal() {
  //   return ty.getDouble(VisionConstants.defaultValue);
  // }

  // public static double getDistanceGyroSeparationFromGoal() {
  //   return tx.getDouble(VisionConstants.defaultValue);
  // }
  
  // public double getAreaOfGoal() {
  //   return ta.getDouble(VisionConstants.defaultAreaValue);
  // }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    return new AutoRoutineCommand(this.driveSubsystem, this.intakeSubsystem, this.shooterSubsystem);
  }
}