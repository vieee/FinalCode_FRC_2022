// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autonomous;

// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
// import edu.wpi.first.wpilibj2.command.CommandBase;
// import frc.robot.Constants.DrivingConstants;
// import frc.robot.subsystems.DriveSubsystem;

/*public class DriveByDistanceCommand extends CommandBase {
  private final DriveSubsystem driveSubsystem;
  private final double distance;
  private final boolean driveForward;*/

  // private double distance, initialSpeed, error;
  /** Creates a new DriveByDistanceCommand. */
  /*public DriveByDistanceCommand(DriveSubsystem driveSubsystem, double distance, boolean driveForward) {
    this.driveSubsystem = driveSubsystem;
    this.distance = driveSubsystem.getDistanceTravelled() + distance;
    this.driveForward = driveForward;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(this.driveSubsystem);
  }*/

  // Called when the command is initially scheduled.
  /*@Override
  public void initialize() {
    SmartDashboard.putNumber("START DRIVE AUTO", driveSubsystem.getDistanceTravelled());
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    SmartDashboard.putNumber("REAL AUTO DRIVE: ", this.driveSubsystem.getDistanceTravelled());
    this.driveSubsystem.drive((this.driveForward ? 1 : -1) * DrivingConstants.kAutoDriveForwardSpeed,
        -1 * (this.driveForward ? 1 : -1) * DrivingConstants.kAutoDriveForwardSpeed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    this.driveSubsystem.drive(0, 0);
    System.out.println("DriveForwardCmd ended!-- " + this.driveSubsystem.getDistanceTravelled());
    // driveSubsystem.setBasePosition();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (this.driveSubsystem.getDistanceTravelled() > this.distance);
  }
}*/
