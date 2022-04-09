// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autonomous.auto_starts.routine;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.commands.autonomous.auto_starts.shooter.ShooterRoutineGroupCommand;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.FeederSubsystem;
import frc.robot.subsystems.IntakeOpeningSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AutoRoutineCompleteGroupCommand extends ParallelCommandGroup {
  /** Creates a new AutoRoutineCompleteGroupCommand. */
  public AutoRoutineCompleteGroupCommand(IntakeOpeningSubsystem intakeOpeningSubsystem, DriveSubsystem driveSubsystem, ShooterSubsystem shooterSubsystem, FeederSubsystem feederSubsystem) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(new ShooterRoutineGroupCommand(shooterSubsystem, feederSubsystem, 15, 2));

    addCommands(new AutoRoutineCommand(intakeOpeningSubsystem, driveSubsystem));
  }
}
