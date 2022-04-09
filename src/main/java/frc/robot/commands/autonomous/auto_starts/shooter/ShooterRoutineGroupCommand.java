// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autonomous.auto_starts.shooter;

import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.autonomous.auto_starts.feeder.FeederRoutineGroupCommand;
import frc.robot.subsystems.FeederSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ShooterRoutineGroupCommand extends ParallelRaceGroup {
  /** Creates a new ShooterRoutineGroupCommand. */
  public ShooterRoutineGroupCommand(ShooterSubsystem shooterSubsystem, FeederSubsystem feederSubsystem, double time, double feederTimeDelay) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(new ShooterAutonomousCommand(shooterSubsystem));

    addCommands(new FeederRoutineGroupCommand(feederSubsystem, feederTimeDelay));
 
    addCommands(new WaitCommand(time));
  }
}