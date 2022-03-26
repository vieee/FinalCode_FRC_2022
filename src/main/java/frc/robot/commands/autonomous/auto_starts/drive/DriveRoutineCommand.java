package frc.robot.commands.autonomous.auto_start.drive;

import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.DriveSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class DriveRoutineCommand extends ParallelRaceGroup {
  /** Creates a new DriveRoutineCommand. */
  public DriveRoutineCommand(DriveSubsystem driveSubsystem, double time) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(new DriveAutonomousCommand(driveSubsystem));

    addCommands(new WaitCommand(time));
  }
}