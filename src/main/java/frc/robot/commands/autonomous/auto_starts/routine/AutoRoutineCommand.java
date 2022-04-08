package frc.robot.commands.autonomous.auto_starts.routine;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
// import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.autonomous.auto_starts.drive.DriveRoutineCommand;
import frc.robot.commands.autonomous.auto_starts.intake.IntakeRoutineGroupCommand;
import frc.robot.commands.autonomous.auto_starts.shooter.ShooterRoutineGroupCommand;
import frc.robot.commands.autonomous.auto_starts.start.ServoStartGroupCommand;
import frc.robot.commands.autonomous.auto_starts.turn.TurnAutonomousCommand;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeOpeningSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AutoRoutineCommand extends SequentialCommandGroup {
  /** Creates a new AutoRoutineCommand. */
  public AutoRoutineCommand(IntakeOpeningSubsystem intakeOpeningSubsystem, DriveSubsystem driveSubsystem,
      IntakeSubsystem intakeSubsystem, ShooterSubsystem shooterSubsystem) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(new ServoStartGroupCommand(intakeOpeningSubsystem, 1));

    addCommands(new DriveRoutineCommand(driveSubsystem, 2));

    addCommands(new IntakeRoutineGroupCommand(intakeSubsystem, 2));

    addCommands(new TurnAutonomousCommand(driveSubsystem, 130));

    addCommands(new ShooterRoutineGroupCommand(shooterSubsystem, 2));
  }
}