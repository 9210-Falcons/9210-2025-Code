// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.mechanisms.Flipper;
import frc.robot.subsystems.mechanisms.Scorer;
import frc.robot.util.CustomAutoBuilder;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class L1Auton extends SequentialCommandGroup {
  /** Creates a new L1Auton. */
  public L1Auton(Drive drive, Scorer scocer, Flipper flipper) {
    Command[] drivePaths = CustomAutoBuilder.getDrivePaths();
    Command autonPath =
        Commands.sequence(
            drivePaths[0],
            Commands.deadline(
                new WaitCommand(0.5),
                Commands.run(() -> scocer.setPower(0.6), scocer)
                    .finallyDo(() -> scocer.setPower(0.0))));
    for (int i = 1; i < drivePaths.length; i++) {
      if (i % 2 == 1) {
        autonPath = Commands.sequence(autonPath, drivePaths[i], new WaitCommand(0.5));
      } else {
        autonPath =
            Commands.sequence(
                autonPath,
                drivePaths[i],
                Commands.deadline(
                    new WaitCommand(0.5),
                    Commands.run(() -> scocer.setPower(0.6), scocer)
                        .finallyDo(() -> scocer.setPower(0.0))));
      }
    }
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(Commands.parallel(autonPath, Commands.run(() -> flipper.L1Scoring(), flipper)));
  }
}
