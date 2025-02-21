// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.mechanisms.Scocer;
import frc.robot.util.CustomAutoBuilder;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class L1Auton extends SequentialCommandGroup {
  /** Creates a new L1Auton. */
  public L1Auton(Drive drive, Scocer scocer) {
    Command[] drivePaths = CustomAutoBuilder.getDrivePaths();
    Command autonPath = Commands.sequence(drivePaths[0]);
    for (int i = 1; i < drivePaths.length; i++) {
      if (i % 2 == 0) {
        autonPath =
            Commands.sequence(
                autonPath,
                Commands.deadline(
                    new WaitCommand(1),
                    Commands.run(() -> scocer.setPower(0.3), scocer)
                        .finallyDo(() -> scocer.setPower(0.0))));
      } else {
        autonPath = Commands.sequence(autonPath, drivePaths[i], new WaitCommand(2));
      }
    }
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(autonPath);
  }
}
