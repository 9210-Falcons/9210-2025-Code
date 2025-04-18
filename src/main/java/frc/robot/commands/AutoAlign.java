// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.LimelightHelpers;
import frc.robot.subsystems.drive.Drive;
import java.util.function.Supplier;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class AutoAlign extends Command {
  /** Creates a new AutoAlign. */
  PIDController streafPID = new PIDController(0.2, 0, 0.000);

  PIDController forwardPID = new PIDController(0.2, 0, 0.000);
  Drive m_Drive;

  public AutoAlign(Drive m_Drive, Supplier<Boolean> branch) {
    // Use addRequirements() here to declare subsystem dependencies.
    LimelightHelpers.setFiducial3DOffset("", branch.get() ? 0.5 : -0.5, 0, 0);
    this.m_Drive = m_Drive;
    addRequirements(m_Drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    SmartDashboard.putNumber("Targetx", LimelightHelpers.getTX(""));
    SmartDashboard.putNumber("PID output", streafPID.calculate(LimelightHelpers.getTX(""), 0));
    DriveCommands.robotDrive(
            m_Drive,
            () -> forwardPID.calculate(LimelightHelpers.getTA(""), 3.5),
            () -> streafPID.calculate(LimelightHelpers.getTX(""), 0),
            () -> 0)
        .execute();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
