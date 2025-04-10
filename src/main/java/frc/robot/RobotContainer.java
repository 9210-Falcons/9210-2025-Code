// Copyright 2021-2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.commands.AutoAlign;
import frc.robot.commands.DriveCommands;
import frc.robot.commands.L1Auton;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.Vision;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIOPigeon2;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.drive.ModuleIOTalonFX;
import frc.robot.subsystems.mechanisms.Flipper;
import frc.robot.subsystems.mechanisms.Scorer;
import frc.robot.util.CustomAutoBuilder;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // Subsystems
  private final Drive drive;
  public final Scorer scocer;
  private final Flipper flipper;
  private final Vision vision;
  // Controller
  //   private final CommandXboxController controller = new CommandXboxController(0);
  private final CommandPS5Controller controller = new CommandPS5Controller(0);
  private final CommandPS5Controller controller2 = new CommandPS5Controller(1);
  // Dashboard inputs
  private final SendableChooser<Command> autoChooser;

  // The precent drive speed will be while slowed
  //   private final double slowedDrivePercent = 0.3;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    switch (Constants.currentMode) {
      case REAL:
        // Real robot, instantiate hardware IO implementations
        drive =
            new Drive(
                new GyroIOPigeon2(),
                new ModuleIOTalonFX(TunerConstants.FrontLeft),
                new ModuleIOTalonFX(TunerConstants.FrontRight),
                new ModuleIOTalonFX(TunerConstants.BackLeft),
                new ModuleIOTalonFX(TunerConstants.BackRight));
        break;

      case SIM:
        // Sim robot, instantiate physics sim IO implementations
        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIOSim(TunerConstants.FrontLeft),
                new ModuleIOSim(TunerConstants.FrontRight),
                new ModuleIOSim(TunerConstants.BackLeft),
                new ModuleIOSim(TunerConstants.BackRight));
        break;

      default:
        // Replayed robot, disable IO implementations
        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {});
        break;
    }
    scocer = new Scorer();
    flipper = new Flipper();
    vision = new Vision(drive);

    // Set up auto routines
    autoChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("Auto Choices", autoChooser);

    // Set up SysId routines
    autoChooser.addOption(
        "Drive Wheel Radius Characterization", DriveCommands.wheelRadiusCharacterization(drive));
    autoChooser.addOption(
        "Drive Simple FF Characterization", DriveCommands.feedforwardCharacterization(drive));
    autoChooser.addOption(
        "Drive SysId (Quasistatic Forward)",
        drive.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "Drive SysId (Quasistatic Reverse)",
        drive.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
    autoChooser.addOption(
        "Drive SysId (Dynamic Forward)", drive.sysIdDynamic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "Drive SysId (Dynamic Reverse)", drive.sysIdDynamic(SysIdRoutine.Direction.kReverse));

    // Configure the button bindings
    configureButtonBindings();
  }

  SlewRateLimiter xSlewRate = new SlewRateLimiter(1.0);
  SlewRateLimiter ySlewRate = new SlewRateLimiter(1.0);
  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    // Default command, normal field-relative drive
    drive.setDefaultCommand(
        DriveCommands.joystickDrive(
            drive,
            () -> -ySlewRate.calculate(controller.getLeftY()),
            () -> -xSlewRate.calculate(controller.getLeftX()),
            () -> -controller.getRightX()));

    controller.R2().whileTrue(new AutoAlign(drive, () -> false));
    // Lock to 0° when A button is held
    // controller
    //    .a()
    //    .whileTrue(
    //        DriveCommands.joystickDriveAtAngle(
    //            drive,
    //            () -> -controller.getLeftY(),
    //            () -> -controller.getLeftX(),
    //            () -> new Rotation2d()));
    controller2
        .cross()
        .whileTrue(
            Commands.run(() -> scocer.setPower(-0.3), scocer)
                .finallyDo(() -> scocer.setPower(0.0)));
    controller.L1().onTrue(Commands.runOnce(() -> DriveCommands.slowSpeed = false));
    controller.R1().onTrue(Commands.runOnce(() -> DriveCommands.slowSpeed = true));
    // controller.b().onTrue(Commands.runOnce(() -> DriveCommands.slowSpeed = false));
    // Full speed is B
    // controller
    //    .x()
    //    .onTrue(Commands.runOnce(() -> DriveCommands.slowSpeed = true)); // Slowed speed is X
    controller2
        .triangle()
        .whileTrue(
            Commands.run(() -> scocer.setPower(0.6), scocer).finallyDo(() -> scocer.setPower(0.0)));
    // Switch to X pattern when X button is pressed
    controller.square().onTrue(Commands.runOnce(drive::stopWithX, drive));

    controller
        .options()
        .onTrue(
            Commands.runOnce(
                    () ->
                        drive.setPose(
                            new Pose2d(drive.getPose().getTranslation(), new Rotation2d())))
                .ignoringDisable(true));

    // Reset gyro to 0° when B button is pressed
    controller
        .circle()
        .onTrue(
            Commands.runOnce(
                    () ->
                        drive.setPose(
                            new Pose2d(drive.getPose().getTranslation(), new Rotation2d())),
                    drive)
                .ignoringDisable(true));

    controller2.L2().onTrue(Commands.run(() -> flipper.L1Scoring(), flipper));
    controller2.R2().onTrue(Commands.run(() -> flipper.L2Scoring(), flipper));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    drive.setPose(CustomAutoBuilder.getStartPose2d());
    if (CustomAutoBuilder.getStartPose2d().getY() == 4.0) {
      try {
        return Commands.sequence(
            AutoBuilder.followPath(PathPlannerPath.fromPathFile("Middle Auton")),
            Commands.deadline(
                new WaitCommand(0.75),
                Commands.run(() -> scocer.setPower(0.6), scocer)
                    .finallyDo(() -> scocer.setPower(0.0))));
      } catch (Exception e) {
      }
    }
    return new L1Auton(drive, scocer, flipper);
    // return autoChooser.getSelected();
  }
}
