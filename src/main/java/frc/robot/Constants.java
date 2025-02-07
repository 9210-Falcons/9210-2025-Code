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

// hi
package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.RobotBase;

/**
 * This class defines the runtime mode used by AdvantageKit. The mode is always "real" when running
 * on a roboRIO. Change the value of "simMode" to switch between "sim" (physics sim) and "replay"
 * (log replay from a file).
 */
public final class Constants {
  public static final Mode simMode = Mode.SIM;
  public static final Mode currentMode = RobotBase.isReal() ? Mode.REAL : simMode;

  public static enum Mode {
    /** Running on a real robot. */
    REAL,

    /** Running a physics simulator. */
    SIM,

    /** Replaying from a log file. */
    REPLAY
  }

  public static class AutonConstants {
    public static final Rotation2d START_ROTATION = Rotation2d.fromDegrees(0); // 180

    public static final Pose2d START_LEFT = new Pose2d(8.0, 7.29, START_ROTATION);
    public static final Pose2d START_CENTER = new Pose2d(8.0, 6.20, START_ROTATION);
    public static final Pose2d START_RIGHT = new Pose2d(8.0, 5.13, START_ROTATION);

    public static final Pose2d AB = new Pose2d(3.0, 4.0, Rotation2d.fromDegrees(0));
    public static final Pose2d CD = new Pose2d(3.7, 2.7, Rotation2d.fromDegrees(60));
    public static final Pose2d EF = new Pose2d(5.2, 2.7, Rotation2d.fromDegrees(120));
    public static final Pose2d GH = new Pose2d(6.0, 4.0, Rotation2d.fromDegrees(180));
    public static final Pose2d IJ = new Pose2d(5.3, 5.3, Rotation2d.fromDegrees(-120));
    public static final Pose2d KL = new Pose2d(3.8, 5.3, Rotation2d.fromDegrees(-60));

    public static final Pose2d R1 = new Pose2d(1.5, 6.6, Rotation2d.fromDegrees(-60));
    public static final Pose2d R0 = new Pose2d(1.5, 1.4, Rotation2d.fromDegrees(60));

    public static final double MAX_VELOCITY = 1;
    public static final double MAX_ACCELERATION = 0.75;
  }
}
