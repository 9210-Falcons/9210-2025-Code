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

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.RobotBase;
import java.util.HashMap;
import java.util.Map;

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
    public static final Rotation2d START_ROTATION = Rotation2d.fromDegrees(180); // 180

    public static final Pose2d START_LEFT = new Pose2d(7.0, 6.0, START_ROTATION);
    public static final Pose2d START_CENTER = new Pose2d(7.0, 4.0, START_ROTATION);
    public static final Pose2d START_RIGHT = new Pose2d(7.0, 2.0, START_ROTATION);

    public static final double LEFT_OFFSET = 0.0; // In Meters
    public static final double RIGHT_OFFSET = -0.0; // In Meters

    public static final Pose2d AB = new Pose2d(3.15, 4.2, Rotation2d.fromDegrees(0));
    public static final Pose2d CD = new Pose2d(3.6, 3.2, Rotation2d.fromDegrees(60));
    public static final Pose2d EF = new Pose2d(5.1, 3.2, Rotation2d.fromDegrees(120));
    public static final Pose2d GH = new Pose2d(5.4, 3.87, Rotation2d.fromDegrees(180));
    public static final Pose2d IJ = new Pose2d(5.1, 4.8, Rotation2d.fromDegrees(-120));
    public static final Pose2d KL = new Pose2d(3.8, 5.3, Rotation2d.fromDegrees(-60));

    public static final Pose2d R1 = new Pose2d(0.6, 7.5, Rotation2d.fromDegrees(-50));
    public static final Pose2d R0 = new Pose2d(0.6, .7, Rotation2d.fromDegrees(50));

    public static final Map<Pose2d, Double> poseAngleMap = new HashMap<>();

    static {
      poseAngleMap.put(AB, 0.0);
      poseAngleMap.put(CD, -60.0);
      poseAngleMap.put(EF, -120.0);
      poseAngleMap.put(GH, -180.0);
      poseAngleMap.put(IJ, -240.0);
      poseAngleMap.put(KL, -300.0);
    }

    public static final double MAX_VELOCITY = 5;
    public static final double MAX_ACCELERATION = 3.2;
  }
}
