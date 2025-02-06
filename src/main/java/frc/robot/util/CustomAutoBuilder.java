// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.IdealStartingState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.Waypoint;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
// import frc.robot.commands.AlignReefAprilTag;
// import frc.robot.subsystems.vision.AprilTag2D;
import java.util.ArrayList;
import java.util.List;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

/** Add your docs here. */
public class CustomAutoBuilder {
  private static final Rotation2d START_ROTATION = Rotation2d.fromDegrees(0); // 180

  private static final Pose2d RIGHT_START = new Pose2d(8.0, 5.13, START_ROTATION);
  private static final Pose2d MIDDLE_START = new Pose2d(8.5, 1.9, START_ROTATION);
  private static final Pose2d LEFT_START = new Pose2d(8.5, 0.8, START_ROTATION);

  private static final Pose2d BACK_RIGHT_SCORE = new Pose2d(5.2, 2.7, Rotation2d.fromDegrees(120));
  private static final Pose2d BACK_LEFT_SCORE =
      new Pose2d(3.7, 2.7, new Rotation2d(Math.toRadians(60)));
  private static final Pose2d LEFT_SCORE = new Pose2d(3.0, 4.0, new Rotation2d(Math.toRadians(0)));
  private static final Pose2d TOP_LEFT_SCORE =
      new Pose2d(3.8, 5.3, new Rotation2d(Math.toRadians(-60)));
  private static final Pose2d TOP_RIGHT_SCORE =
      new Pose2d(5.3, 5.3, new Rotation2d(Math.toRadians(-120)));
  private static final Pose2d RIGHT_SCORE =
      new Pose2d(6.0, 4.0, new Rotation2d(Math.toRadians(180)));

  private static final Pose2d RIGHT_LOAD_STATION =
      new Pose2d(1.5, 6.6, Rotation2d.fromDegrees(-60));
  private static final Pose2d LEFT_LOAD_STATION = new Pose2d(1.5, 1.4, Rotation2d.fromDegrees(60));

  public static LoggedDashboardChooser<Pose2d>[] scoreChoosers;
  public static LoggedDashboardChooser<Pose2d>[] loadStationChoosers;
  public static LoggedDashboardChooser<Pose2d> startChooser;
  public static LoggedDashboardChooser<Integer> displayChooser;

  public static Field2d m_field = new Field2d();
  public static Translation2d[] vertexs = new Translation2d[6];
  public static int NUMBER_OF_CHOOSERS = 1;

  @SuppressWarnings("unchecked")
  public static void chooserBuilder() {
    startChooser = new LoggedDashboardChooser<Pose2d>("Start Position");
    displayChooser = new LoggedDashboardChooser<Integer>("Path Display");

    scoreChoosers = new LoggedDashboardChooser[NUMBER_OF_CHOOSERS];
    loadStationChoosers = new LoggedDashboardChooser[NUMBER_OF_CHOOSERS];

    // Change the number after "i < " to add to the path length. Both number MUST be
    // the same.
    for (int i = 0; i < scoreChoosers.length; i++)
      scoreChoosers[i] = new LoggedDashboardChooser<Pose2d>(String.format("Score Position %s", i));
    for (int i = 0; i < loadStationChoosers.length; i++)
      loadStationChoosers[i] =
          new LoggedDashboardChooser<Pose2d>(String.format("Load Station %s", i));

    for (int i = 0; i < NUMBER_OF_CHOOSERS * 2 - 1; i++) {
      displayChooser.addOption("Path " + i, i);
    }
    displayChooser.addDefaultOption("Path 0", 0);
    startChooser.addOption("Right", RIGHT_START);
    startChooser.addOption("Middle", MIDDLE_START);
    startChooser.addOption("Left", LEFT_START);

    startChooser.addDefaultOption("Right", RIGHT_START);

    for (LoggedDashboardChooser<Pose2d> scoreChooser : scoreChoosers) {
      scoreChooser.addOption("Back Right", BACK_RIGHT_SCORE);
      scoreChooser.addOption("Back Left", BACK_LEFT_SCORE);
      scoreChooser.addOption("Left", LEFT_SCORE);
      scoreChooser.addOption("Top Left", TOP_LEFT_SCORE);
      scoreChooser.addOption("Top Right", TOP_RIGHT_SCORE);
      scoreChooser.addOption("Right", RIGHT_SCORE);

      scoreChooser.addDefaultOption("Back Right", BACK_RIGHT_SCORE);
    }

    for (LoggedDashboardChooser<Pose2d> loadStationChooser : loadStationChoosers) {
      loadStationChooser.addOption("Right Load Station", RIGHT_LOAD_STATION);
      loadStationChooser.addOption("Left Load Station", LEFT_LOAD_STATION);

      loadStationChooser.addDefaultOption("Right Load Station", RIGHT_LOAD_STATION);
    }

    SmartDashboard.putData(m_field);

    for (int i = 0; i < reefPointsAngles.length; i++) {
      vertexs[i] =
          new Translation2d(
              REEF_X_BLUE + (REEF_SIZE + 0.2) * Math.sin(reefPointsAngles[i]),
              REEF_Y + (REEF_SIZE + 0.2) * Math.cos(reefPointsAngles[i]));
    }
  }

  public static Command autonPath;
  public static PathPlannerPath startPath;
  // public static ArrayList<Pose2d[]> paths = new ArrayList<>();

  public static void update() {
    ArrayList<Pose2d[]> paths = new ArrayList<>();
    startPath = getPathFromPoints(startChooser.get().getTranslation(), scoreChoosers[0].get());

    paths.add(startPath.getPathPoses().toArray(new Pose2d[startPath.getPathPoses().size()]));
    autonPath = AutoBuilder.followPath(startPath);

    for (int i = 0; i < scoreChoosers.length - 1; i++) {
      PathPlannerPath path1 =
          getPathFromPoints(scoreChoosers[i].get().getTranslation(), loadStationChoosers[i].get());
      PathPlannerPath path2 =
          getPathFromPoints(
              loadStationChoosers[i].get().getTranslation(), scoreChoosers[i + 1].get());

      paths.add(path1.getPathPoses().toArray(new Pose2d[path1.getPathPoses().size()]));
      paths.add(path2.getPathPoses().toArray(new Pose2d[path2.getPathPoses().size()]));

      autonPath =
          Commands.sequence(
              autonPath, AutoBuilder.followPath(path1), AutoBuilder.followPath(path2));
    }
    m_field.getObject("traj").setPoses(paths.get(displayChooser.get()));
  }

  public static Command getAutonCommand() {
    return Commands.sequence(autonPath);
  }

  public static PathPlannerPath getPathFromPoints(Translation2d point1, Pose2d point2) {
    PathConstraints constraints = new PathConstraints(1, 0.75, Math.PI, 2 * Math.PI);
    List<Waypoint> waypoints = generateWaypoints(point1, point2.getTranslation());

    return new PathPlannerPath(
        waypoints,
        new ArrayList<>(),
        new ArrayList<>(),
        new ArrayList<>(),
        new ArrayList<>(),
        constraints,
        new IdealStartingState(
            0.0, START_ROTATION), // The ideal starting state, this is only relevant for pre-planned
        // paths, so can
        // be null for on-the-fly paths.
        new GoalEndState(
            0.0,
            point2
                .getRotation()
                .rotateBy(
                    Rotation2d.fromDegrees(
                        180))), // Goal end state. You can set a holonomic rotation here. If
        // using a differential drivetrain, the rotation will have no
        // effect.
        false);
  }

  public static Pose2d getStartPose2d() {
    if (DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red) {
      return new Pose2d(
          startPath.flipPath().getPathPoses().get(0).getTranslation(),
          START_ROTATION.rotateBy(Rotation2d.fromDegrees(180)));
    }
    return startChooser.get();
  }

  public static List<Waypoint> generateWaypoints(Translation2d startPoint, Translation2d endPoint) {
    ArrayList<Waypoint> waypoints = new ArrayList<>();
    waypoints.add(new Waypoint(null, startPoint, startPoint));
    waypoints.add(new Waypoint(endPoint, endPoint, null));
    ArrayList<Integer> intersectedPlanes = getIntersectedPlanes(startPoint, endPoint);
    if (intersectedPlanes.isEmpty()) {
      return waypoints;
    }
    int planeLength =
        Math.min(
            Math.abs(intersectedPlanes.get(0) - intersectedPlanes.get(1)),
            6 - Math.abs(intersectedPlanes.get(0) - intersectedPlanes.get(1)));
    Translation2d vertexPoint1, vertexPoint2;
    double L1, L2, L3, t1, t2;
    Translation2d[] controlPoints;

    switch (planeLength) {
      case 1 -> {
        vertexPoint2 = endPoint;
        vertexPoint1 =
            intersectedPlanes.get(0) == 0 && intersectedPlanes.get(1) == 5
                ? vertexs[0]
                : vertexs[intersectedPlanes.get(0) + 1];

        L1 = dist(startPoint.getX(), startPoint.getY(), vertexPoint1.getX(), vertexPoint1.getY());
        L2 =
            dist(
                vertexPoint1.getX(), vertexPoint1.getY(), vertexPoint2.getX(), vertexPoint2.getY());
        t1 = L1 / (L1 + L2) * 0.86;

        controlPoints =
            getControlPoints(startPoint, endPoint, vertexPoint1, vertexPoint2, t1, 0.999);
      }
      case 2 -> {
        vertexPoint1 = vertexs[intersectedPlanes.get(0) + 1];
        vertexPoint2 = vertexs[intersectedPlanes.get(0) + 2];
        if (intersectedPlanes.get(0) == 0 && intersectedPlanes.get(1) == 4) {
          vertexPoint1 = vertexs[5];
          vertexPoint2 = vertexs[0];
        } else if (intersectedPlanes.get(0) == 1 && intersectedPlanes.get(1) == 5) {
          vertexPoint1 = vertexs[0];
          vertexPoint2 = vertexs[1];
        }

        if (dist(startPoint.getX(), startPoint.getY(), vertexPoint1.getX(), vertexPoint1.getY())
            > dist(
                startPoint.getX(), startPoint.getY(), vertexPoint2.getX(), vertexPoint2.getY())) {
          Translation2d cpy = vertexPoint1;
          vertexPoint1 = vertexPoint2;
          vertexPoint2 = cpy;
        }
        L1 = dist(startPoint.getX(), startPoint.getY(), vertexPoint1.getX(), vertexPoint1.getY());
        L2 =
            dist(
                vertexPoint1.getX(), vertexPoint1.getY(), vertexPoint2.getX(), vertexPoint2.getY());
        L3 = dist(vertexPoint2.getX(), vertexPoint2.getY(), endPoint.getX(), endPoint.getY());

        t1 = L1 / (L1 + L2 + L3) * 0.86;
        t2 = 1 - L3 / (L1 + L2 + L3);

        controlPoints = getControlPoints(startPoint, endPoint, vertexPoint1, vertexPoint2, t1, t2);
      }
      case 3 -> {
        int intersectVertex1 = intersectedPlanes.get(0) + 1;
        int intersectVertex2 = intersectedPlanes.get(0) + 2;

        if (intersectedPlanes.get(0) == 2 && intersectedPlanes.get(1) == 5) {
          intersectVertex1 = 3;
          intersectVertex2 = 4;
        }
        vertexPoint1 = vertexs[intersectVertex1];
        vertexPoint2 = vertexs[intersectVertex2];

        if (dist(startPoint.getX(), startPoint.getY(), vertexPoint1.getX(), vertexPoint1.getY())
            > dist(
                startPoint.getX(), startPoint.getY(), vertexPoint2.getX(), vertexPoint2.getY())) {
          Translation2d cpy = vertexPoint1;
          vertexPoint1 = vertexPoint2;
          vertexPoint2 = cpy;
        }
        L1 = dist(startPoint.getX(), startPoint.getY(), vertexPoint1.getX(), vertexPoint1.getY());
        L2 =
            dist(
                vertexPoint1.getX(), vertexPoint1.getY(), vertexPoint2.getX(), vertexPoint2.getY());
        L3 = dist(vertexPoint2.getX(), vertexPoint2.getY(), endPoint.getX(), endPoint.getY());

        t1 = L1 / (L1 + L2 + L3) * 0.86;
        t2 = 1 - L3 / (L1 + L2 + L3);

        controlPoints = getControlPoints(startPoint, endPoint, vertexPoint1, vertexPoint2, t1, t2);
      }
      default -> {
        controlPoints = new Translation2d[] {new Translation2d(), new Translation2d()};
      }
    }
    waypoints.set(0, new Waypoint(null, startPoint, startPoint.plus(controlPoints[0])));
    waypoints.set(1, new Waypoint(endPoint.plus(controlPoints[1]), endPoint, null));
    return waypoints;
  }

  private static final double[] reefPointsAngles =
      new double[] {0, Math.PI / 3, 2 * Math.PI / 3, Math.PI, 4 * Math.PI / 3, 5 * Math.PI / 3};
  private static final double REEF_Y = 4;
  private static final double REEF_X_BLUE = 4.5;
  private static final double REEF_SIZE = 1.57;

  // Intersection code
  public static ArrayList<Integer> getIntersectedPlanes(
      Translation2d startPoint, Translation2d endPoint) {
    ArrayList<Integer> intersectedPlanes = new ArrayList<>();
    for (int i = 0; i < reefPointsAngles.length; i++) {
      Translation2d vertex1 =
          new Translation2d(
              REEF_X_BLUE + REEF_SIZE * Math.sin(reefPointsAngles[i]),
              REEF_Y + REEF_SIZE * Math.cos(reefPointsAngles[i]));
      int nextI = (i + 1) % 6;
      Translation2d vertex2 =
          new Translation2d(
              REEF_X_BLUE + REEF_SIZE * Math.sin(reefPointsAngles[nextI]),
              REEF_Y + REEF_SIZE * Math.cos(reefPointsAngles[nextI]));
      if (doIntersect(vertex1, vertex2, startPoint, endPoint)) {
        intersectedPlanes.add(i);
      }
    }
    return intersectedPlanes;
  }

  // Math stuff
  public static Translation2d[] getControlPoints(
      Translation2d startPoint,
      Translation2d endPoint,
      Translation2d vertexPoint1,
      Translation2d vertexPoint2,
      double t1,
      double t2) {
    double t1Minus = (1 - t1);
    double t1MinusSquared = t1Minus * t1Minus;
    double t1MinusCubed = t1MinusSquared * t1Minus;
    double t1Squared = t1 * t1;
    double t1Cubed = t1Squared * t1;

    double t2Minus = (1 - t2);
    double t2MinusSquared = t2Minus * t2Minus;
    double t2MinusCubed = t2MinusSquared * t2Minus;
    double t2Squared = t2 * t2;
    double t2Cubed = t2Squared * t2;

    Translation2d controlP2 = new Translation2d(0, 0);

    double num2_1X =
        3
            * t1MinusSquared
            * t1
            * (-endPoint.getX() * t2Cubed - startPoint.getX() * t2MinusCubed + vertexPoint2.getX());
    double num2_2X =
        3
            * (endPoint.getX() * t1Cubed + startPoint.getX() * t1MinusCubed - vertexPoint1.getX())
            * t2MinusSquared
            * t2;

    double den_2_1 = 9 * t1MinusSquared * t1 * t2Minus * t2Squared;
    double den_2_2 = -9 * t1Minus * t1Squared * t2MinusSquared * t2;

    controlP2 =
        controlP2.plus(
            new Translation2d((num2_1X + num2_2X) / (den_2_1 + den_2_2) - endPoint.getX(), 0));
    double num2_1Y =
        3
            * t1MinusSquared
            * t1
            * (-endPoint.getY() * t2Cubed - startPoint.getY() * t2MinusCubed + vertexPoint2.getY());
    double num2_2Y =
        3
            * (endPoint.getY() * t1Cubed + startPoint.getY() * t1MinusCubed - vertexPoint1.getY())
            * t2MinusSquared
            * t2;

    controlP2 =
        controlP2.plus(
            new Translation2d(0, (num2_1Y + num2_2Y) / (den_2_1 + den_2_2) - endPoint.getY()));

    if (t2 == 0.999) {
      controlP2 = new Translation2d(0, 0);
    }

    double num1_1X =
        vertexPoint1.getX() - t1MinusCubed * startPoint.getX() - t1Cubed * endPoint.getX();
    double num1_2X = -3 * t1Squared * t1Minus * (controlP2.getX() + endPoint.getX());

    double den1 = 3 * t1 * t1MinusSquared;
    Translation2d controlP1 = new Translation2d(0, 0);
    controlP1 =
        controlP1.plus(new Translation2d((num1_1X + num1_2X) / den1 - startPoint.getX(), 0));

    double num1_1Y =
        vertexPoint1.getY() - t1MinusCubed * startPoint.getY() - t1Cubed * endPoint.getY();
    double num1_2Y = -3 * t1Squared * t1Minus * (controlP2.getY() + endPoint.getY());

    controlP1 =
        controlP1.plus(new Translation2d(0, (num1_1Y + num1_2Y) / den1 - startPoint.getY()));
    return new Translation2d[] {controlP1, controlP2};
  }

  // The intersection code
  public static boolean onSegment(Translation2d p, Translation2d q, Translation2d r) {
    return q.getX() <= Math.max(p.getX(), r.getX())
        && q.getX() >= Math.min(p.getX(), r.getX())
        && q.getY() <= Math.max(p.getY(), r.getY())
        && q.getY() >= Math.min(p.getY(), r.getY());
  }

  public static int orientationLine(Translation2d p, Translation2d q, Translation2d r) {
    double val =
        (q.getY() - p.getY()) * (r.getX() - q.getX())
            - (q.getX() - p.getX()) * (r.getY() - q.getY());
    if (val == 0) return 0; // collinear
    return (val > 0) ? 1 : 2; // 1 -> clockwise, 2 -> counterclockwise
  }

  public static boolean doIntersect(
      Translation2d p1, Translation2d q1, Translation2d p2, Translation2d q2) {
    int o1 = orientationLine(p1, q1, p2);
    int o2 = orientationLine(p1, q1, q2);
    int o3 = orientationLine(p2, q2, p1);
    int o4 = orientationLine(p2, q2, q1);

    // General case
    if (o1 != o2 && o3 != o4) return true;

    // Special cases
    if (o1 == 0 && onSegment(p1, p2, q1)) return true;
    if (o2 == 0 && onSegment(p1, q2, q1)) return true;
    if (o3 == 0 && onSegment(p2, p1, q2)) return true;
    if (o4 == 0 && onSegment(p2, q1, q2)) return true;

    return false; // No intersection
  }

  public static double dist(double x1, double y1, double x2, double y2) {
    double deltaX = x2 - x1;
    double deltaY = y2 - y1;
    return Math.sqrt(deltaX * deltaX + deltaY * deltaY);
  }
}
