// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LimelightHelpers;
import frc.robot.subsystems.drive.Drive;

public class Vision extends SubsystemBase {
  Drive drive;
  /** Creates a new Vision. */
  public Vision(Drive drive) {
    this.drive = drive;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    LimelightHelpers.PoseEstimate limelightMeasurement =
        LimelightHelpers.getBotPoseEstimate_wpiBlue("limelight");
    if (limelightMeasurement.tagCount >= 2) { // Only trust measurement if we see multiple tags
      // m_poseEstimator.setVisionMeasurementStdDevs(VecBuilder.fill(0.7, 0.7, 9999999));
      drive.addVisionMeasurement(
          limelightMeasurement.pose,
          limelightMeasurement.timestampSeconds,
          VecBuilder.fill(0.7, 0.7, 9999999));
    }
    double tx = LimelightHelpers.getTX("");
    double ty = LimelightHelpers.getTY("");
    double ta = LimelightHelpers.getTA("");
    SmartDashboard.putNumber("targetx", tx);
    SmartDashboard.putNumber("targety", ty);
    SmartDashboard.putNumber("targeta", ta);
  }
}
