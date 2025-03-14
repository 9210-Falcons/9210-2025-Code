// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.mechanisms;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Flipper extends SubsystemBase {
  PIDController flipperPID = new PIDController(0.04, 0, 0);
  SparkMax flipper;
  /** Creates a new flipper. */
  public Flipper() {
    flipper = new SparkMax(9, MotorType.kBrushless);
  }

  public void L1Scoring() {
    flipper.set(flipperPID.calculate(getFlipperPosition(), 32));
  }

  public void L2Scoring() {
    flipper.set(flipperPID.calculate(getFlipperPosition(), 13));
  }

  public double getFlipperPosition() {
    return flipper.getEncoder().getPosition();
  }

  public void setFlipper(double power) {
    flipper.set(power);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Flipper position", getFlipperPosition());
  }
}
