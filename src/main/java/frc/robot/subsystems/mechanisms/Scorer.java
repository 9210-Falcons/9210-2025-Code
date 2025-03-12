// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.mechanisms;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
// import org.littletonrobotics.junction.Logger;

public class Scorer extends SubsystemBase {
  SparkMax L1 = new SparkMax(30, MotorType.kBrushed);
  /** Creates a new Scocer. */
  public Scorer() {}

  public void setPower(double power) {
    // Logger.recordOutput("/Score/power", power);
    L1.set(power);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
