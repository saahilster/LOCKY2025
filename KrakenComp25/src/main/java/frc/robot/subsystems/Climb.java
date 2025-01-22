// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.Constants.MotorConstants;

public class Climb extends SubsystemBase {
  double gearRatio = 0;
  private final TalonFX climbMotor = new TalonFX(MotorConstants.climbMotorID);


  /** Creates a new Climb. */
  public Climb() {}

  public void Move(double speed){
    climbMotor.set(speed);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
