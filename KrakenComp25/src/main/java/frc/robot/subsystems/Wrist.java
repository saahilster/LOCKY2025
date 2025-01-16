// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.motorcontrol.Talon;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.Constants.MotorConstants;

public class Wrist extends SubsystemBase {
  private final TalonFX wristMotor = new TalonFX(MotorConstants.wrist);
  private final TalonFX coralIntake = new TalonFX(MotorConstants.coralIntake);
  /** Creates a new Wrist. */
  public Wrist() {
    MotorConfigs();
  }

  private void MotorConfigs(){

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
