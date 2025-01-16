// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.Constants.MotorConstants;

public class Elevator extends SubsystemBase {
  /** Creates a new Elevator. */
  private final TalonFX leftMotor = new TalonFX(MotorConstants.leftCascade);
  private final TalonFX rightMotor = new TalonFX(MotorConstants.rightCascade);
  Follower follower = new Follower(0, false);

  public Elevator() {
    BrakeMode();
    rightMotor.setControl(follower);

    MotorConfig();
  }

  private void BrakeMode(){
    leftMotor.setNeutralMode(NeutralModeValue.Brake);
    rightMotor.setNeutralMode(NeutralModeValue.Brake);
  }

  public void Move(double speed){
    leftMotor.set(speed);
  }

  private void MotorConfig(){

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
