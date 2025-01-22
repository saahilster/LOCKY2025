// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.Constants.MotorConstants;

public class Elevator extends SubsystemBase {
  /** Creates a new Elevator. */
  private final TalonFX leftMotor = new TalonFX(MotorConstants.leftCascadeID);
  private final TalonFX rightMotor = new TalonFX(MotorConstants.rightCascadeID);
  Follower follower = new Follower(0, true);

  private final double elevatorGearRatio = 100;
  final MotionMagicVoltage mmRequest = new MotionMagicVoltage(0);

  public Elevator() {
    BrakeMode();
    rightMotor.setControl(follower);
    MotorConfig();
  }

  private void BrakeMode(){
    leftMotor.setNeutralMode(NeutralModeValue.Brake);
    rightMotor.setNeutralMode(NeutralModeValue.Brake);
  }

  public void ManualMove(double speed){
    leftMotor.set(speed);
  }

  public void MagicMove(double feet){
    leftMotor.setControl(mmRequest.withPosition(feet));
  }

  private void MotorConfig(){
    var cascadeConfig = new TalonFXConfiguration();
    cascadeConfig.Feedback.SensorToMechanismRatio = elevatorGearRatio;
    var slot0Config = cascadeConfig.Slot0;
    slot0Config.kS = 0;
    slot0Config.kV = 0;
    slot0Config.kA = 0;
    slot0Config.kP = 0;
    slot0Config.kI = 0;
    slot0Config.kD = 0;

    var elevateMM = cascadeConfig.MotionMagic;
    elevateMM.MotionMagicCruiseVelocity = 0;
    elevateMM.MotionMagicAcceleration = 0;
    elevateMM.MotionMagicJerk = 0;
    leftMotor.getConfigurator().apply(slot0Config);
    rightMotor.getConfigurator().apply(slot0Config);

  }

  //TODO: Set limits
  public boolean ElevatorLimits(){
    return false;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
