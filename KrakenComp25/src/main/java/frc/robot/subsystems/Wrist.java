// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.motorcontrol.Talon;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.Constants.MotorConstants;

public class Wrist extends SubsystemBase {
  //references
  private final TalonFX wristMotor = new TalonFX(MotorConstants.wristMotorID);
  private final TalonFX coralIntake = new TalonFX(MotorConstants.coralIntakeID);
  final MotionMagicVoltage mmRequest = new MotionMagicVoltage(0);

  private final double gearRatio = 100;
  /** Creates a new Wrist. */
  public Wrist() {
    MotorConfigs();
  }

  private void MotorConfigs(){
    var wristConfig = new TalonFXConfiguration();
    var slot0Config = wristConfig.Slot0;
    slot0Config.kS = 0;
    slot0Config.kV = 0;
    slot0Config.kA = 0;
    slot0Config.kP = 0;
    slot0Config.kI = 0;
    slot0Config.kD = 0;

    var wristMotionMagic = wristConfig.MotionMagic;
    wristMotionMagic.MotionMagicCruiseVelocity = 0;
    wristMotionMagic.MotionMagicAcceleration = 0;
    wristMotionMagic.MotionMagicJerk = 0;
    wristConfig.Feedback.SensorToMechanismRatio = gearRatio;
    wristMotor.getConfigurator().apply(slot0Config);
  }

  public void ManualMove(double speed){
    wristMotor.set(speed);
  }

  public void RotatePivot(double degrees){
    double processedDegrees = degrees / 360;
    wristMotor.setControl(mmRequest.withPosition(processedDegrees));
  }

  public boolean WristLimit(){
    return false;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
