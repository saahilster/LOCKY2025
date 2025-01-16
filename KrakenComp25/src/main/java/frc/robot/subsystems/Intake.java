// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.Constants.MotorConstants;

import static edu.wpi.first.units.Units.*;

import java.util.function.DoubleSupplier;

public class Intake extends SubsystemBase {
  /** Creates a new Intake. */

  private final double pivotGearRatio = 0;


  private final TalonFX pivotMotor = new TalonFX(MotorConstants.pivot);
  private final TalonFX intakeMotor = new TalonFX(MotorConstants.algaeIntake);
  final MotionMagicVoltage mmRequest = new MotionMagicVoltage(0); 

  //SysID related
  private VoltageOut vOut = new VoltageOut(0.0);

  private final SysIdRoutine sysIdRoutine = new SysIdRoutine(
    new SysIdRoutine.Config(
      null,
      Volts.of(0),
      Seconds.of(3),
      state -> SignalLogger.writeString("state", state.toString())
    ), 
    new SysIdRoutine.Mechanism(
      volts -> pivotMotor.setControl(vOut.withOutput(volts.in(Volts))), 
      null,
      this));

  public Command sysIDQuasistatic(SysIdRoutine.Direction direction){
    return sysIdRoutine.quasistatic(direction);
  }

  public Command sysIDDynamic(SysIdRoutine.Direction direction){
    return sysIdRoutine.dynamic(direction);
  }

  public Intake() {
    MotorConfigs();
  }

  private void MotorConfigs(){
    var pivotConfig = new TalonFXConfiguration();
    var slot0Config = pivotConfig.Slot0;
    slot0Config.kS = 0;
    slot0Config.kV = 0;
    slot0Config.kA = 0;
    slot0Config.kP = 0;
    slot0Config.kI = 0;
    slot0Config.kD = 0;

    var pivotMotionMagic = pivotConfig.MotionMagic;
    pivotMotionMagic.MotionMagicCruiseVelocity = 0;
    pivotMotionMagic.MotionMagicAcceleration = 0;
    pivotMotionMagic.MotionMagicJerk = 0;
    pivotConfig.Feedback.SensorToMechanismRatio = pivotGearRatio;
    pivotMotor.getConfigurator().apply(slot0Config);
  }

  //for feed forward or PID pivot using degrees
  public void RotatePivot(double degrees){
    double processedDegrees = degrees / 360;
    pivotMotor.setControl(mmRequest.withPosition(processedDegrees));
  }

  //manual move
  public void MovePivot(double speed){

  }

  public void MoveIntake(double speed){

  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
