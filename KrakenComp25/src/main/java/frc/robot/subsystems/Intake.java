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
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.motorcontrol.Talon;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.Constants.MotorConstants;

import static edu.wpi.first.units.Units.*;

import java.util.function.DoubleSupplier;

import javax.print.attribute.standard.MediaSize.Other;

//TODO: remove all pivot code after wrist code is implemented
public class Intake extends SubsystemBase {
  /** Creates a new Intake. */
  private final TalonFX intakeMotor = new TalonFX(MotorConstants.algaeIntakeID, "Other");
  private final TalonFX pivotMotor = new TalonFX(21, "Other");
  private LED ledSUb = LED.getInstance();

  public Intake() {
    intakeMotor.setNeutralMode(NeutralModeValue.Brake);
  }

  public void MoveIntake(double speed){
    intakeMotor.set(speed);
    ledSUb.Intaking();
  }

  public void MovePivot(double speed){
    pivotMotor.set(speed);
  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
