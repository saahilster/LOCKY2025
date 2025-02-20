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

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.motorcontrol.Talon;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.Constants.MotorConstants;
import static edu.wpi.first.units.Units.*;
import java.util.function.DoubleSupplier;
import javax.print.attribute.standard.MediaSize.Other;

public class Intake extends SubsystemBase {
  /** Creates a new Intake. */
  private final TalonFX intakeMotor = new TalonFX(21, "Other");
  private final TalonFX pivotMotor = new TalonFX(16, "Other");
<<<<<<< Updated upstream
  private LED ledSUb = LED.getInstance();
=======
  // private LED ledSUb = LED.getInstance();
  private VoltageOut vOut = new VoltageOut(0);
  private MotionMagicVoltage request = new MotionMagicVoltage(0);
  private double gearRatio = 48.33;

  private final SysIdRoutine armRoutine = new SysIdRoutine(
      new SysIdRoutine.Config(
          Volts.per(Seconds).of(0.4),
          Volts.of(0.4),
          Seconds.of(4),
          state -> SignalLogger.writeString("pivot state", state.toString())),
      new SysIdRoutine.Mechanism(
          volts -> pivotMotor.setControl(vOut.withOutput(volts.in(Volts))),
          null,
          this));

  public Command sysDynamic(SysIdRoutine.Direction direction) {
    return armRoutine.dynamic(direction);
  }

  public Command sysQuasistatic(SysIdRoutine.Direction direction) {
    return armRoutine.quasistatic(direction);
  }

  private void Config() {
    var armConfig = new TalonFXConfiguration();
    armConfig.Feedback.SensorToMechanismRatio = gearRatio;
    armConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    pivotMotor.setNeutralMode(NeutralModeValue.Brake);
    var slot0Config = armConfig.Slot0;
    slot0Config.kS = 0.1;
    slot0Config.kV = 0.1;
    slot0Config.kA = 0.1;
    // slot0Config.kG = 38.404;
    slot0Config.kP = 0;
    slot0Config.kI = 0;
    slot0Config.kD = 0;

    armConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
    armConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold = 5;
    armConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
    armConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold = -190;

    var armMM = armConfig.MotionMagic;
    armMM.MotionMagicCruiseVelocity = 0.2;
    armMM.MotionMagicAcceleration = 0.2;
    armMM.MotionMagicJerk = 0;
    pivotMotor.getConfigurator().apply(armConfig);
  }
>>>>>>> Stashed changes

  public Intake() {
    intakeMotor.setNeutralMode(NeutralModeValue.Brake);
  }

  public void MoveIntake(double speed) {
    intakeMotor.set(speed);
    ledSUb.Intaking();
  }

<<<<<<< Updated upstream
  public void MovePivot(double speed){
    pivotMotor.set(speed);
=======
  public void MagicMove(double degrees) {
    pivotMotor.setControl(request.withPosition(Units.degreesToRotations(degrees)));
>>>>>>> Stashed changes
  }

  public void MovePivot(double speed) {
    pivotMotor.set(speed);
  }

  public double GetPosition() {
    return pivotMotor.getPosition().getValue().in(Degrees);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
<<<<<<< Updated upstream
=======
    SmartDashboard.putNumber("Pivot Position", GetPosition());
    SmartDashboard.putNumber("Pivot Voltage", pivotMotor.getMotorVoltage().getValueAsDouble());
>>>>>>> Stashed changes
  }
}
