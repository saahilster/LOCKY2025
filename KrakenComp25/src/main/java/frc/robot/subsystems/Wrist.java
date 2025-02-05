// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volt;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.controls.VoltageOut;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Velocity;
import edu.wpi.first.wpilibj.motorcontrol.Talon;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.Constants.MotorConstants;
import com.ctre.phoenix6.SignalLogger;

//TODO: DEPRECATE THIS SUB
public class Wrist extends SubsystemBase {
  // references
  private final TalonFX wristMotor = new TalonFX(MotorConstants.wristMotorID);

  private VoltageOut vOut = new VoltageOut(0);

  private final SysIdRoutine sysIdRoutine = new SysIdRoutine(
      new SysIdRoutine.Config(
          null,
          Volt.of(3),
          Seconds.of(3),
          state -> SignalLogger.writeString("state", state.toString())),
      new SysIdRoutine.Mechanism(
          volts -> wristMotor.setControl(vOut.withOutput(volts.in(Volt))),
          null,
          this));

  public Command sysIDQuasistatic(SysIdRoutine.Direction direction) {
    return sysIdRoutine.quasistatic(direction);
  }

  public Command sysIDDynamic(SysIdRoutine.Direction direction) {
    return sysIdRoutine.dynamic(direction);
  }

  private final double gearRatio = 100;

  /** Creates a new Wrist. */
  public Wrist() {
    MotorConfigs();
  }

  private void MotorConfigs() {
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

  public void ManualMove(double speed) {
    wristMotor.set(speed);
  }

  public void RotateWrist(double degrees) {
    MotionMagicVoltage request = new MotionMagicVoltage(Units.degreesToRotations(degrees));
    wristMotor.setControl(request);
  }

  private double WristAngle() {
    return wristMotor.getPosition().getValueAsDouble() / 360;
  }

  public boolean WristLimit() {
    return false;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Degrees", WristAngle());
  }
}
