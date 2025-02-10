// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.Constants.MotorConstants;

public class Arm extends SubsystemBase {
  /** Creates a new Arm. */
  private final TalonFX armMotor = new TalonFX(MotorConstants.wristMotorID, "Other");
  double gearRatio = 100;
  private VoltageOut vOut = new VoltageOut(0);
  private static Arm instance;
  MotionMagicVoltage request = new MotionMagicVoltage(0);


  public static Arm getInstance(){
    if(instance == null){
      instance = new Arm();
    }
    return instance;
  }

  private final SysIdRoutine pivotRoutine = new SysIdRoutine(
      new SysIdRoutine.Config(
          Volts.per(Seconds).of(0.35),
          Volts.of(0.65),
          Seconds.of(5),
          state -> SignalLogger.writeString("arm state", state.toString())),
      new SysIdRoutine.Mechanism(
          volts -> armMotor.setControl(vOut.withOutput(volts.in(Volts))),
          null,
          this));

  public Command sysDynamic(SysIdRoutine.Direction direction){
      return pivotRoutine.dynamic(direction);
    }
    public Command sysQuasistatic(SysIdRoutine.Direction direction){
      return pivotRoutine.quasistatic(direction);
    }

  public Arm() {
    Config();
    SignalLogger.start();
    
  }

  public void ManualMove(double speed){
    armMotor.set(speed);
  }

  private void Config() {
    var armConfig = new TalonFXConfiguration();
    armConfig.Feedback.SensorToMechanismRatio = gearRatio;
    armConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    armMotor.setNeutralMode(NeutralModeValue.Brake);
    var slot0Config = armConfig.Slot0;
    slot0Config.kS = 0.18405;
    slot0Config.kV = 9.5916;
    slot0Config.kA = 0.97255;
    // slot0Config.kG = 38.404;
    slot0Config.kP = 58.161;
    slot0Config.kI = 0;
    slot0Config.kD = 13.754;

    armConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
    armConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold = 5;
    armConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
    armConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold = -190;

    var armMM = armConfig.MotionMagic;
    armMM.withMotionMagicCruiseVelocity(0.1).
    withMotionMagicAcceleration(0.1).
    withMotionMagicJerk(0);
    // armMotor.getConfigurator().apply(slot0Config);
    armMotor.getConfigurator().apply(armConfig);
  }

  public double GetAngle(){
    return armMotor.getPosition().getValue().in(Degrees);
  }

  public void MagicMove(double degrees){
      armMotor.setControl(request.withPosition(Units.degreesToRotations(degrees)));
  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    // System.out.println(GetAngle());
  }
}
