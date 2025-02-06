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

  public static Arm getInstance(){
    if(instance == null){
      instance = new Arm();
    }
    return instance;
  }

  private final SysIdRoutine armRoutine = new SysIdRoutine(
      new SysIdRoutine.Config(
          Volts.per(Seconds).of(0.2),
          Volts.of(0.5),
          Seconds.of(3),
          state -> SignalLogger.writeString("arm state", state.toString())),
      new SysIdRoutine.Mechanism(
          volts -> armMotor.setControl(vOut.withOutput(volts.in(Volts))),
          null,
          this));

  public Command sysDynamic(SysIdRoutine.Direction direction){
      return armRoutine.dynamic(direction);
    }
    public Command sysQuasistatic(SysIdRoutine.Direction direction){
      return armRoutine.quasistatic(direction);
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
    var slot0Config = armConfig.Slot0;
    slot0Config.kS = 0;
    slot0Config.kV = 0;
    slot0Config.kA = 0;
    slot0Config.kP = 0;
    slot0Config.kI = 0;
    slot0Config.kD = 0;

    var armMM = armConfig.MotionMagic;
    armMM.MotionMagicCruiseVelocity = 0;
    armMM.MotionMagicAcceleration = 0;
    armMM.MotionMagicJerk = 0;
    armMotor.getConfigurator().apply(slot0Config);
  }

  public double GetAngle(){
    return armMotor.getPosition().getValue().in(Degrees);
  }

  public void MagicMove(double degrees){
    MotionMagicVoltage request = new MotionMagicVoltage(Units.degreesToRotations(degrees));
    armMotor.setControl(request);
  }



  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
