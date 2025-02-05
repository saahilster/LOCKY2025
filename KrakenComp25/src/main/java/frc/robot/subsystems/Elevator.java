// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix.Util;
import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.sim.ChassisReference;

import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.Unit;
import edu.wpi.first.wpilibj.motorcontrol.Talon;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.Constants.MotorConstants;

public class Elevator extends SubsystemBase {
  /** Creates a new Elevator. */
  private final TalonFX leftMotor = new TalonFX(MotorConstants.leftCascadeID, "Other");
  private final TalonFX rightMotor = new TalonFX(MotorConstants.rightCascadeID, "Other");
  Follower follower = new Follower(MotorConstants.leftCascadeID, true);

  private final TalonFX armMotor = new TalonFX(MotorConstants.wristMotorID, "Other");
  private static Elevator instance;

  private VoltageOut vOut = new VoltageOut(0);
  private static final double elevatorGearRatio = 100;
  private static final double armGearRatio = 100;

  private LED ledSub = LED.getInstance();

  public static Elevator getInstance(){
    if(instance == null){
      instance = new Elevator();
    }
    return instance;
  }

  //TODO: CHANGE VALUES FOR LOWER SPEEDS
  private final SysIdRoutine elevatorRoutine = new SysIdRoutine(
      new SysIdRoutine.Config(
          null,
          Volts.of(2),
          Seconds.of(3),
          state -> SignalLogger.writeString("state", state.toString())),
      new SysIdRoutine.Mechanism(
          volts -> leftMotor.setControl(vOut.withOutput(volts.in(Volts))),
          null,
          this));

  //TODO: CHANGE VALUES
  //ARM SYSID ROUTINE
  private final SysIdRoutine armRoutine = new SysIdRoutine(
      new SysIdRoutine.Config(
          null,
          Volts.of(3),
          Seconds.of(3),
          state -> SignalLogger.writeString("state", state.toString())),
      new SysIdRoutine.Mechanism(
          volts -> leftMotor.setControl(vOut.withOutput(volts.in(Volts))),
          null,
          this));

    public Command sysDynamic(SysIdRoutine.Direction direction){
      return elevatorRoutine.dynamic(direction);
    }
    public Command sysQuasistatic(SysIdRoutine.Direction direction){
      return elevatorRoutine.quasistatic(direction);
    }

  public Elevator() {
    BrakeMode();
    rightMotor.setControl(follower);
    CascadeConfig();
    ArmConfig();
    SignalLogger.start();
  }

  private void BrakeMode() {
    leftMotor.setNeutralMode(NeutralModeValue.Brake);
    rightMotor.setNeutralMode(NeutralModeValue.Brake);
    armMotor.setNeutralMode(NeutralModeValue.Brake);
  }

  public void ManualMove(double speed) {
    leftMotor.set(speed);
  }

  public void MagicMove(double feet) {
    MotionMagicVoltage request = new MotionMagicVoltage(0);
    leftMotor.setControl(request);
  }

  private void CascadeConfig() {
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

  private void ArmConfig() {
    var armConfig = new TalonFXConfiguration();
    armConfig.Feedback.SensorToMechanismRatio = armGearRatio;
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
    leftMotor.getConfigurator().apply(slot0Config);
  }

  public void ArmManual(double speed){
    armMotor.set(speed);
  }

  public void ArmMagic(double degrees){
    MotionMagicVoltage request = new MotionMagicVoltage(Units.degreesToRotations(degrees));
    armMotor.setControl(request);
  }

  // TODO: Set limits
  public boolean ElevatorLimits() {
    return false;
  }

  public boolean ArmLimits(){
    return false;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

}
