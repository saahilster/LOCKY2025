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
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.Constants.MotorConstants;

public class Elevator extends SubsystemBase {
  /** Creates a new Elevator. */
  private final TalonFX leftMotor = new TalonFX(MotorConstants.leftCascadeID);
  private final TalonFX rightMotor = new TalonFX(MotorConstants.rightCascadeID);
  Follower follower = new Follower(MotorConstants.leftCascadeID, true);

  private VoltageOut vOut = new VoltageOut(0);

  private static final double elevatorGearRatio = 100;

  private final ElevatorSim elevatorSim = new ElevatorSim(LinearSystemId.createElevatorSystem(DCMotor.getKrakenX60(1),
      4.312,
      0,
      elevatorGearRatio), 
      DCMotor.getKrakenX60(1),
      1.0292588,
      0,
      true,
      1.0292588,
      0.001);

  //TODO: CHANGE VALUES FOR LOWER SPEEDS
  private final SysIdRoutine sysIdRoutine = new SysIdRoutine(
      new SysIdRoutine.Config(
          null,
          Volts.of(3),
          Seconds.of(3),
          state -> SignalLogger.writeString("state", state.toString())),
      new SysIdRoutine.Mechanism(
          volts -> leftMotor.setControl(vOut.withOutput(volts.in(Volts))),
          null,
          this));


  public Elevator() {
    BrakeMode();
    rightMotor.setControl(follower);
    MotorConfig();
  }

  private void BrakeMode() {
    leftMotor.setNeutralMode(NeutralModeValue.Brake);
    rightMotor.setNeutralMode(NeutralModeValue.Brake);
  }

  public void ManualMove(double speed) {
    leftMotor.set(speed);
  }

  public void MagicMove(double feet) {
    MotionMagicVoltage request = new MotionMagicVoltage(feet);
    leftMotor.setControl(request);
  }

  private void MotorConfig() {
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

  // TODO: Set limits
  public boolean ElevatorLimits() {
    return false;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    var leftMotorSim = leftMotor.getSimState();
    var rightMotorSim = rightMotor.getSimState();
    leftMotorSim.Orientation = ChassisReference.CounterClockwise_Positive;
    rightMotorSim.Orientation = ChassisReference.Clockwise_Positive;

    leftMotorSim.setSupplyVoltage(12);
    rightMotorSim.setSupplyVoltage(12);
  }
}
