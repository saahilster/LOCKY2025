// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix.Util;
import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.sim.ChassisReference;

import edu.wpi.first.math.MathUtil;
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
  private final TalonFX rightMotor = new TalonFX(MotorConstants.rightCascadeID, "Other");

  private static Elevator instance;

  private VoltageOut vOut = new VoltageOut(0);
  private static final double elevatorGearRatio = 100;
  private static final double spoolDiameter = 1.625;
  private static final double maxHeight = 64.75;
  private static final double minHeight = 39.5;

  private static final double spoolCircumference = spoolDiameter * Math.PI;
  //2 accounts for the stages of the cascade 
  //gear ratio not accounted for beacause sensorToMechanismRatio is used
  private static final double heightPerRotation = spoolCircumference * 2;
  private MotionMagicVoltage request = new MotionMagicVoltage(0);

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
          Volts.per(Seconds).of(0.75),
          Volts.of(1.5),
          Seconds.of(4),
          state -> SignalLogger.writeString("state", state.toString())),
      new SysIdRoutine.Mechanism(
          volts -> rightMotor.setControl(vOut.withOutput(volts.in(Volts))),
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
    CascadeConfig();
    SignalLogger.start();
  }

  private void BrakeMode() {
    rightMotor.setNeutralMode(NeutralModeValue.Brake);
  }

  public void ManualMove(double speed) {
    rightMotor.set(speed);
  }

  private void CascadeConfig() {
    var cascadeConfig = new TalonFXConfiguration();
    cascadeConfig.Feedback.SensorToMechanismRatio = elevatorGearRatio;
    cascadeConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

    var slot0Config = cascadeConfig.Slot0;
    slot0Config.GravityType = GravityTypeValue.Elevator_Static;
    slot0Config.kS = 0.062304;
    slot0Config.kV = 6.0618;
    slot0Config.kA = 0.0011081;
    slot0Config.kG = 0.041429;
    slot0Config.kP = 66.977;
    slot0Config.kI = 0;
    slot0Config.kD = 9.2718;

    // cascadeConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
    // cascadeConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold = maxHeight;
    // cascadeConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
    // cascadeConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold = minHeight;

    var elevateMM = cascadeConfig.MotionMagic;
    elevateMM.MotionMagicCruiseVelocity = 1.67;
    elevateMM.MotionMagicAcceleration = 5;
    elevateMM.MotionMagicJerk = 0;
    rightMotor.getConfigurator().apply(cascadeConfig);
  }

  public double GetHeight(){
    return rightMotor.getPosition().getValueAsDouble() * heightPerRotation;
  }

  public void SetHeight(double inches){
    double rotations = inches / heightPerRotation;
    rightMotor.setControl(request.withPosition(rotations));
  }

  public void ResetPosition(){
    rightMotor.setPosition(0);
  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    System.out.println(GetHeight());
  }

}
