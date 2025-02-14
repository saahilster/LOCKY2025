// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.

// package frc.robot.subsystems;

// import edu.wpi.first.wpilibj2.command.SubsystemBase;
// import com.ctre.phoenix.led.CANdle;
// import com.ctre.phoenix.led.CANdle.LEDStripType;
// import com.ctre.phoenix.led.ColorFlowAnimation.Direction;
// import com.ctre.phoenix.led.CANdleConfiguration;
// import com.ctre.phoenix.led.ColorFlowAnimation;
// import com.ctre.phoenix.led.SingleFadeAnimation;
// import com.ctre.phoenix.led.StrobeAnimation;

// public class LED extends SubsystemBase {
//   /** Creates a new LED. */
//   private final CANdle candle = new CANdle(20, "Other");
//   private static LED instance;

//   // TODO: Change names and colors to desired

//   // when intake is armed
//   public SingleFadeAnimation noteAnim = new SingleFadeAnimation(255, 110, 0);
//   // angle reached
//   public ColorFlowAnimation armAngle = new ColorFlowAnimation(0, 100, 200, 50, 0.7, 100, Direction.Forward);
//   // floor level
//   public SingleFadeAnimation floorLevel = new SingleFadeAnimation(165, 92, 0);
//   // slow drive
//   public StrobeAnimation slowDriveAnim = new StrobeAnimation(255, 0, 0, 100, 0.05, 100);
//   // defualt anime
//   public SingleFadeAnimation defaultAnim = new SingleFadeAnimation(255, 255, 255, 255, 0.07, 100);

//   private SingleFadeAnimation reefHeightAnim;

//   public StrobeAnimation SHOOT = new StrobeAnimation(0, 255, 0, 0, 0.5, 100);

//   // private enum LEDState {
//   //   L1, L2, L3, L4,
//   //   coralIntake,
//   //   algaeIntake,
//   //   none
//   // }

//   // public LEDState state;

//   public LED() {
//     CANdleConfiguration config = new CANdleConfiguration();
//     config.brightnessScalar = 2;
//     candle.configLEDType(LEDStripType.GRB);
//     candle.configV5Enabled(true);
//     config.statusLedOffWhenActive = true;
//     candle.configVBatOutput(CANdle.VBatOutputMode.Modulated);
//     candle.configAllSettings(config, 100);
//   }

//   public static LED getInstance() {
//     if (instance == null) {
//       instance = new LED();
//     }
//     return instance;
//   }

//   public void SetHeightColors(int level) {
//     switch (level) {
//       case 1:
//         reefHeightAnim = new SingleFadeAnimation(level, level, level);
//         break;
//       case 2:
//         reefHeightAnim = new SingleFadeAnimation(level, level, level);
//         break;
//       case 3:
//         reefHeightAnim = new SingleFadeAnimation(level, level, level);
//         break;
//       case 4:
//         reefHeightAnim = new SingleFadeAnimation(level, level, level);
//         break;
//       default:
//         System.out.println("VALUE OUT OF BOUNDS");
//         break;
//     }
//   }

//   public void TestLED() {
//     candle.setLEDs(255, 0, 0);
//   }

//   public void ChangeLED(int r, int g, int b){
//     candle.setLEDs(r, g, b);
//   }

//   public void Intaking() {
//     candle.animate(armAngle);
//   }

//   // private void StateListener(){
//   //   switch (state) {
//   //     case L1:
//   //     candle.setLEDs(63, 63, 63);
//   //       break;
//   //     case L2:
//   //     candle.setLEDs(126, 126, 126);
//   //       break;
//   //     case L3:
//   //     candle.setLEDs(189, 189, 189);
//   //       break;
//   //     case L4:
//   //     candle.setLEDs(252, 252, 252);
//   //       break;
//   //     case coralIntake:
//   //       break;
//   //       case algaeIntake:
//   //       break;
      
//   //     default:
//   //       break;
//   //   }
//   // }

//   @Override
//   public void periodic() {
//     // This method will be called once per scheduler run
//     // StateListener();
//   }
// }
