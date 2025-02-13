// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import java.lang.ModuleLayer.Controller;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.PS5Controller;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.Constants.ControllerConstants;
import frc.robot.commands.AlgaeIntake;
import frc.robot.commands.ArmMagic;
import frc.robot.commands.CascadeMagic;
import frc.robot.commands.CascadeMove;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Climb;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.LED;
import frc.robot.subsystems.Vision;
import frc.robot.subsystems.Wrist;

public class RobotContainer {

    //fetching instances of subsystems
    private static RobotContainer instance;
    private Elevator elevatorSub = Elevator.getInstance();
    private LED ledSub = LED.getInstance();
    public Intake intakeSub = new Intake();
    // private static Vision visionSub = Vision.getInstance(); 
    private Arm armSub = Arm.getInstance();
    private Climb climbSub = Climb.getInstance();

    public double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    public double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max
                                                                                     // angular velocity
    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

    private final Telemetry logger = new Telemetry(MaxSpeed);

    private SendableChooser<Command> autoChooser;

    public final PS5Controller driver = new PS5Controller(0);
    public final PS5Controller operator = new PS5Controller(1);

    // DRIVER INPUTS
    private final JoystickButton brakeButton = new JoystickButton(driver, ControllerConstants.b_L2);
    private final JoystickButton slowButton = new JoystickButton(driver, ControllerConstants.b_R2);
    private final JoystickButton recenterButton = new JoystickButton(driver, ControllerConstants.b_L1);
    private final JoystickButton moveButton = new JoystickButton(driver, ControllerConstants.b_O);
    private final JoystickButton intakeButton = new JoystickButton(driver, ControllerConstants.b_X);
    private final JoystickButton outTakeButton = new JoystickButton(driver, ControllerConstants.b_O);
    private final JoystickButton ledTestButton = new JoystickButton(driver, ControllerConstants.b_TRI);

    // OPERATOR INPUTS

    private final JoystickButton armUp = new JoystickButton(operator, ControllerConstants.b_R1);
    private final JoystickButton armDown = new JoystickButton(operator, ControllerConstants.b_L1);
    private final JoystickButton elevUp = new JoystickButton(operator, ControllerConstants.b_R2);
    private final JoystickButton elevDown = new JoystickButton(operator, ControllerConstants.b_L2);
    private final POVButton armPosUp = new POVButton(operator, 0);
    private final POVButton armPosDown = new POVButton(operator, 180);
    private final POVButton armPosNeutral = new POVButton(operator, 270);
    private final JoystickButton cascade1 = new JoystickButton(operator, ControllerConstants.b_X);
    private final JoystickButton resetButton = new JoystickButton(operator, ControllerConstants.b_O);
    private final JoystickButton cascadeHome = new JoystickButton(operator, ControllerConstants.b_SQR);
    private final JoystickButton cascadeHigh = new JoystickButton(operator, ControllerConstants.b_TRI);

    


    // SysID tuning will be on operator controller
    private final JoystickButton dyanamicForward = new JoystickButton(operator, ControllerConstants.b_SQR);
    private final JoystickButton dyanamicBackward = new JoystickButton(operator, ControllerConstants.b_X);
    private final JoystickButton quasiForward = new JoystickButton(operator, ControllerConstants.b_TRI);
    private final JoystickButton quasiBackward = new JoystickButton(operator, ControllerConstants.b_O);

    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();

    public static RobotContainer getInstance() {
        if (instance == null) {
            instance = new RobotContainer();
        }
        return instance;
    }

    public RobotContainer() {

        NamedCommands.registerCommand("Recenter",
                drivetrain.runOnce(() -> drivetrain.seedFieldCentric()).withTimeout(0.1));
        drivetrain.resetPose(new Pose2d(new Translation2d(3.4, 4), new Rotation2d(0)));
        configureBindings();
    }

    private void configureBindings() {
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        drivetrain.setDefaultCommand(
                // Drivetrain will execute this command periodically
                drivetrain.applyRequest(() -> drive.withVelocityX(-driver.getLeftY() * MaxSpeed) // Drive forward with
                                                                                                 // negative Y (forward)
                        .withVelocityY(-driver.getLeftX() * MaxSpeed) // Drive left with negative X (left)
                        .withRotationalRate(-driver.getRightX() * MaxAngularRate) // Drive counterclockwise with
                                                                                  // negative X (left)
                ));

        
        slowButton.whileTrue(drivetrain.applyRequest(() -> drive.withVelocityX(-driver.getLeftY() * 1.5) // Drive
                                                                                                         // forward with
                                                                                                         // negative Y
                                                                                                         // (forward)
                .withVelocityY(-driver.getLeftX() * 1.5) // Drive left with negative X (left)
                .withRotationalRate(-driver.getRightX() * MaxAngularRate) // Drive counterclockwise with negative X
                                                                          // (left)
        ));
        slowButton.whileTrue(new RunCommand(()-> ledSub.ChangeLED(240, 200, 90), ledSub));
        slowButton.whileFalse(new RunCommand(()-> ledSub.ChangeLED(0, 0, 0), ledSub));


        brakeButton.whileTrue(drivetrain.applyRequest(() -> brake));
        recenterButton.onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));
        intakeButton.whileTrue(new AlgaeIntake(0.8, intakeSub));
        outTakeButton.whileTrue(new AlgaeIntake(-0.8, intakeSub));

        // OPERATOR BINDINGS
        // For SysID

        // dyanamicForward.whileTrue(intakeSub.sysDynamic(Direction.kForward));
        // dyanamicBackward.whileTrue(intakeSub.sysDynamic(Direction.kReverse));
        // quasiForward.whileTrue(intakeSub.sysQuasistatic(Direction.kForward));
        // quasiBackward.whileTrue(intakeSub.sysQuasistatic(Direction.kReverse));

        // armPosUp.onTrue(new ArmMagic(armSub, 0));
        // armPosDown.onTrue(new ArmMagic(armSub, -180));
        // armPosNeutral.onTrue(new ArmMagic(armSub, -90));

        // cascade1.whileTrue(new CascadeMagic(elevatorSub, 10));
        // resetButton.onTrue(new InstantCommand(()-> elevatorSub.ResetPosition(), elevatorSub));
        // cascadeHome.whileTrue(new CascadeMagic(elevatorSub, 0));
        // cascadeHigh.whileTrue(new CascadeMagic(elevatorSub, 57.1));


        ledTestButton.onTrue(new RunCommand(() -> ledSub.TestLED(), ledSub));

        intakeSub.setDefaultCommand(new RunCommand(()-> intakeSub.MovePivot(operator.getLeftX() * 0.1), intakeSub));

        armDown.whileTrue(new CascadeMove(0.2, elevatorSub));
        elevDown.whileTrue(new CascadeMove(-0.2, elevatorSub));


        elevatorSub.setDefaultCommand(new RunCommand(()-> elevatorSub.ManualMove(-operator.getLeftY()), elevatorSub));
        armSub.setDefaultCommand(new RunCommand(()-> armSub.ManualMove(-operator.getRightY() * 0.5), armSub));
        // climbSub.setDefaultCommand(new RunCommand(()-> climbSub.Move(operator.getLeftX()), climbSub));

        // armUp.whileTrue(new ArmMove(armSub, 0.1));
        // armDown.whileTrue(new ArmMove(armSub, -0.1));
        // elevUp.whileTrue(new CascadeMove(0.2, elevatorSub));
        // elevDown.whileTrue(new CascadeMove(-0.2, elevatorSub));

        // // Run SysId routines when holding back/start and X/Y.
        // // Note that each routine should be run exactly once in a single log.
        // joystick.back().and(joystick.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        // joystick.back().and(joystick.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        // joystick.start().and(joystick.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        // joystick.start().and(joystick.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

        // reset the field-centric heading on left bumper press

        drivetrain.registerTelemetry(logger::telemeterize);

        // AUTONOMOUS
        autoChooser = AutoBuilder.buildAutoChooser();
        SmartDashboard.putData("Auto Chooser", autoChooser);
    }

    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }
}
