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
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.Constants.ControllerConstants;
import frc.robot.commands.AlgaeIntake;
import frc.robot.commands.ArmMagic;
import frc.robot.commands.CascadeMagic;
import frc.robot.commands.CascadeMove;
import frc.robot.commands.PivotCommand;
import frc.robot.commands.PivotMagic;
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

    // fetching instances of subsystems
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
    private final POVButton coralHeightPOV = new POVButton(operator, 180);
    private final POVButton pivotUp = new POVButton(operator, 270);
    private final POVButton pivotDown = new POVButton(operator, 90);
    private final JoystickButton cascade1 = new JoystickButton(operator, ControllerConstants.b_X);
    private final JoystickButton resetButton = new JoystickButton(operator, ControllerConstants.b_O);
    private final JoystickButton cascadeHome = new JoystickButton(operator, ControllerConstants.b_SQR);
    private final JoystickButton cascadeHigh = new JoystickButton(operator, ControllerConstants.b_TRI);
    private final JoystickButton intakeSequence = new JoystickButton(operator, ControllerConstants.b_L1);
    private final JoystickButton L3Button = new JoystickButton(operator, ControllerConstants.b_R1);
    private final JoystickButton L2Button = new JoystickButton(operator, ControllerConstants.b_R2);

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

    
            slowButton.whileTrue(drivetrain.applyRequest(() -> drive.withVelocityX(-driver.getLeftY() * 1) // Drive
                                                                                                     // forward with
                                                                                                     // negative Y
                                                                                                     // (forward)
            .withVelocityY(-driver.getLeftX() * 1.5) // Drive left with negative X (left)
            .withRotationalRate(-driver.getRightX() * MaxAngularRate) // Drive counterclockwise with negative X
                                                                      // (left)
        ));

        slowButton.whileTrue(new RunCommand(()-> ledSub.TestLED(240, 200, 90), ledSub));
        slowButton.whileFalse(new RunCommand(()-> ledSub.TestLED(0, 0, 0), ledSub));

        slowButton.whileTrue(new RunCommand(()-> ledSub.TestLED(200, 150, 0),
        ledSub));
        slowButton.whileFalse(new RunCommand(()-> ledSub.TestLED(255, 0, 0)));
        intakeButton.whileTrue(new AlgaeIntake(0.8, intakeSub));
        outTakeButton.whileTrue(new AlgaeIntake(-0.8, intakeSub));

        // OPERATOR BINDINGS

        // dyanamicForward.whileTrue(intakeSub.sysDynamic(Direction.kForward));
        // dyanamicBackward.whileTrue(intakeSub.sysDynamic(Direction.kReverse));
        // quasiForward.whileTrue(intakeSub.sysQuasistatic(Direction.kForward));
        // quasiBackward.whileTrue(intakeSub.sysQuasistatic(Direction.kReverse));

        // pivotDown.whileTrue(new PivotCommand(intakeSub, 0.1));
        // pivotUp.whileTrue(new PivotCommand(intakeSub, -0.1));

        pivotDown.whileTrue(new PivotCommand(intakeSub, 0.10));
        pivotUp.whileTrue(new PivotCommand(intakeSub, -0.10));

        armPosUp.whileTrue(new ArmMagic(armSub, 0));

        // coralHeightPOV.whileTrue(new CascadeMagic(elevatorSub, 16.8));
        coralHeightPOV.whileTrue(
                new ParallelCommandGroup(
                        new CascadeMagic(elevatorSub, 16.8)).
                        alongWith(new ArmMagic(armSub, 0))
                        );

        resetButton.onTrue(new InstantCommand(() -> elevatorSub.ResetPosition(), elevatorSub));
        cascadeHome.whileTrue(new CascadeMagic(elevatorSub, 0));
        // intakeSequence.whileTrue(
        // new SequentialCommandGroup(
        // new CascadeMagic(elevatorSub, 46.24491881238843).withTimeout(1),
        // new ArmMagic(armSub, -180).withTimeout(0.85),
        // new CascadeMagic(elevatorSub, 34.38456285565362).withTimeout(1),
        // new CascadeMagic(elevatorSub, 46.24491881238843).withTimeout(0.9)
        // )
        // );

        intakeSequence.whileTrue(
                new ParallelCommandGroup(
                        new CascadeMagic(elevatorSub, 46.24491881238843).withTimeout(0.5)
                                .alongWith(
                                        // Add a delay to the ArmMagic command
                                        new SequentialCommandGroup(
                                                new WaitCommand(0.2), // Adjust the delay time (in seconds) as needed
                                                new ArmMagic(armSub, -175).withTimeout(0.5)))
                                .andThen(
                                        new WaitCommand(0.65),
                                        new CascadeMagic(elevatorSub, 34.38456285565362).withTimeout(0.5)
                                                .andThen(new CascadeMagic(elevatorSub, 46.24491881238843)
                                                        .withTimeout(0.5).alongWith(new WaitCommand(0.3))
                                                        .andThen(new ArmMagic(armSub, -30))))));
                                                        
        intakeSequence.whileTrue(new RunCommand(()-> ledSub.TestLED(0, 100, 200),
        ledSub));
        intakeSequence.whileFalse(new RunCommand(()-> ledSub.TestLED(255, 0, 0),
        ledSub));

        cascadeHigh.whileTrue(
                new ParallelCommandGroup(
                        new CascadeMagic(elevatorSub, 56.504),
                        new ArmMagic(armSub, -24.345703125)));
        cascadeHigh.whileTrue(new RunCommand(()-> ledSub.TestLED(0, 100, 200),
        ledSub));
        cascadeHigh.whileFalse(new RunCommand(()-> ledSub.TestLED(255, 0, 0),
        ledSub));
                        
        
        L3Button.whileTrue(
                new ParallelCommandGroup(
                        new CascadeMagic(elevatorSub, 32.293),
                        new ArmMagic(armSub, -37)));

        L3Button.whileTrue(new RunCommand(()-> ledSub.TestLED(0, 200, 100),
        ledSub));
        L3Button.whileFalse(new RunCommand(()-> ledSub.TestLED(255, 0, 0),
        ledSub));

        L2Button.whileTrue(
                new ParallelCommandGroup(
                        new CascadeMagic(elevatorSub, 15),
                        new ArmMagic(armSub, -24.433)));
        L2Button.whileTrue(new RunCommand(()-> ledSub.TestLED(0, 150, 40),
        ledSub));
        L2Button.whileFalse(new RunCommand(()-> ledSub.TestLED(255, 0, 0),
        ledSub));

        elevatorSub.setDefaultCommand(new RunCommand(() -> elevatorSub.ManualMove(-operator.getLeftY() * 0.9), elevatorSub));

        armSub.setDefaultCommand(new RunCommand(() -> armSub.ManualMove(-operator.getRightY() * 0.5), armSub));
        climbSub.setDefaultCommand(new RunCommand(()-> climbSub.Move(operator.getLeftX()), climbSub));

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