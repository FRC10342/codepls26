// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.commands.ShootingCommands.AlignTest;
import frc.robot.commands.IntakeCommand;
import frc.robot.commands.IntakeHingeCommand;
import frc.robot.commands.AutoCommands.IntakeAutoCommand;
//import frc.robot.commands.AutoCommands.ShootAutoCommand;
import frc.robot.commands.ShootingCommands.AimCommand;
import frc.robot.commands.ShootingCommands.AimCommandDown;
import frc.robot.commands.ShootingCommands.AlignCommand;
import frc.robot.commands.ShootingCommands.ShootCommand;
import frc.robot.commands.AgitateRollerTest;
import frc.robot.commands.DumpIntakeCommand;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.PitchSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.subsystems.ZoneBasedTargeting;
import frc.robot.commands.ShootingCommands.PitchTestCommand;
import frc.robot.commands.ShootingCommands.AimCommandDown;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;


public class RobotContainer {


    private double MaxSpeed = 1.0 * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

    private final Telemetry logger = new Telemetry(MaxSpeed);

    public final CommandXboxController joystick = new CommandXboxController(0);

    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();

    private final VisionSubsystem vision;

    private final ZoneBasedTargeting zone;

    private final AlignCommand align;

    private final AimCommand aim;

    private final ShootCommand shoot;

    // Hold-to-score: runs align (yaw) and aim (pitch) continuously, and only starts
    // feeding balls once the drivetrain is actually on-target.
    private final Command alignAimShoot;
    
    private final ShooterSubsystem shootvar;

    private final IntakeCommand intake;

   private final DriveSubsystem auto;

    private final AgitateRollerTest agitate;

    private final DumpIntakeCommand dump;

    private final SendableChooser<Command> autoChooser;

    private final PitchSubsystem pitchvar;

    private final AlignTest aligntest;

    private final AimCommandDown aimCommandDown;

    //private final ShootAutoCommand autoShoot;

    private final IntakeAutoCommand intakeAuto;

    //private final PitchSupplier pitchsup;

    public RobotContainer() {
        vision = new VisionSubsystem(drivetrain);
        zone = new ZoneBasedTargeting();
        shootvar = new ShooterSubsystem();
        pitchvar = new PitchSubsystem();
        aim = new AimCommand(drivetrain, vision, pitchvar);
        align = new AlignCommand(vision, drivetrain, zone);
        intake = new IntakeCommand(shootvar);
        auto = new DriveSubsystem(drivetrain);
        agitate = new AgitateRollerTest(shootvar);
        dump = new DumpIntakeCommand(shootvar);
        aligntest = new AlignTest(vision, drivetrain);
        //autoShoot = new ShootAutoCommand(shootvar);
        shoot = new ShootCommand(shootvar);
        intakeAuto = new IntakeAutoCommand(shootvar);
        aimCommandDown = new AimCommandDown(pitchvar);

        alignAimShoot = Commands.parallel(
            align,
            aim,
            Commands.waitUntil(align::isAligned).andThen(shoot)
        );

        NamedCommands.registerCommand("IntakeCommand", new IntakeAutoCommand(shootvar).withTimeout(2)); //Placeholder
        //NamedCommands.registerCommand("AutoShoot", new ShootAutoCommand(shootvar).withTimeout(3)); //Placeholder
        NamedCommands.registerCommand("AutoShoot", new ShootCommand(shootvar).withTimeout(3)); //Placeholder
       // NamedCommands.registerCommand("FarAutoPitchShoot", new PitchTestCommand(pitchvar, 0.25));


        autoChooser = AutoBuilder.buildAutoChooser();

        SmartDashboard.putData("Auto Chooser", autoChooser);
        
        configureBindings(); 
    }

    public CommandSwerveDrivetrain getDrivetrain() {
        return drivetrain;
    }

    private void configureBindings() {
        System.out.println(MaxSpeed);
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        drivetrain.setDefaultCommand(
        drivetrain.applyRequest(() -> {
        return drive.withVelocityX(-joystick.getLeftY() * MaxSpeed)
                    .withVelocityY(-joystick.getLeftX() * MaxSpeed)
                    .withRotationalRate(-joystick.getRightX() * MaxAngularRate);
        })
        );

        // Idle while the robot is disabled. This ensures the configured
        // neutral mode is applied to the drive motors while disabled.
        final var idle = new SwerveRequest.Idle();
        RobotModeTriggers.disabled().whileTrue(
            drivetrain.applyRequest(() -> idle).ignoringDisable(true)
        );

        joystick.a().whileTrue(drivetrain.applyRequest(() -> brake));
        //joystick.b().whileTrue(drivetrain.applyRequest(() ->
        //    point.withModuleDirection(new Rotation2d(-joystick.getLeftY(), -joystick.getLeftX()))
        //));

        // Run SysId routines when holding back/start and X/Y.
        // Note that each routine should be run exactly once in a single log.
        //joystick.back().and(joystick.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        //joystick.back().and(joystick.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        //joystick.start().and(joystick.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        //joystick.start().and(joystick.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

        // Reset the field-centric heading on left bumper press.
        joystick.leftBumper().onTrue(drivetrain.runOnce(drivetrain::seedFieldCentric));

        // change to command sequence
        joystick.x().whileTrue(
           shoot
        );

        joystick.leftTrigger().whileTrue(aimCommandDown);

        // Hold to auto-score: align yaw to the hub, aim pitch, and shoot once on-target.
        joystick.rightTrigger().whileTrue(alignAimShoot);

        // Side-project continuous AprilTag tracking test (see AlignTest) - never bound before.
        joystick.b().whileTrue(aligntest);

        //joystick.b().onTrue(new PitchDownSup(pitchsup::getPitch));

       // joystick.x().onTrue(pitchsup);

        //joystick.b().onTrue(aim);
        joystick.y().whileTrue(new IntakeCommand(shootvar));

        // povRight, povUp, povDown are free - IntakeHingeCommandUp/ClimbCommand/ClimbDownCommand
        // were removed as unnecessary for the new robot.

        joystick.povLeft().onTrue(new IntakeHingeCommand(shootvar).withTimeout(0.25));

        joystick.rightBumper().whileTrue(agitate);//backwards roller

        // Eject everything (was built but never bound to anything). Rebind if Back is wanted for something else.
        joystick.back().whileTrue(dump);

        drivetrain.registerTelemetry(logger::telemeterize);

        //joystick.start().whileTrue(new InstantCommand(() -> System.out.println("Robot Good")));

    }

    public Command getAutonomousCommand() {
        Command auto = autoChooser.getSelected();
        if (auto == null) auto = new InstantCommand();

        auto = new SequentialCommandGroup(
            auto,
            new WaitCommand(5.0) // always wait 5 seconds
        );
        return auto;
}
}

