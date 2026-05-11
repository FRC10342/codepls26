// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.IOException;
import java.util.ArrayList;
import java.util.List;
import java.util.Objects;
import java.util.stream.Collectors;

import org.json.simple.parser.ParseException;

import com.ctre.phoenix6.HootAutoReplay;
import com.ctre.phoenix6.Utils;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.trajectory.PathPlannerTrajectory;
import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.Odometry;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.FieldObject2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.FieldVisual;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.PitchSubsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.RobotContainer;

public class Robot extends TimedRobot {

    private String autoName = "";

    private String newAutoName = "";

    private Command m_autonomousCommand;

    private final RobotContainer m_robotContainer;

    private FieldVisual fieldvisual;

    private CommandSwerveDrivetrain drivetrain;

    private ShooterSubsystem shooter;

    private PitchSubsystem pitch;


    /* log and replay timestamp and joystick data */
    private final HootAutoReplay m_timeAndJoystickReplay = new HootAutoReplay()
        .withTimestampReplay()
        .withJoystickReplay();

    public Robot() {
        m_robotContainer = new RobotContainer();
        this.drivetrain = m_robotContainer.getDrivetrain();
        this.fieldvisual = new FieldVisual(drivetrain);
        this.shooter = new ShooterSubsystem();
        this.pitch = new PitchSubsystem();
    }

    @Override
    public void robotPeriodic() {
        CommandScheduler.getInstance().run(); 
        m_timeAndJoystickReplay.update();
        fieldvisual.update();

        SmartDashboard.putNumber("climb position!!!", shooter.getClimbMotor().getPosition().getValueAsDouble());

    }

    @Override
    public void disabledInit() {}

@Override
public void disabledPeriodic() {
    /*fieldvisual.update();
    // Get the currently selected autonomous command
    Command auto = m_robotContainer.getAutonomousCommand();
    if (auto == null) {
        return; // no auto selected
    }

    // Get the name of the auto
    String newAutoName = auto.getName();

    // Only update if the auto selection changed
    if (!Objects.equals(autoName, newAutoName)) {
        autoName = newAutoName;
        System.out.println("Displaying auto: " + autoName);

        try {
            // Load all paths from the PathPlanner auto file
            List<PathPlannerPath> paths = PathPlannerAuto.getPathGroupFromAutoFile(autoName);

            // Get the persistent FieldObject2d from FieldVisual
            FieldObject2d pathObj = fieldvisual.getPathObj();
            if (pathObj == null) {
                System.err.println("pathObj is null");
                return;
            }

            // Collect ALL poses from all paths into a single list
            List<Pose2d> allTrajectoryPoses = new ArrayList<>();

            for (PathPlannerPath path : paths) {
                // Use the path points you already used before
                var points = path.getAllPathPoints(); // returns a List of PathPoint (or similar)
                if (points == null || points.isEmpty()) {
                    continue;
                }

                for (int i = 0; i < points.size(); i++) {
                    var point = points.get(i);

                    // Extract X/Y from the point's position (Translation2d)
                    double x = point.position.getX();
                    double y = point.position.getY();

                    // Try to obtain a rotation from the point if available.
                    // If not available, compute heading from the next point (tangent).
                    Rotation2d rot = null;

                    // 1) Common PathPlanner field: holonomicRotation (Rotation2d)
                    try {
                        // Many PathPlanner versions expose a public field named holonomicRotation
                        java.lang.reflect.Field f = point.getClass().getField("holonomicRotation");
                        Object val = f.get(point);
                        if (val instanceof Rotation2d) {
                            rot = (Rotation2d) val;
                        }
                    } catch (NoSuchFieldException | IllegalAccessException ignored) {
                        // field not present or inaccessible; fall through
                    }

                    // 2) Some versions have a getter getHolonomicRotation()
                    if (rot == null) {
                        try {
                            java.lang.reflect.Method m = point.getClass().getMethod("getHolonomicRotation");
                            Object val = m.invoke(point);
                            if (val instanceof Rotation2d) {
                                rot = (Rotation2d) val;
                            }
                        } catch (NoSuchMethodException | IllegalAccessException | java.lang.reflect.InvocationTargetException ignored) {
                            // no getter; fall through
                        }
                    }

                    // 3) Some versions expose a heading angle in degrees or radians (try common names)
                    if (rot == null) {
                        try {
                            java.lang.reflect.Field f = point.getClass().getField("heading"); // degrees?
                            Object val = f.get(point);
                            if (val instanceof Double) {
                                rot = Rotation2d.fromDegrees((Double) val);
                            } else if (val instanceof Float) {
                                rot = Rotation2d.fromDegrees(((Float) val).doubleValue());
                            }
                        } catch (NoSuchFieldException | IllegalAccessException ignored) {
                        }
                    }

                    // 4) Fallback: compute heading from next point (tangent). If last point, reuse previous rotation or zero.
                    if (rot == null) {
                        if (i < points.size() - 1) {
                            var next = points.get(i + 1);
                            double nx = next.position.getX();
                            double ny = next.position.getY();
                            double dx = nx - x;
                            double dy = ny - y;
                            if (Math.hypot(dx, dy) > 1e-6) {
                                rot = new Rotation2d(Math.atan2(dy, dx));
                            } else {
                                rot = new Rotation2d(0.0);
                            }
                        } else if (!allTrajectoryPoses.isEmpty()) {
                            // reuse last rotation if available
                            rot = allTrajectoryPoses.get(allTrajectoryPoses.size() - 1).getRotation();
                        } else {
                            rot = new Rotation2d(0.0);
                        }
                    }

                    allTrajectoryPoses.add(new Pose2d(x, y, rot));
                }
            }

            // Update FieldObject2d with all poses at once
            pathObj.setPoses(allTrajectoryPoses);

            System.out.println("Total path points loaded: " + allTrajectoryPoses.size());
            if (!allTrajectoryPoses.isEmpty()) {
                System.out.println("Sample pose[0]: " + allTrajectoryPoses.get(0));
            }

            System.out.println("Auto name: " + autoName);
System.out.println("Paths loaded: " + (paths == null ? "null" : paths.size()));
for (int p = 0; p < paths.size(); p++) {
    System.out.println("  path[" + p + "] points: " + paths.get(p).getAllPathPoints().size());
}


        } catch (IOException | ParseException e) {
            e.printStackTrace();
        }
    }*/
}




    @Override
    public void disabledExit() {}

    @Override
    public void autonomousInit() {
        m_autonomousCommand = m_robotContainer.getAutonomousCommand();
    System.out.println("Auto command: " + m_autonomousCommand);

    if (m_autonomousCommand != null) {
        if (Utils.isSimulation()) {
            // In sim, run auto and follow speeds independently so Glass doesn't close
            m_autonomousCommand.schedule();
            drivetrain.followAutoSpeeds().schedule();
        } else {
            // Real robot: race normally
            m_autonomousCommand
                .raceWith(drivetrain.followAutoSpeeds())
                .andThen(drivetrain::stopAutoSpeeds)
                .schedule();
        }
    }
    }

    @Override
    public void autonomousPeriodic() {
        //System.out.println("AutoWorking");
        fieldvisual.update();
    }

    @Override
    public void autonomousExit() {}

    @Override
    public void teleopInit() {
        shooter.getClimbMotor().setPosition(0);
        pitch.getPitchMotor().setPosition(0);
        if (m_autonomousCommand != null) {
            CommandScheduler.getInstance().cancel(m_autonomousCommand);
        }
        SmartDashboard.putString("robotStatus", "i luv u Orion");
    }

    @Override
    public void teleopPeriodic() {}

    @Override
    public void teleopExit() {}

    @Override
    public void testInit() {
        CommandScheduler.getInstance().cancelAll();
    }

    @Override
    public void testPeriodic() {}

    @Override
    public void testExit() {}

    @Override
    public void simulationPeriodic() {
        double dt = 0.02; // 20 ms loop
        //drivetrain.updateSimState(dt);
        fieldvisual.update();  // so the Field2d moves in Glass
    }

}



