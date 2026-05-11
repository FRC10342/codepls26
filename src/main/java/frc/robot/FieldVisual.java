package frc.robot;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.FieldObject2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.Odometry;

public class FieldVisual {
    

    public final Field2d field = new Field2d();
    private final CommandSwerveDrivetrain drivetrain;
    public final FieldObject2d pathObj = field.getObject("path");

    double testX = 0;

    public FieldVisual(CommandSwerveDrivetrain drivetrain) {
        this.drivetrain = drivetrain;
    
        SmartDashboard.putData("Field", field);

        // Draw hub centers once
        field.getObject("Blue Hub")
                .setPose(Constants.FieldConstants.BLUE_HUB_CENTER);

        field.getObject("Red Hub")
                .setPose(Constants.FieldConstants.RED_HUB_CENTER);
    }

    // Call this from Robot.periodic()
    public void update() {
        
        Pose2d robotPose = drivetrain.getState().Pose;
        
        if (robotPose != null) {
            field.setRobotPose(robotPose);
        }
        field.setRobotPose(robotPose);

        SmartDashboard.putString(
            "RobotPose",
            drivetrain.getState().Pose.toString()
        );
    }

    public FieldObject2d getPathObj() {
        return pathObj;
    }
    public Field2d getField() {
        return field;
    }
    
}

