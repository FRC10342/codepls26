package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;

public final class Constants {

    private Constants() {}

    public static final class FieldConstants {

        public static final Pose2d BLUE_HUB_CENTER =
                new Pose2d(4.65, 4.0, new Rotation2d(-Math.PI));

        public static final Pose2d RED_HUB_CENTER =
                new Pose2d(12, 4.0, new Rotation2d(0));

        public static Pose2d getHubCenter() {
            var alliance = DriverStation.getAlliance();

            if (alliance.isPresent()
                    && alliance.get() == DriverStation.Alliance.Red) {
                return RED_HUB_CENTER;
            }
            return BLUE_HUB_CENTER;
        }
    }
}
