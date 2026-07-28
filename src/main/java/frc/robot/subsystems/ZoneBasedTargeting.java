package frc.robot.subsystems;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import java.util.Optional;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.util.Units;

public class ZoneBasedTargeting extends SubsystemBase {

    public enum AllianceColor {
    RED,
    BLUE
    }

    public enum Zone {
    OUR_ZONE,
    NEUTRAL_ZONE_1,
    NEUTRAL_ZONE_2,
    OPPONENT_ZONE,
    UNKNOWN
    }

    private static final double FIELD_LENGTH = 651.22; // in inches
    
    /**
     * Gets the current alliance automatically from DriverStation
     */
    private AllianceColor getAlliance() {
    Optional<DriverStation.Alliance> dsAllianceOpt = DriverStation.getAlliance();

    // Use a default if empty
    DriverStation.Alliance dsAlliance = dsAllianceOpt.orElse(DriverStation.Alliance.Blue);

    if (dsAlliance == DriverStation.Alliance.Red) {
        return AllianceColor.RED;
    } else { // BLUE
        return AllianceColor.BLUE;
    }
    }

    /**
     * Determines the current zone of the robot based on its Pose2d
     */
    
    public Zone getCurrentZone(Pose2d robotPose) {
        double x = Units.metersToInches(robotPose.getX());
        double quarter = FIELD_LENGTH / 4.0;

        AllianceColor alliance = getAlliance();

        if (alliance == AllianceColor.BLUE) {
            if (x >= 0 && x < quarter) {
                return Zone.OUR_ZONE;
            } else if (x >= quarter && x < 2 * quarter) {
                return Zone.NEUTRAL_ZONE_1;
            } else if (x >= 2 * quarter && x < 3 * quarter) {
                return Zone.NEUTRAL_ZONE_2;
            } else if (x >= 3 * quarter && x <= FIELD_LENGTH) {
                return Zone.OPPONENT_ZONE;
            }
        } else { // RED alliance: mirror zones
            if (x >= 0 && x < quarter) {
                return Zone.OPPONENT_ZONE;
            } else if (x >= quarter && x < 2 * quarter) {
                return Zone.NEUTRAL_ZONE_2;
            } else if (x >= 2 * quarter && x < 3 * quarter) {
                return Zone.NEUTRAL_ZONE_1;
            } else if (x >= 3 * quarter && x <= FIELD_LENGTH) {
                return Zone.OUR_ZONE;
            }
        }

        return Zone.UNKNOWN;
    }
}