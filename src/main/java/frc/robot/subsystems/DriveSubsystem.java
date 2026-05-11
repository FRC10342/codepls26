    package frc.robot.subsystems;

    import com.pathplanner.lib.auto.AutoBuilder;
    import com.pathplanner.lib.config.PIDConstants;
    import com.pathplanner.lib.config.RobotConfig;
    import com.pathplanner.lib.controllers.PPHolonomicDriveController;

    import edu.wpi.first.wpilibj2.command.SubsystemBase;
    import edu.wpi.first.wpilibj.DriverStation;


    public class DriveSubsystem extends SubsystemBase {
        private final CommandSwerveDrivetrain drivetrain;
        /** 
         * @param drivetrain
         */
        public DriveSubsystem(CommandSwerveDrivetrain drivetrain) {
            this.drivetrain = drivetrain;
            
            RobotConfig config;
            try {
                config = RobotConfig.fromGUISettings();
            } catch (Exception e) {
                throw new RuntimeException(e);
            }
            AutoBuilder.configure(
                drivetrain::getCurrentPose,
                drivetrain::resetPose,
                drivetrain::getRobotRelativeSpeeds,
                (speeds, feedForwards) -> drivetrain.applyRobotSpeeds(speeds),
                new PPHolonomicDriveController(
                    new PIDConstants(8.0, 0, 0),
                    new PIDConstants(8.0, 0, 0)
                ),
                config,
                () -> DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue)
                        == DriverStation.Alliance.Red,
                this
            );
        }     
    }