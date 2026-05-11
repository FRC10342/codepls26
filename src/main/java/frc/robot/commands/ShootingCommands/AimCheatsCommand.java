package frc.robot.commands.ShootingCommands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;

import java.util.function.Supplier;

public class AimCheatsCommand extends SequentialCommandGroup {

    public AimCheatsCommand(
       // Supplier<AlignCommand> align,
        Supplier<AimCommand> aim,
        Supplier<ShootCommand> shoot
    ) 
    {
        addCommands(
           // align.get(),
            new ParallelDeadlineGroup(
                aim.get(),
                shoot.get()
            )
        );
    }
}