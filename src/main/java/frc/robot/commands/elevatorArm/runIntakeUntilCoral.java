package frc.robot.commands.elevatorArm;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.arm.ShooterSubsystem;

public class runIntakeUntilCoral extends SequentialCommandGroup{
    public runIntakeUntilCoral(ShooterSubsystem shooter) {
        addCommands(
            shooter.ShooterShootSpecific(0.77).until(()->shooter.IsCoral()),
            shooter.SlowIntake().withTimeout(0.1)
            //shooter.SlowIntakeOnce(),
            //new WaitCommand(0.5),
            //shooter.OtReisStop()
        );
    }
}
