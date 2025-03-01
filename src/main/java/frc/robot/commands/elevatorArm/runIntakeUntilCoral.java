package frc.robot.commands.elevatorArm;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.arm.ShooterSubsystem;

public class runIntakeUntilCoral extends SequentialCommandGroup{
    public runIntakeUntilCoral(ShooterSubsystem shooter) {
        addCommands(
            shooter.OtReisShooter().until(()->shooter.IsCoral())
        );
    }
}
