package frc.robot.commands.elevatorArm;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.arm.AngleSubsystem;
import frc.robot.subsystems.elevator.Elevator;

public class safeElevator extends ParallelCommandGroup
{
    public safeElevator(Elevator elevator,AngleSubsystem angle,double goal) {
        addCommands(
            new armSet(angle, 0.61),
            new elevatorSet(elevator, goal)
        );
    }
}
