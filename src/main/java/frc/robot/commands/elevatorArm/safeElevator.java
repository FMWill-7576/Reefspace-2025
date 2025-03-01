package frc.robot.commands.elevatorArm;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.arm.AngleSubsystem;
import frc.robot.subsystems.arm.ArmConstants;
import frc.robot.subsystems.arm.ShooterSubsystem;
import frc.robot.subsystems.elevator.Elevator;

public class safeElevator extends SequentialCommandGroup
{
    public safeElevator(Elevator elevator,AngleSubsystem angle,double goal) {
        addCommands(
           angle.setAngleCommand(0.6),
           elevator.setgoal(goal)
        );
    }
}
