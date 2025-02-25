package frc.robot.commands.elevatorArm;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.arm.AngleSubsystem;
import frc.robot.subsystems.elevator.Elevator;

public class safeElevator extends SequentialCommandGroup
{
    public safeElevator(Elevator elevator,AngleSubsystem angle,double goal) {
        addCommands(
            //angle.setArmSafe(),
            elevator.setgoal(goal)
        );
    }
}
