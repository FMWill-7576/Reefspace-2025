package frc.robot.commands.elevatorArm;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.elevator.Elevator;

public class elevatorSet extends SequentialCommandGroup{
    public elevatorSet(Elevator elev,double setpoint){
        addCommands(
            new WaitCommand(1),
            elev.setgoal(setpoint)
        );
    }
}
