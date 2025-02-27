package frc.robot.commands.elevatorArm;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.arm.AngleSubsystem;
import frc.robot.subsystems.arm.ArmConstants;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.ElevatorConstants;

public class setElevatorState extends SequentialCommandGroup{
    public setElevatorState(Elevator elev, AngleSubsystem angle, int state) {
        addCommands(
            angle.setAngleCommand(ArmConstants.safeSetpoint),
            elev.setgoal(ElevatorConstants.states[state-1]),
            angle.setAngleCommand(ArmConstants.states[state-1])
        );
    }
}
