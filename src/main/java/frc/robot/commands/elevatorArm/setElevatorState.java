package frc.robot.commands.elevatorArm;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.arm.AngleSubsystem;
import frc.robot.subsystems.arm.ArmConstants;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.ElevatorConstants;

public class setElevatorState extends SequentialCommandGroup {
    public setElevatorState(Elevator elev, AngleSubsystem angle,int state) {
        addCommands(

                angle.setAngleCommand(ArmConstants.safeSetpoint)
                        .onlyIf(() -> 
                            !(elev.getCurrentState() == 1 && state == 2) &&
                            !(elev.getCurrentState() == 2 && state == 1)),

                angle.setAngleCommand(ArmConstants.states[1])
                        .onlyIf(() ->
                            ((elev.getCurrentState() == 1 && state == 2) ||
                            (elev.getCurrentState() == 2 && state == 1)
                            )),

                elev.setgoal(ElevatorConstants.states[state - 1]),
                angle.setAngleCommand(ArmConstants.states[state - 1]),

                Commands.runOnce(()->elev.setMultiplier(0.5)).onlyIf(()->state==4),
                Commands.runOnce(()->elev.setMultiplier(1)).onlyIf(()->state!=4)

                );
    }
}
