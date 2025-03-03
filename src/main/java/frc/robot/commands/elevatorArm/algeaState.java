package frc.robot.commands.elevatorArm;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.arm.AngleSubsystem;
import frc.robot.subsystems.arm.ArmConstants;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.ElevatorConstants;

public class algeaState extends SequentialCommandGroup{
    public algeaState(Elevator elev, AngleSubsystem angle, int state) {
        addCommands(
            angle.setAngleCommand(ArmConstants.safeSetpoint).onlyIf(()->angle.getEncoder()<ArmConstants.safeSetpoint),
            angle.setAngleCommand(ArmConstants.algeaStates[0]).onlyIf(()->state==1),
            elev.setgoal(ElevatorConstants.algeaStates[state-1]),
            angle.setAngleCommand(ArmConstants.algeaStates[state-1])
        );
    }
}
