package frc.robot.commands.elevatorArm;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.arm.AngleSubsystem;
import frc.robot.subsystems.arm.ArmConstants;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.ElevatorConstants;

public class algaeChaos extends SequentialCommandGroup{
    public algaeChaos(Elevator elev, AngleSubsystem angle) {
        double angleSetpoint = 0;
        double elevatorSetpoint = 0;
        addCommands(
            angle.setAngleCommand(ArmConstants.safeSetpoint),
            elev.setgoal(elevatorSetpoint),
            angle.setAngleCommand(angleSetpoint)
        );
    }
}
