package frc.robot.commands.elevatorArm;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.arm.AngleSubsystem;
import frc.robot.subsystems.arm.ArmConstants;

public class armSet extends SequentialCommandGroup{
    public armSet(AngleSubsystem angle, double set) {
        addCommands(
            angle.setAngleCommand(ArmConstants.homeSetpoint)
        );
    }
}
