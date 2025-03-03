package frc.robot.subsystems.climb;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ClimbSubsystem extends SubsystemBase{
    private SparkMax motor = new SparkMax(0, MotorType.kBrushless);
    private SparkMaxConfig motor_config = new SparkMaxConfig();

    private double climbValue = 0.1;

    public ClimbSubsystem() {
        motor_config
            .inverted(false)
            .idleMode(IdleMode.kBrake);

        motor.configure(motor_config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    public void SetMotor(double set) {
        motor.set(set);
    }

    public Command ClimbIdle() {
        return run(()->SetMotor(0));
    }

    public Command ClimbUp() {
        return run(()->SetMotor(climbValue));
    }

    public Command ClimbDown() {
        return run(()->SetMotor(-climbValue));
    }
}
