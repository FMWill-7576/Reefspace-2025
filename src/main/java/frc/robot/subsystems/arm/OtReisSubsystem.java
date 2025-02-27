package frc.robot.subsystems.arm;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public class OtReisSubsystem extends SubsystemBase{

    private SparkMax shooterMotor = new SparkMax(12, MotorType.kBrushless);
    private SparkMaxConfig config = new SparkMaxConfig();
    public OtReisSubsystem(){
        config
            .inverted(false)
            .idleMode(IdleMode.kCoast);

        shooterMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    public void OtReisSet(double set) {
        shooterMotor.set(set);
    }

    public Command OtReisShooter(){
        return run(()->shooterMotor.set(-1));
    }

    public Command TimedShooter(){
        return OtReisIntake().withTimeout(1);
    }

    public Command OtReisIntake(){
        return run(()->shooterMotor.set(1));
    }

    public Command OtReisStop(){
        return runOnce(()->shooterMotor.set(0));
    }

    public Command OtReisSetCommand(double goal){
        return run(()->shooterMotor.set(goal));
    }
}

