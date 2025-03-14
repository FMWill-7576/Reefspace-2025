package frc.robot.subsystems.arm;

import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.units.measure.Distance;
import frc.robot.subsystems.elevator.ElevatorConstants;

public class ArmConstants {
    //meters
    public static final double kArmLength = 0.2;
    public static final double  kArmReduction = 20;

    public static final int kAnglePort = 11;
    public static final int kShooterPort = 12;

    public static double kAngleRatio = 43.6;

    public static double homeForwardLimit = 0.8;

    public static double homeSetpoint = 0;
    public static double safeSetpoint = 0.513;
    public static double homeToSecondState = 0.2109375;

    //Tuning
    public static double kG = 0.2;
    public static double kV = 0;
    //public static double kA = 0;
    public static double kS = 0;
    double[] setpoints = {
        0,
    };

    //Presets
    public static double[] states = {
        homeSetpoint,
        0.2,
        0.178701,
        0.2,
    };
    

    public static double[] algeaStates = {
        0.61767, //home
        1.15087, //shooter
        1.01708, //second state
        0.909, //max
    };
    
}
