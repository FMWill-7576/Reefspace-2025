package frc.robot.subsystems.elevator;

public class ElevatorConstants {
    

    //Ports
    public static int mainMotorPort = 21;
    public static int secondaryMotorPort = 22;

    //Tuning
    public static double kS = 0;
    public static double kG = 0.7;
    public static double kV = 0;
    public static double kA = 0.05;

    public static String kS_key = "Elevator_kS";
    public static String kG_key = "Elevator_kG";
    public static String kV_key = "Elevator_kV";
    public static String kA_key = "Elevator_kA";

    public static double kP = 1.2;
    public static double kI = 0;
    public static double kD = 1;


    //Controls
    public static double kMaxVelocity = 1;
    public static double kMaxAcceleration = 1;

    public static double minOutput = -0.8;
    public static double maxOutput = 0.95;

    //Robot Information

    public static double maxPosition = 4.5;
    public static int smartCurrent = 50;

    //Presets
    public static double[] states = {
        0,
        1.5,
        3,
        4.4,
    };
}
