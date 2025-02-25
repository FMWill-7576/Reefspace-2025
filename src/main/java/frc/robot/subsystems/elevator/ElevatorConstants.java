package frc.robot.subsystems.elevator;

public class ElevatorConstants {
    

    //Ports
    public static int mainMotorPort = 21;
    public static int secondaryMotorPort = 22;

    //Tuning
    public static double kS = 0.05;
    public static double kG = 0.2;
    public static double kV = 0.2   ;

    public static String kS_key = "Elevator_kS";
    public static String kG_key = "Elevator_kG";
    public static String kV_key = "Elevator_kV";
    public static String kA_key = "Elevator_kA";

    public static double kP = 0.9;
    public static double kI = 0;
    public static double kD = 1.2;

    public static double minOutput = -0.6;
    public static double maxOutput = 0.95;

    //Robot Information

    public static double maxPosition = 4.68;
    public static int smartCurrent = 50;

    //Presets
    public static double[] states = {
        0.0,
        1.5,
        3.0,
        4.4,
    };
}
