package frc.robot.subsystems.elevator;

public class ElevatorConstants {
    

    //Ports
    public static int mainMotorPort = 21;
    public static int secondaryMotorPort = 22;

    //Tuning
    public static double kS = 0.05;
    public static double kG = 0.9;
    public static double kV = 0.05;

    public static String kS_key = "Elevator_kS";
    public static String kG_key = "Elevator_kG";
    public static String kV_key = "Elevator_kV";
    public static String kA_key = "Elevator_kA";

    public static double kP = 0.7;
    public static double kI = 0;
    public static double kD = 4;

    public static double minOutput = -0.4;
    public static double maxOutput = 1;

    //Robot Information

    public static double maxPosition = 4.68;
    public static int smartCurrent = 50;

    //Presets
    public static double[] states = {
        0.0,
        1.224,
        2.4301,
        ElevatorConstants.maxPosition,
    };

    public static double[] algeaStates = {
        0.11804,//home
        1.1759033203125, //shoot
        2.33, //shooter
        3.1787,//max
    };
}
