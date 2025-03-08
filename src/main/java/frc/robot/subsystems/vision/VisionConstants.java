package frc.robot.subsystems.vision;

import java.util.Dictionary;
import java.util.Hashtable;
import java.util.Enumeration;
import java.util.HashMap;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.util.Units;
import frc.robot.eUtil;

public class VisionConstants {
    public static double kP = 0;
    public static double kI = 0;
    public static double kD = 0;

    public static String kP_string = "kP_vision";
    public static String kI_string = "kI_vision";
    public static String kD_string = "kD_vision";

    public static double pitch = 2;
    public static double yaw = -18;
    public static double height = 0;

    //System

    public static double cameraHeigthtMeters = 1;
    public static double cameraPitchInRadians = Units.degreesToRadians(30);

    //delete 12 afterward!
    public static int[] reefIDs = {
        6,7,8,9,10,11,

        17,18,19,20,21,22
    };


    //Allignment
    public static double leftArea = 14.870;
    public static double leftHorizontalOffset = 16.05;

    public static double rightArea = 6.080;
    public static double rightHorizontalOffset = 19.22;

    public static double areaTolerance = 0.4;
    public static double horizontalTolerance = 0.7;

    public static double aprilHeightInfo(int key){
        HashMap<Integer, Double> h = new HashMap<>();
        
        h.put(6, eUtil.centimeterToMeter(17));
        h.put(7, eUtil.centimeterToMeter(17));
        h.put(8, eUtil.centimeterToMeter(17));
        h.put(9, eUtil.centimeterToMeter(17));
        h.put(10, eUtil.centimeterToMeter(17));
        h.put(11, eUtil.centimeterToMeter(17));

        //delete later
        h.put(12, eUtil.centimeterToMeter(17));

        h.put(17, eUtil.centimeterToMeter(17));
        h.put(18, eUtil.centimeterToMeter(17));
        h.put(19, eUtil.centimeterToMeter(17));
        h.put(20, eUtil.centimeterToMeter(17));
        h.put(21, eUtil.centimeterToMeter(17));
        h.put(22, eUtil.centimeterToMeter(17));
        


        return h.get(key);
    }
}
