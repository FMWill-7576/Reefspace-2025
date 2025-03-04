package frc.robot;

public class eUtil {
    public static double centimeterToMeter(double c){
        return c/100;
    }

    public static double meterToCentimeter(double c){
        return c*100;
    }

    public static Boolean isIntExistsInArray(int v, int[] t){
        boolean exist = false;
        for (int i : t) {
            if(v==i){
                exist = true;
                break;
            }
        }
        return exist;
    }
    public static double map(double x, double inMin, double inMax, double outMin, double outMax){
        return (x-inMin)*(outMax-outMin)/(inMax-inMin)+outMin;
    }
}