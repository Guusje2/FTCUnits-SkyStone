package org.firstinspires.ftc.teamcode.MathEssentials;

public class MathFunctions {


    /**
    Checks if a value is within a certain range
     */
    public static boolean Ish (double value, double range, double checkvalue){
        if ( checkvalue - range < value && value < checkvalue + range) {
            return  true;
        } else {
            return false;
        }
    }

    /**Input any angle , and returns it between 180 and -180
     *
     * @param angle
     * @return between -180 and 180
     */
    public static double FixAngle (double angle){
        while(angle < -180){
            angle += 360;
        }
        while (angle > 180) {
            angle -= 360;
        }
        return angle;
    }

    /**
     * returns an angle between -PI/2 and PI/2
     */
    public static double FixAngleRad (double angle){
        while (angle <= -Math.PI/2){
            angle += Math.PI/2;
        }
        while (angle >= Math.PI/2){
            angle -= Math.PI/2;
        }
        return angle;
    }

    /**
     * always returns an angle
     * @param angle in degrees
     */
    /*
    public static double fixedaSin (double angle) {
        double angleRad = Math.toRadians(angle);
        if(angleRad > .5*Math.PI() && angleRad < Math.PI) {
            return Math.sin()
        }
    }*/
}
