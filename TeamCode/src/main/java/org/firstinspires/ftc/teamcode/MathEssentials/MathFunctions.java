package org.firstinspires.ftc.teamcode.MathEssentials;

import java.util.ArrayList;

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
     * Checks for intersections between lineSegment and circle
     * @param circleCenter
     * @param radius
     * @param linePoint1
     * @param linePoint2
     * @return
     */
    public static ArrayList<Vector2> lineCircleIntersection(Vector2 circleCenter, double radius, Vector2 linePoint1, Vector2 linePoint2){

        //make sure we don't get really small line
        /*if (Math.abs(linePoint1.Y - linePoint2.Y) < 0.003){
            linePoint1.Y = linePoint2.Y + 0.003;
        }

        if (Math.abs(linePoint1.X - linePoint2.X) < 0.003){
            linePoint1.X = linePoint2.X + 0.003;
        }*/

        //translating the points with circlecenter
        linePoint1 = circleCenter.subtract(linePoint1);
        linePoint2 = circleCenter.subtract(linePoint2);


        //Calculating as if in origin

        //calcuting line paramets
        double a = (linePoint2.Y - linePoint1.Y)/(linePoint2.Y - linePoint1.Y);
        double b = -1*a*linePoint1.X + linePoint1.Y;

        ArrayList<Vector2> arrayList = new ArrayList<Vector2>();
        double x1 = 0.0;
        double x2 = 0.0;
        double _x1 = (-2*a*b + Math.sqrt(4*a*a*b*b-4*(1+a*a)*(b*b + radius*radius)))/(2+2*a*a);
        if (Double.isInfinite(_x1) || Double.isNaN(_x1) ){
        } else {
            x1 = _x1;
        }
        double _x2 = (-2*a*b - Math.sqrt(4*a*a*b*b-4*(1+a*a)*(b*b + radius*radius)))/(2+2*a*a);
        if (Double.isInfinite(_x2) || Double.isNaN(_x2) ){
        } else {
            x2 = _x2;
        }

        //Check if intersections are on line SEGMENT
        //line moves up
        if (a > 0){
            if (x1 != 0.0 && x1 < linePoint2.X ){
                double y1 = a*x1 + b;
                arrayList.add(new Vector2(x1,y1));
            }
            if( x2 != 0.0 && x2 < linePoint2.X){
                double y2 = a * x2 + b;
                arrayList.add(new Vector2(x2, y2));
            }

        } else {
            if (x1 != 0.0 && x1 < linePoint1.X ){
                double y1 = a*x1 + b;
                arrayList.add(new Vector2(x1,y1));
            }
            if( x2 != 0.0 && x2 < linePoint1.X){
                double y2 = a * x2 + b;
                arrayList.add(new Vector2(x2, y2));
            }
        }
        return arrayList;
    }

}
