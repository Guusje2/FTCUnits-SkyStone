package org.firstinspires.ftc.teamcode.MathEssentials;

public class CurvePoint {
    public double x;
    public double y;
    public double moveSpeed;
    public double turnSpeed;
    public double followDistance;
    public double pointLength;
    public double slowDownTurnDegrees;
    public double slowDownTurnAmount;

    public  CurvePoint(double x, double y, double moveSpeed, double turnSpeed, double followDistance, double slowDownTurnDegrees, double slowDownTurnAmount){

        this.x = x;
        this.y = y;
        this.moveSpeed = moveSpeed;
        this.turnSpeed = turnSpeed;
        this.followDistance = followDistance;
        this.slowDownTurnDegrees = slowDownTurnDegrees;
        this.slowDownTurnAmount = slowDownTurnAmount;
    }

    public CurvePoint (CurvePoint thisPoint){
        x = thisPoint.x;
        y = thisPoint.y;
        moveSpeed = thisPoint.moveSpeed;
        turnSpeed = thisPoint.turnSpeed;
        followDistance = thisPoint.followDistance;
        slowDownTurnAmount = thisPoint.slowDownTurnAmount;
        slowDownTurnDegrees = thisPoint.slowDownTurnDegrees;
        pointLength = thisPoint.pointLength;
    }

    public Vector2 toVector2() {
        Vector2 a = new Vector2(x,y);
        return  a;
    }

    public void setVector2(Vector2 point) {
        x = point.X;
        y = point.Y;
    }
}
