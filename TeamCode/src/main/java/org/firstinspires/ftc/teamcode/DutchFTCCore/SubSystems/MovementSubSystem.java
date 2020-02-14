package org.firstinspires.ftc.teamcode.DutchFTCCore.SubSystems;

import org.firstinspires.ftc.teamcode.DutchFTCCore.Robot;
import org.firstinspires.ftc.teamcode.DutchFTCCore.Robotconfig;

public class MovementSubSystem extends SubSystem {
    public static MovementSubSystem instance;
    Robot bot;
    /**
     * Movement of the robot on the x axis in a value between -1 and 1;
     */
    public static double xMov;
    /**
     * Movement of the robot on the y axis in a value between -1 and 1;
     */
    public static double yMov;
    /**
     * Rotational input of the robot in a value between -1 and 1;
     */
    public static double rotation;

    @Override
    public void Start() {
        super.Start();
        bot = Robot.instance;
        instance = this;
    }

    @Override
    public void Update() {
        super.Update();
    }

    public void DriveChecksKiwiDrive(){

    }

    public void DriveChecks4WheelTankDrive(){
        //the variables for the motor speeds
        double right = -yMov + rotation;
        double left = yMov + rotation;

        //setting the motor speeds
        bot.MotorBackLeft.setPower(left);
        bot.MotorFrontLeft.setPower(left);
        bot.MotorBackRight.setPower(right);
        bot.MotorFrontRight.setPower(right);

    }

    public void DriveChecksTankDrive(){
        //the variables for the motor speeds
        double right = -yMov + rotation;
        double left = yMov + rotation;

        //setting the motor speeds
        bot.MotorBackLeft.setPower(left);
        bot.MotorBackRight.setPower(right);

    }

    public void DriveChecksHDrive5Motors(){
        double left;
        double right;
        double middle;

    }

    public void DriveChecksHDrive3Motors(){

        //the variables for the motor speeds
        double right = -yMov + rotation;
        double left = yMov + rotation;
        double middle = xMov;

        //setting the motor speeds
        bot.MotorBackLeft.setPower(left);
        bot.MotorBackRight.setPower(right);

    }

    public void DriveChecksMechanum(){

        double stickangle;
        double speed;
        double frontLeftPower;
        double frontRightPower;
        double backLeftPower;
        double backRightPower;

        //radius of circle the stick is in
        speed = (Math.hypot(xMov, yMov));

        //inverse tangent calculate angle of stick
        stickangle = Math.atan2(xMov, yMov);

        //twisting the circle of units 45 degrees
        stickangle -= Math.PI / 4;

        frontLeftPower = (speed * Math.cos(stickangle) + rotation);
        frontRightPower = (speed * Math.sin(stickangle) - rotation);
        backLeftPower = (speed * Math.sin(stickangle) + rotation);
        backRightPower = (speed * Math.cos(stickangle) - rotation);

        bot.MotorFrontLeft.setPower(frontLeftPower);
        bot.MotorFrontRight.setPower(frontRightPower);
        bot.MotorBackLeft.setPower(backLeftPower);
        bot.MotorBackRight.setPower(backRightPower);

    }
}
