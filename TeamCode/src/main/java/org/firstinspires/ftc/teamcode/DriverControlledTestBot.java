package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.MathEssentials.Vector2;

@TeleOp(name = "Guus", group = "testbot")
public class DriverControlledTestBot extends OpMode {
    public DriveTrainMecanumEncoder drivetrain;
    public void init() {
        drivetrain = new DriveTrainMecanumEncoder(hardwareMap.dcMotor.get("MotorBackLeft"), hardwareMap.dcMotor.get("MotorBackRight"), hardwareMap.dcMotor.get("MotorFrontLeft"), hardwareMap.dcMotor.get("MotorFrontRight"), hardwareMap.get(BNO055IMU.class, "imu"));
    }

    public void loop() {
        drivetrain.UpdatePos();
        double left;
        double right;
        double front;

        left = gamepad1.left_stick_y;
        front = gamepad1.left_stick_x;

        drivetrain.MoveToPos(new Vector2(left*358,front*358),1);
    }
}
