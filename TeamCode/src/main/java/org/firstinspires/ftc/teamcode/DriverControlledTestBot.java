package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import java.io.IOException;

@TeleOp(name = "DriverControlled1stick", group = "testbot")
public class DriverControlledTestBot extends OpMode {
    public DriveTrainMecanumEncoder drivetrain;
    public void init() {
        try {
            drivetrain = new DriveTrainMecanumEncoder(hardwareMap.dcMotor.get("MotorBackLeft"), hardwareMap.dcMotor.get("MotorBackRight"), hardwareMap.dcMotor.get("MotorFrontLeft"), hardwareMap.dcMotor.get("MotorFrontRight"), hardwareMap.get(BNO055IMU.class, "imu"));
        } catch (IOException e) {
            e.printStackTrace();
        }
    }

    public void loop() {
        drivetrain.yMovement = gamepad1.left_stick_y;
        drivetrain.xMovement = gamepad1.left_stick_x;


        drivetrain.rotation = 0;
            drivetrain.rotation -= gamepad1.left_trigger;

            drivetrain.rotation += gamepad1.right_trigger;


        telemetry.addData("Rotation", drivetrain.rotation);
        telemetry.update();
        drivetrain.MoveRotation();
    }
}
