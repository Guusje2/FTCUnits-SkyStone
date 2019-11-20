package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "DriverControlledTestBot", group = "TestBot")
public class TestBotDriverControlled extends OpMode {
    DriveTrainMecanum a;
    private float rotation;

    @Override
    public void init() {
        a = new DriveTrainMecanum(hardwareMap.dcMotor.get("MotorBackLeft"),hardwareMap.dcMotor.get("MotorFrontLeft"),hardwareMap.dcMotor.get("MotorBackRight"),hardwareMap.dcMotor.get("MotorFrontRight"),hardwareMap.get(BNO055IMU.class,"imu"));
    }

    @Override
    public void loop() {
        rotation += gamepad1.right_trigger;
        rotation -= gamepad1.left_trigger;
        a.rotation = rotation;
        a.xMovement = gamepad1.left_stick_y;
        a.yMovement = gamepad1.left_stick_x;
        a.MoveRotation();
    }
}
