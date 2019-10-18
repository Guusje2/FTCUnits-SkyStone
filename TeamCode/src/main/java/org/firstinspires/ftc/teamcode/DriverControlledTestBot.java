package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "DriverControlled1stick", group = "testbot")
public class DriverControlledTestBot extends OpMode {
    public DriveTrainMecanumEncoder drivetrain;
    public void init() {
        drivetrain = new DriveTrainMecanumEncoder(hardwareMap.dcMotor.get("MotorBackLeft"), hardwareMap.dcMotor.get("MotorBackRight"), hardwareMap.dcMotor.get("MotorFrontLeft"), hardwareMap.dcMotor.get("MotorFrontRight"), hardwareMap.get(BNO055IMU.class, "imu"));
    }

    public void loop() {
        drivetrain.UpdatePos();
        drivetrain.yMovement = gamepad1.left_stick_y;
        drivetrain.xMovement = gamepad1.left_stick_x;
    }
}
