package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "VuileBalkBak", group = "")
public class DrivercontrolledSkyStone extends OpMode {
    DriveTrainMecanum a;
    public Servo bakServo1;
    public Servo bakServo2;
    public double PosServo;
    public double rotation;
    @Override
    public void init() {
        a = new DriveTrainMecanum(hardwareMap.dcMotor.get("MotorBackLeft"),hardwareMap.dcMotor.get("MotorFrontLeft"),hardwareMap.dcMotor.get("MotorBackRight"),hardwareMap.dcMotor.get("MotorFrontRight"),hardwareMap.get(BNO055IMU.class,"imu"));
        bakServo1 = hardwareMap.servo.get("BalkServo1");
        bakServo2 = hardwareMap.servo.get("BalkServo2");
    }

    @Override
    public void loop() {
        if (gamepad1.dpad_down)
        {
            PosServo -= 0.01;
        }
        if (gamepad1.dpad_up){
            PosServo += 0.01;
        }

        bakServo1.setPosition(PosServo);
        bakServo2.setPosition(PosServo);




        rotation += gamepad1.right_trigger;
        rotation -= gamepad1.left_trigger;
        a.rotation = rotation;
        a.xMovement = gamepad1.left_stick_x;
        a.yMovement = gamepad1.left_stick_y;
        a.MoveRotation();
    }
}
