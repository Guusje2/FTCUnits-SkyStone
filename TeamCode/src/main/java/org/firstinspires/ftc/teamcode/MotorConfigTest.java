package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name = "MotorConfigTest")
public class MotorConfigTest extends OpMode {
    public DcMotor BackLeft;
    public DcMotor FrontLeft;
    public DcMotor BackRight;
    public DcMotor FrontRight;


    @Override
    public void init() {
        BackLeft = hardwareMap.dcMotor.get("MotorBackLeft");
        FrontLeft = hardwareMap.dcMotor.get("MotorFrontLeft");
        BackRight = hardwareMap.dcMotor.get("MotorBackRight");
        FrontRight = hardwareMap.dcMotor.get("MotorFrontRight");
    }

    @Override
    public void loop() {
        if (gamepad1.a){
            BackLeft.setPower(1);
        } else {
            BackLeft.setPower(0);
        }
        if (gamepad1.b){
            FrontLeft.setPower(1);
        } else {
            FrontLeft.setPower(0);
        }if (gamepad1.x){
            BackRight.setPower(1);
        } else {
            BackRight.setPower(0);
        }if (gamepad1.y){
            FrontRight.setPower(1);
        } else {
            FrontRight.setPower(0);
        }
    }
}
