package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

@Autonomous(name = "StopDemo")
public class StopDemo extends LinearOpMode {
    public DcMotor BackLeft;
    public DcMotor FrontLeft;
    public DcMotor BackRight;
    public DcMotor FrontRight;

    @Override
    public void runOpMode() throws InterruptedException {
        BackLeft = hardwareMap.dcMotor.get("MotorBackLeft");
        FrontLeft = hardwareMap.dcMotor.get("MotorFrontLeft");
        BackRight = hardwareMap.dcMotor.get("MotorBackRight");
        FrontRight = hardwareMap.dcMotor.get("MotorFrontRight");

        waitForStart();

        while (opModeIsActive()) {
            BackRight.setPower(1);
            FrontRight.setPower(1);
            BackLeft.setPower(-1);
            FrontLeft.setPower(-1);
        }
        BackRight.setPower(0);
        FrontRight.setPower(0);
        BackLeft.setPower(0);
        FrontLeft.setPower(0);
    }
}
