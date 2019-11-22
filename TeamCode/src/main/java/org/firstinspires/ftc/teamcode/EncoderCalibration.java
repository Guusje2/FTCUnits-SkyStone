package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import java.io.IOException;

@TeleOp(name = "EncoderCalibration")
public class EncoderCalibration extends OpMode {
    public DcMotor MotorBackLeft;
    public DcMotor MotorFrontLeft;
    public DcMotor MotorFrontRight;
    public DcMotor MotorBackRight;

    @Override
    public void init() {
        MotorBackLeft = hardwareMap.dcMotor.get("MotorBackLeft");
        MotorFrontLeft = hardwareMap.dcMotor.get("MotorFrontLeft");
        MotorFrontRight = hardwareMap.dcMotor.get("MotorFrontRight");
        MotorBackRight = hardwareMap.dcMotor.get("MotorBackRight");

        //setting zero power behavior
        MotorBackRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        MotorBackLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        MotorFrontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        MotorFrontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        MotorFrontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        MotorBackLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        MotorFrontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        MotorBackLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


        try {
            logUtils.StartLogging(1);
        } catch (IOException e) {
            e.printStackTrace();
        }
    }


    @Override
    public void loop() {
        if (gamepad1.b) {
            MotorFrontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            MotorBackLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            telemetry.addData("Status", "Resetting encoders");
            MotorFrontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            MotorBackLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        }

        if (gamepad1.a) {
            logUtils.Log(logUtils.logType.normal, MotorFrontLeft.getCurrentPosition() + "," + MotorBackLeft.getCurrentPosition(), 1);
        }

        telemetry.addData("xPulses", MotorFrontLeft.getCurrentPosition());
        telemetry.addData("yPulses", MotorBackLeft.getCurrentPosition());
        telemetry.update();
    }

    @Override
    public void stop() {
        logUtils.StopLogging(1);
        super.stop();
    }
}
