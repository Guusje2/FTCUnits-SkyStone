package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import java.io.IOException;


@TeleOp(name = "VuileBalkBak", group = "")
public class DrivercontrolledSkyStone extends OpMode {
    DriveTrainMecanum a;
    public Servo SchuifServo1;
    public Servo SchuifServo2;
    public double PosServo;
    public double rotation;
    public DcMotor KlemMotor1;
    public DcMotor KlemMotor2;
    private double PosSchuifServo = 1;

    @Override
    public void init() {
        a = new DriveTrainMecanum(hardwareMap.dcMotor.get("MotorBackLeft"),hardwareMap.dcMotor.get("MotorFrontLeft"),hardwareMap.dcMotor.get("MotorBackRight"),hardwareMap.dcMotor.get("MotorFrontRight"),hardwareMap.get(BNO055IMU.class,"imu"));

        KlemMotor1 = hardwareMap.dcMotor.get("KlemMotor");
        KlemMotor2 = hardwareMap.dcMotor.get("KlemMotor2");
        SchuifServo1 = hardwareMap.servo.get("SchuifServo1");
        SchuifServo2 = hardwareMap.servo.get("SchuifServo2");
        try {
            logUtils.StartLogging(3, "Voltage");
        } catch (IOException e) {
            e.printStackTrace();
        }
    }

    @Override
    public void loop() {

        if (gamepad2.dpad_down)
        {
            PosSchuifServo -= 0.01;
        }
        if (gamepad2.dpad_up){
            PosSchuifServo += 0.01;
        }
        if (PosSchuifServo < 0.17){
            PosSchuifServo = 0.17;
        } else if (PosSchuifServo >0.82){
            PosSchuifServo = 0.82;
        }

        SchuifServo1.setPosition(PosSchuifServo);
        SchuifServo2.setPosition(PosSchuifServo);

        

        telemetry.addData("servopos1", SchuifServo1.getPosition());
        telemetry.addData("servopos2", SchuifServo2.getPosition());

        KlemMotor1.setPower(gamepad2.left_stick_y);
        KlemMotor2.setPower(gamepad2.left_stick_y*-1);

        rotation = 0;
        rotation += gamepad1.right_trigger;
        rotation -= gamepad1.left_trigger;
        a.rotation = rotation;
        a.xMovement = gamepad1.left_stick_y;
        a.yMovement = -gamepad1.left_stick_x;
        a.MoveRotation();
        logUtils.Log(logUtils.logType.normal, String.valueOf(getBatteryVoltage()) + "," + -gamepad1.left_stick_y + "," + gamepad1.left_stick_x + "," + rotation ,3);
    }

    @Override
    public void stop() {
        logUtils.StopLogging(3);
        super.stop();
    }

    public double getBatteryVoltage() {
        double result = Double.POSITIVE_INFINITY;
        for (VoltageSensor sensor : hardwareMap.voltageSensor) {
            double voltage = sensor.getVoltage();
            if (voltage > 0) {
                result = Math.min(result, voltage);
            }
        }
        return result;
    }
}
