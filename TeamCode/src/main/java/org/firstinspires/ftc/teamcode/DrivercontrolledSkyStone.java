package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import java.io.IOException;


@TeleOp(name = "VuileBalkBak", group = "")
public class DrivercontrolledSkyStone extends OpMode {
    DriveTrainMecanum a;
    public Servo bakServo1;
    public Servo bakServo2;
    public Servo armServo1;
    public Servo armServo2;
    public double PosServo;
    public double rotation;
    public DcMotor Armmotor;
    private double PosArmServo = 1;

    @Override
    public void init() {
        a = new DriveTrainMecanum(hardwareMap.dcMotor.get("MotorBackLeft"),hardwareMap.dcMotor.get("MotorFrontLeft"),hardwareMap.dcMotor.get("MotorBackRight"),hardwareMap.dcMotor.get("MotorFrontRight"),hardwareMap.get(BNO055IMU.class,"imu"));
        bakServo1 = hardwareMap.servo.get("BalkServo1");
        bakServo2 = hardwareMap.servo.get("BalkServo2");
        Armmotor = hardwareMap.dcMotor.get("armmotor");
        armServo1 = hardwareMap.servo.get("ArmServo1");
        armServo2 = hardwareMap.servo.get("ArmServo2");
        try {
            logUtils.StartLogging(3, "Voltage");
        } catch (IOException e) {
            e.printStackTrace();
        }
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

        if (gamepad2.dpad_down)
        {
            PosArmServo -= 0.01;
        }
        if (gamepad2.dpad_up){
            PosArmServo += 0.01;
        }
        if (PosArmServo < -0.75){
            PosArmServo = -0.75;
        } else if (PosArmServo >1){
            PosArmServo = 1;
        }

        armServo1.setPosition(PosArmServo);
        armServo2.setPosition(1-PosArmServo);

        if (PosServo < 0){
            PosServo = 0;
        } else if (PosServo >1){
            PosServo = 0.9;
        }


        bakServo1.setPosition(PosServo);
        bakServo2.setPosition(1-PosServo);

        telemetry.addData("servopos1", armServo1.getPosition());
        telemetry.addData("servopos2", armServo2.getPosition());

        Armmotor.setPower(gamepad2.left_stick_y*0.5);

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
