package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.ValueProvider;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.MathEssentials.Vector2;

import java.util.Calendar;
import java.util.Vector;

import static org.firstinspires.ftc.teamcode.RobotConstants.*;

/**
 * Created by guusd on 9/23/2017.
 * FTC 2017, FTCunits
 * Used for testing highly experimental stuff, like the logutils systeem or the new autonomous nav system
 */

@TeleOp(name = "TestController", group = "Guus")

public class testController extends LinearOpMode {

    public DriveTrainMecanumEncoder a;
    logUtils e = new logUtils();

    public void runOpMode() {
        a = new DriveTrainMecanumEncoder(
                hardwareMap.dcMotor.get("MotorBackLeft"),
                hardwareMap.dcMotor.get("MotorBackRight"),
                hardwareMap.dcMotor.get("MotorFrontLeft"),
                hardwareMap.dcMotor.get("MotorFrontRight"),
                hardwareMap.get(BNO055IMU.class, "imu")
        );
        //a.dashboard.addConfigVariable("Odometry", "mmPerPulse",
        a.opMode = this;
        TelemetryPacket b = new TelemetryPacket();
        b.put("Status","Waiting");
        a.dashboard.sendTelemetryPacket(b);
        waitForStart();
        try {
            logUtils.Log(logUtils.logType.normal,"Time,X,Y",1);
        } catch (Exception e){

        }
        int i = 5;
        while (opModeIsActive()){
            a.UpdatePos();

            try {
                logUtils.Log(logUtils.logType.normal, Calendar.getInstance().getTime().toString() + "," + a.CurrentPos.X + "," + a.CurrentPos.Y,1);

            } catch (Exception e){

            }
            if (i == 0 ) {

                a.MoveSideWaySeconds(1,5);
                i=1;
            } else if (i == 1){
                a.TurnToAngle(90,1,0.25);
                i = 2;
            } else if (i == 2){
                a.TurnToAngle(-90  ,1,0.25);
                i=3;
            } else if (i == 3) {
                a.TurnToAngle(135, 1, 0.25);
                i=4;
            } else if (i == 4){
                a.MoveToPos(new Vector2(590,0),0.75f);
                i=5;
            } else if (i==5){

            }
        }
        logUtils.StopLogging(1);


        //a.DriveForwardCorrection(3, .5f);
    }



}