package org.firstinspires.ftc.teamcode;


import com.acmerobotics.dashboard.config.ValueProvider;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;


import org.firstinspires.ftc.teamcode.MathEssentials.MathFunctions;
import org.firstinspires.ftc.teamcode.MathEssentials.Vector2;

import java.io.IOException;
import java.util.Calendar;

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
        try {
            a = new DriveTrainMecanumEncoder(
                    hardwareMap.dcMotor.get("MotorBackLeft"),
                    hardwareMap.dcMotor.get("MotorBackRight"),
                    hardwareMap.dcMotor.get("MotorFrontLeft"),
                    hardwareMap.dcMotor.get("MotorFrontRight"),
                    hardwareMap.get(BNO055IMU.class, "imu")
            );
        } catch (IOException ex) {
            ex.printStackTrace();
        }
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
        int i = 4;
        while (opModeIsActive()){
            a.UpdatePos();

            try {

                logUtils.Log(logUtils.logType.normal, Calendar.getInstance().getTime().toString() + "," + a.currentPos.X + "," + a.currentPos.Y,1);


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

                if(MathFunctions.Ish(a.currentPos.DistanceToVector2(new Vector2(1000,1000)), 10, 0)){
                    i= 5;
                }
                a.MoveToPos(new Vector2(1000,1000),0.4, 90, 0.2);

            } else if (i==5){

            }
        }
        logUtils.StopLogging(1);


        //a.DriveForwardCorrection(3, .5f);
    }



}