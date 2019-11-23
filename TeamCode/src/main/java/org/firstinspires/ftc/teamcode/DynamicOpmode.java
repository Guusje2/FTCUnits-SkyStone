package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.Actions.RobotAction;
import org.firstinspires.ftc.teamcode.MathEssentials.MathFunctions;
import org.firstinspires.ftc.teamcode.MathEssentials.Vector2;

import java.io.IOException;

@Autonomous(name = "DynamicAuto")
public class DynamicOpmode extends LinearOpMode {
    public DriveTrainMecanumEncoder a;
    RobotAction[] actions;
    int ActionIndex;
    private double startWaitTime;

    @Override
    public void runOpMode() throws InterruptedException {
        try {
            a = new DriveTrainMecanumEncoder(hardwareMap.dcMotor.get("MotorBackLeft"), hardwareMap.dcMotor.get("MotorFrontLeft"), hardwareMap.dcMotor.get("MotorFrontRight"), hardwareMap.dcMotor.get("MotorBackRight"), hardwareMap.get(BNO055IMU.class, "imu"));
        } catch (IOException e) {
            e.printStackTrace();
        }
        a.loadJSONFile();
        a.currentPos = a.ImportedPos.add(new Vector2(200, 222.5));
        actions = a.actions;

        while (opModeIsActive()){
            if (ActionIndex + 1 <= actions.length){
                RobotAction action = actions[ActionIndex];
                switch (action.action) {
                    case wait:
                        Wait(action.paramter);
                        break;
                    case park:
                        Park(action.paramter);
                        break;
                    case foundation:
                        break;
                }
            }
            a.UpdatePos();
            telemetry.addData("Currentpos", "X: " + a.currentPos.X + " Y: " + a.currentPos.Y);
            telemetry.update();
        }
    }

    public void Wait (double timeSeconds){
        if (startWaitTime == 0){
            startWaitTime = getRuntime();
        }
        telemetry.addData("Status", "waiting");
        if (getRuntime()-startWaitTime > timeSeconds){
            startWaitTime = 0;
            ActionIndex++;
        }
    }

    public void Park(double pos){
        if (pos == 1){
            if (MathFunctions.Ish(a.currentPos.X,50,1790) && MathFunctions.Ish(a.currentPos.Y, 50,222.5)){
                ActionIndex++;
                return;
            }
            a.MoveToPos(new Vector2(1790, 222.5),0.75, 0,0.25);
        }
        if (pos == 2){
            if (MathFunctions.Ish(a.currentPos.X,50,1790) && MathFunctions.Ish(a.currentPos.Y, 50,850)){
                ActionIndex++;
                return;
            }
            a.MoveToPos(new Vector2(1790, 850),0.75, 0,0.25);
        }
    }

}
