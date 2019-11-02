package org.firstinspires.ftc.teamcode;

import android.Manifest;
import android.app.Activity;
import android.content.pm.PackageManager;
import android.support.v4.app.ActivityCompat;
import android.util.JsonReader;

import com.acmerobotics.dashboard.FtcDashboard;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.teamcode.MathEssentials.MathFunctions;
import org.firstinspires.ftc.teamcode.MathEssentials.Vector2;
import org.opencv.core.Mat;

import java.io.File;
import java.io.FileInputStream;
import java.io.FileNotFoundException;
import java.io.IOException;
import java.io.InputStream;
import java.io.InputStreamReader;
import java.io.UnsupportedEncodingException;


/**
 * Extended from DriveTrainMecanum, but includes encoder based driving
 * Expects the encoder horizontal to be on MotorFrontLeft and encoder vertical to be on MotorBackLeft
 */
public class DriveTrainMecanumEncoder extends DriveTrainMecanum {
    public RobotConstants robotConstants;
    private int xEncoderPulses;
    private int yEncoderPulses;

    public Vector2 currentPos;

    private double dx = 0;
    private double dy = 0;
    public double currAngle;
    double prevAngle;

    public  DriveTrainMecanumEncoder (DcMotor _MotorBackLeft, DcMotor _MotorBackRight, DcMotor _MotorFrontLeft, DcMotor _MotorFrontRight, BNO055IMU _imu) {
        MotorBackLeft = _MotorBackLeft;
        MotorBackRight = _MotorBackRight;
        MotorFrontLeft = _MotorFrontLeft;
        MotorFrontRight = _MotorFrontRight;
        MotorBackLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        MotorFrontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        try {
            Thread.sleep(50);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
        MotorBackLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        MotorFrontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robotConstants = new RobotConstants();
        imu = _imu;
        xEncoderPulses = MotorFrontLeft.getCurrentPosition();
        yEncoderPulses = MotorBackLeft.getCurrentPosition();

        currentPos = new Vector2(0,0);

		currAngle = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES).thirdAngle;
        try {
            FtcDashboard.start();
            dashboard = FtcDashboard.getInstance();
        } catch (Exception e) {

        }
    }

    /**
        Used for updating the position of the robot via the Odometry system
     */
    public void UpdatePos() {
        //putting the angle of the robot in a variable to reduce calls to the imu(which are slower)
        currAngle = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES).thirdAngle;
        //inverted because the encoder is mounted the wrong way, not easily fixable
        int xPulsesCurrent = MotorFrontLeft.getCurrentPosition();
        int yPulsesCurrent = MotorBackLeft.getCurrentPosition();

        //Calculate encoder deltas
        double xEncoderDelta = xPulsesCurrent - xEncoderPulses;
        double yEncoderDelta = yPulsesCurrent - yEncoderPulses;

        //Calculate Xmovement relative to robot in mm
        double xMovementRobot = xEncoderDelta * RobotConstants.mmPerPulse;
        double yMovementRobot = yEncoderDelta * RobotConstants.mmPerPulse;

        //Calculate the accurate deltas in world space
        double dx = ((Math.sin(MathFunctions.FixAngleRad(Math.toRadians(currAngle))) * yMovementRobot) + (Math.cos(MathFunctions.FixAngleRad(Math.toRadians(currAngle))) * xMovementRobot))*-1;
        double dy = ((Math.cos(MathFunctions.FixAngleRad(Math.toRadians(currAngle))) * yMovementRobot) + (Math.sin(MathFunctions.FixAngleRad(Math.toRadians(currAngle))) * xMovementRobot))*    1;

        //add the deltas to the current position

        currentPos = new Vector2(currentPos.X + dx, currentPos.Y + dy);


		//send telemetry
        TelemetryPacket b = new TelemetryPacket();
        b.put("xPulse", MotorFrontLeft.getCurrentPosition());
        b.put("yPulse", MotorBackLeft.getCurrentPosition());
        b.put("dX", dx);
        b.put("dY", dy);

        b.put("xPos", currentPos.X);
        b.put("yPos", currentPos.Y);
        b.put("angle", currAngle);
        //b.fieldOverlay().fillRect(currentPos.X/25.4 ,currentPos.Y/25.4 ,20,20);

        dashboard.sendTelemetryPacket(b);
		//set all values for the next run
        xEncoderPulses = xPulsesCurrent;
        yEncoderPulses = yPulsesCurrent;
		prevAngle = currAngle;
    }

    /**
     * Used for updating the position of the robot via the Odometry system
     * @param packet the telemetry packet to which all the telemetry stuff is added
     */
    public void UpdatePos(TelemetryPacket packet) {
        //putting the angle of the robot in a variable to reduce calls to the imu(which are slower)
        currAngle = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES).thirdAngle;
        int xPulsesCurrent = MotorFrontLeft.getCurrentPosition();
        int yPulsesCurrent = MotorBackLeft.getCurrentPosition();

        //Calculate encoder deltas
        double xEncoderDelta = xPulsesCurrent - xEncoderPulses;
        double yEncoderDelta = yPulsesCurrent - yEncoderPulses;

        //Calculate Xmovement relative to robot in mm
        double xMovementRobot = xEncoderDelta * RobotConstants.mmPerPulse;
        double yMovementRobot = yEncoderDelta * RobotConstants.mmPerPulse;

        //Calculate the accurate deltas in world space
        double dx = ((Math.sin(MathFunctions.FixAngleRad(Math.toRadians(currAngle))) * yMovementRobot) + (Math.cos(MathFunctions.FixAngleRad(Math.toRadians(currAngle))) * xMovementRobot))*-1;
        double dy = ((Math.cos(MathFunctions.FixAngleRad(Math.toRadians(currAngle))) * yMovementRobot) + (Math.sin(MathFunctions.FixAngleRad(Math.toRadians(currAngle))) * xMovementRobot))*    1;

        //add the deltas to the current position

        currentPos = new Vector2(currentPos.X + dx, currentPos.Y + dy);




        packet.put("xPulse", MotorFrontLeft.getCurrentPosition());
        packet.put("yPulse", MotorBackLeft.getCurrentPosition());

        packet.put("xPos", currentPos.X);
        packet.put("yPos", currentPos.Y);


        packet.put("dx", dx);
        packet.put("dy", dy);
        packet.put("angle", currAngle);
        //divide by 25.4 because from mm to inches
        //

        dashboard.sendTelemetryPacket(packet);
        xEncoderPulses = xPulsesCurrent;
        yEncoderPulses = yPulsesCurrent;
		prevAngle = currAngle;
    }

    /**
     * Turns the robot to a specific angle
     * @param angle angle to turn to
     * @param speed speed to turn with
     * @param precision the precision of the angle turned to. This is used in the Ish function, as a value range in which the angle should be
     */
    public void TurnToAngle (double angle, double speed, double precision) {
        speed = speed/10;
        while (!MathFunctions.Ish(currAngle,precision, MathFunctions.FixAngle( angle)) && opMode.opModeIsActive()) {
            //calculate the delta and send it to the dashboard
            double delta = angle - currAngle;
            TelemetryPacket packet = new TelemetryPacket();
            packet.put("Angle Delta TurnToAngle", delta);
            packet.put("Current Angle", currAngle);

            imu.getAngularVelocity().unit = AngleUnit.DEGREES;

            if (delta > 0) {
                MotorBackLeft.setPower(speed*(Math.abs(delta/22.5)) + -0.005* imu.getAngularVelocity().zRotationRate + 0.1);
                MotorFrontLeft.setPower(speed*(Math.abs(delta/22.5))+ -0.005* imu.getAngularVelocity().zRotationRate + 0.1);
                MotorFrontRight.setPower(speed*(Math.abs(delta/22.5))+ -0.005* imu.getAngularVelocity().zRotationRate + 0.1);
                MotorBackRight.setPower(speed*(Math.abs(delta/22.5))+ -0.005* imu.getAngularVelocity().zRotationRate + 0.1);
            } else {
                MotorBackLeft.setPower(-speed*(Math.abs(delta/22.5))- -0.005* imu.getAngularVelocity().zRotationRate - 0.1);
                MotorFrontLeft.setPower(-speed*(Math.abs(delta/22.5))- -0.005* imu.getAngularVelocity().zRotationRate - 0.1);
                MotorFrontRight.setPower(-speed*(Math.abs(delta/22.5))- -0.005* imu.getAngularVelocity().zRotationRate -0.1);
                MotorBackRight.setPower(-speed*(Math.abs(delta/22.5)) - -0.005* imu.getAngularVelocity().zRotationRate - 0.1);
            }
            UpdatePos(packet);
        }
        MotorBackLeft.setPower(0);
        MotorFrontLeft.setPower(0);
        MotorFrontRight.setPower(0);
        MotorBackRight.setPower(0);
    }

    /**
     * this lets the robot moves to the right
     * @param _speed if positive, moves to the right. keep between 1 and -1
     * @param _timeSeconds how long the function is
     */
    public void MoveSideWaySeconds(double _speed, double _timeSeconds){
        double starttime = java.lang.System.currentTimeMillis();
        while (java.lang.System.currentTimeMillis() < starttime+_timeSeconds && opMode.opModeIsActive()){
            MotorFrontLeft.setPower(_speed);
            MotorBackLeft.setPower(-_speed);
            MotorFrontRight.setPower(_speed);
            MotorBackRight.setPower(-_speed);
            UpdatePos();
        }
        MotorBackLeft.setPower(0);
        MotorFrontLeft.setPower(0);
        MotorFrontRight.setPower(0);
        MotorBackRight.setPower(0);

    }



    /**
     * Drives forward (or backward) whilst holding the current angle for a set time
     * @param timeSeconds time in seconds to drive forward
     * @param Speed movement speed
     */
    public void DriveForwardCorrection (float timeSeconds, float Speed)
    {
        double startAngle = currAngle;
        double endTime = System.currentTimeMillis() + (timeSeconds*1000);
        double left = 0;
        double right = 0;
        double correction;
        while (System.currentTimeMillis() < endTime && opMode.opModeIsActive()){
            correction = (startAngle - currAngle)*-0.1;
            right = Speed + correction;
            left = (Speed - correction)*-1;
            MotorFrontRight.setPower(right);
            MotorBackRight.setPower(right);
            MotorFrontLeft.setPower(left);
            MotorBackLeft.setPower(left);
            UpdatePos();
        }
        MotorBackLeft.setPower(0);
        MotorFrontLeft.setPower(0);
        MotorBackRight.setPower(0);
        MotorFrontRight.setPower(0);

    }

    /**
     * moves the robot forwrd for a certain distance based on encoder values
     * @param Speed
     * @param distance
     */
    public void EncoderDriveForwardCorrection (float Speed, double distance){
        double startAngle = currAngle;
        double left = 0;
        double right = 0;
        double correction;

        double startPosY = currentPos.Y;

        double motorSpeed=0;

        while (startPosY + distance > currentPos.Y && opMode.opModeIsActive()){

            correction = (startAngle - currAngle)*-0.1;
            motorSpeed = Speed;
            right = motorSpeed + correction;
            left = (motorSpeed - correction);
            MotorFrontRight.setPower(right);
            MotorBackRight.setPower(right);
            MotorFrontLeft.setPower(left);
            MotorBackLeft.setPower(left);
            UpdatePos();
        }
        MotorBackLeft.setPower(0);
        MotorFrontLeft.setPower(0);
        MotorBackRight.setPower(0);
        MotorFrontRight.setPower(0);
    }


    /**
     * Moves the robot to a position on the field
     * @param point point to move to
     * @param speed robot speed
     */
    public void MoveToPos(Vector2 point, double speed, double targetAngle, double turnSpeed){

        double distanceToTarget = point.DistanceToVector2(currentPos);

        double worldAngleTarget = Math.atan2(point.Y - currentPos.Y, point.X - currentPos.X);

        double robotAnglePoint = MathFunctions.FixAngleRad(worldAngleTarget - Math.toRadians(currAngle));

        double relativedX = Math.cos(robotAnglePoint) * distanceToTarget;
        double relativedY = Math.sin(robotAnglePoint) * distanceToTarget;

        double xPower = relativedX / (Math.abs(relativedX) + Math.abs(relativedY));
        double yPower = relativedY / (Math.abs(relativedX) + Math.abs(relativedY));

        xMovement = xPower * speed;
        yMovement = -yPower * speed;


        double relativeRotation = robotAnglePoint - Math.PI + Math.toRadians(targetAngle);
        rotation = Range.clip( relativeRotation/Math.toRadians(25), -1, 1)* turnSpeed;

        if (distanceToTarget < 100){
            rotation = 0;
        }
        Move();
        UpdatePos();

	}

	/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /////            Pos import shit
    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

    Vector2 ImportedPos;
    boolean isBlue;

    public void loadJSONFile() {
        File input = new File(logUtils.getPublicAlbumStorageDir("FTCunits") + "startpos.json");
        InputStream is;
        ImportedPos = new Vector2(0,0);
        try{
            is = new FileInputStream(input);
            JsonReader reader = new JsonReader(new InputStreamReader(is, "UTF-8"));
            while (reader.hasNext()){
                String name = reader.nextName();
                switch (name){
                    case "x":
                    ImportedPos.X = reader.nextInt();
                        break;
                    case "y":
                        ImportedPos.Y = reader.nextInt();
                        break;
                    case "blueAlliance":
                        isBlue = reader.nextBoolean();
                        break;
                }
                reader.endObject();
            }
        } catch (FileNotFoundException e) {
            e.printStackTrace();
        } catch (UnsupportedEncodingException e) {
            e.printStackTrace();
        } catch (IOException e) {
            e.printStackTrace();
        }
    }




    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /////               import van stackoverflow
    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

    // Storage Permissions
    private static final int REQUEST_EXTERNAL_STORAGE = 1;
    private static String[] PERMISSIONS_STORAGE = {
            Manifest.permission.READ_EXTERNAL_STORAGE,
            Manifest.permission.WRITE_EXTERNAL_STORAGE
    };

    /**
     * Checks if the app has permission to write to device storage
     *
     * If the app does not has permission then the user will be prompted to grant permissions
     *
     * @param activity
     */
    public static void verifyStoragePermissions(Activity activity) {
        // Check if we have write permission
        int permission = ActivityCompat.checkSelfPermission(activity, Manifest.permission.READ_EXTERNAL_STORAGE);

        if (permission != PackageManager.PERMISSION_GRANTED) {
            // We don't have permission so prompt the user
            ActivityCompat.requestPermissions(
                    activity,
                    PERMISSIONS_STORAGE,
                    REQUEST_EXTERNAL_STORAGE
            );
        }
    }
}
