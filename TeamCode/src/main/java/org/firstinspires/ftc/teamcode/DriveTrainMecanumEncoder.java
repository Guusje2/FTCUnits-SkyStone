package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.teamcode.MathEssentials.MathFunctions;
import org.firstinspires.ftc.teamcode.MathEssentials.Vector2;

/**
 * Extended from DriveTrainMecanum, but includes encoder based driving
 * Expects the encoder horizontal to be on MotorFrontLeft and encoder vertical to be on MotorBackLeft
 */
public class DriveTrainMecanumEncoder extends DriveTrainMecanum {
    public RobotConstants robotConstants;
    private int xEncoderPulses;
    private int yEncoderPulses;
    public Vector2 CurrentPos;
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
        CurrentPos = new Vector2(0,0);
		currAngle = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES).thirdAngle;
        try {
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
        double dx = (Math.sin(MathFunctions.FixAngleRad(Math.toRadians(currAngle))) * yMovementRobot) + (Math.cos(MathFunctions.FixAngleRad(Math.toRadians(currAngle))) * xMovementRobot);
        double dy = (Math.cos(MathFunctions.FixAngleRad(Math.toRadians(currAngle))) * yMovementRobot) + (Math.sin(MathFunctions.FixAngleRad(Math.toRadians(currAngle))) * xMovementRobot);

        //add the deltas to the current position
        CurrentPos = new Vector2(CurrentPos.X + dx,CurrentPos.Y + dy);

		//send telemetry
        TelemetryPacket b = new TelemetryPacket();
        b.put("xPulse", MotorFrontLeft.getCurrentPosition());
        b.put("yPulse", MotorBackLeft.getCurrentPosition());
        b.put("dX", dx);
        b.put("dY", dy);
        b.put("xPos", CurrentPos.X);
        b.put("yPos", CurrentPos.Y);
        b.put("angle", currAngle);
        //b.fieldOverlay().fillRect(CurrentPos.X/25.4 ,CurrentPos.Y/25.4 ,20,20);
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
        double dx = (Math.sin(MathFunctions.FixAngleRad(Math.toRadians(currAngle))) * yMovementRobot) + (Math.cos(MathFunctions.FixAngleRad(Math.toRadians(currAngle))) * xMovementRobot);
        double dy = (Math.cos(MathFunctions.FixAngleRad(Math.toRadians(currAngle))) * yMovementRobot) + (Math.sin(MathFunctions.FixAngleRad(Math.toRadians(currAngle))) * xMovementRobot);

        //add the deltas to the current position
        CurrentPos = new Vector2(CurrentPos.X + dx,CurrentPos.Y + dy);


        packet.put("xPulse", MotorFrontLeft.getCurrentPosition());
        packet.put("yPulse", MotorBackLeft.getCurrentPosition());
        packet.put("xPos", CurrentPos.X);
        packet.put("yPos", CurrentPos.Y);
        packet.put("dx", dx);
        packet.put("dy", dy);
        packet.put("angle", currAngle);
        //divide by 25.4 because from mm to inches
        //
        // packet.fieldOverlay().fillRect(CurrentPos.X/25.4 + 77,CurrentPos.Y/25.4 + 77,20,20);
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
        speed = speed/12;
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
        double startPosY = CurrentPos.Y;

        double motorSpeed=0;

        while (startPosY + distance > CurrentPos.Y && opMode.opModeIsActive()){
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

    public void MoveToPos(Vector2 point, float speed){
		double startDistance = CurrentPos.DistanceToVector2(point);
		double currDistance;
		double dx = point.X - CurrentPos.X;
        double dy = point.Y - CurrentPos.Y;
        double absoluteAngle = Math.toDegrees(Math.atan2(dx,dy));
        TurnToAngle(absoluteAngle,2*speed, .1);
		while(MathFunctions.Ish(currDistance, 10, 0) {
			currDistance = CurrentPos.DistanceToVector2(point);
			//TODO: add speed of robot to this calculation
			double forwardSpeed = speed*(currDistance/startDistance)
			//times -1 because our left drivetrain motors are mirrored from the right ones
			double speedLeft = (forwardSpeed*0.8 + -0.1*(currAngle - absoluteAngle))*-1;
			double speedRight = forwardSpeed*0.8 + 0.1*(currAngle - absoluteAngle);
			MotorBackLeft.setPower(speedLeft);
			MotorFrontLeft.setPower(speedLeft);
			MotorBackRight.setPower(speedRight);
			MotorFrontRight.setPower(speedRight);
		}
	}
}
