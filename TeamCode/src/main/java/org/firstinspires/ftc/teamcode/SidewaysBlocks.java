package org.firstinspires.ftc.teamcode;

        import com.qualcomm.hardware.bosch.BNO055IMU;
        import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
        import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
        import com.qualcomm.robotcore.eventloop.opmode.OpMode;

        import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
        import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
        import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;

@Autonomous(name = "SidewaysBlocks")
public class SidewaysBlocks extends LinearOpMode {
    DriveTrainMecanum driveTrain;

    /**
     * In s
     */
    public double timeToMove = 1.5;
    public double startTime;
    public double startAngle;

    @Override
    public void runOpMode() {
        driveTrain = new DriveTrainMecanum(hardwareMap.dcMotor.get("MotorBackLeft"),hardwareMap.dcMotor.get("MotorBackRight"),hardwareMap.dcMotor.get("MotorFrontLeft"),hardwareMap.dcMotor.get("MotorFrontRight"),hardwareMap.get(BNO055IMU.class,"imu"));

        startAngle = driveTrain.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES).thirdAngle;
        waitForStart();
        startTime = getRuntime();
        while (startTime + timeToMove > getRuntime() && opModeIsActive()){
            //driveTrain.rotation = 0.1*(startAngle - driveTrain.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES).thirdAngle);
            driveTrain.xMovement = 1;
            driveTrain.MoveRotation();
        }

    }

}
