package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@TeleOp
public class OdometrySensorTest extends LinearOpMode
{
    public static double SIDE_LENGTH = 16;

    DcMotorEx motorFL;
    DcMotorEx motorFR;
    DcMotorEx motorBL;
    DcMotorEx motorBR;

    SparkFunOTOS myOdometrySensor;

    private static void rotatePoints(double[] xPoints, double[] yPoints, double angle)
    {
        for (int i = 0; i < xPoints.length; i++)
        {
            double x = xPoints[i];
            double y = yPoints[i];
            xPoints[i] = x * Math.cos(angle) - y * Math.sin(angle);
            yPoints[i] = x * Math.sin(angle) + y * Math.cos(angle);
        }
    }

    @Override
    public void runOpMode() throws InterruptedException
    {
        FtcDashboard dashboard = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());

        motorFL = hardwareMap.get(DcMotorEx.class, "frontLeft");
        motorFR = hardwareMap.get(DcMotorEx.class, "frontRight");
        motorBL = hardwareMap.get(DcMotorEx.class, "backLeft");
        motorBR = hardwareMap.get(DcMotorEx.class, "backRight");

        motorFL.setDirection(DcMotorEx.Direction.REVERSE);
        motorFR.setDirection(DcMotorEx.Direction.FORWARD);
        motorBL.setDirection(DcMotorEx.Direction.REVERSE);
        motorBR.setDirection(DcMotorEx.Direction.FORWARD);

        motorFL.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        motorFR.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        motorBL.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        motorBR.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        motorFL.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        motorFR.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        motorBL.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        motorBR.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

        motorFL.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        motorFR.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        motorBL.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        motorBR.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        myOdometrySensor = hardwareMap.get(SparkFunOTOS.class, "sfe_otos");
        SparkFunOTOS.otos_pose2d_t offsetPose = myOdometrySensor.new otos_pose2d_t(-6, 0, 90);
        myOdometrySensor.setOffset(offsetPose);
        myOdometrySensor.setLinearScalar(1.0);
        myOdometrySensor.setAngularScalar(1.0);

        // Calibrate IMU. Must be stationary and flat during this time! By default, this will take
        // about 612ms, but can be made faster by taking fewer samples
        myOdometrySensor.calibrateImu();

        // Reset tracking algorithm on OTOS
        myOdometrySensor.resetTracking();


        waitForStart();

        while (opModeIsActive())
        {
            double forward = -gamepad1.left_stick_y;
            double sidewaysRight = gamepad1.left_stick_x;
            double turnCCW = -gamepad1.right_stick_x;

            double powerFL = forward + sidewaysRight - turnCCW;
            double powerFR = forward - sidewaysRight + turnCCW;
            double powerBL = forward - sidewaysRight - turnCCW;
            double powerBR = forward + sidewaysRight + turnCCW;

            if (gamepad1.left_bumper)
            {
                powerFL /= 4;
                powerFR /= 4;
                powerBL /= 4;
                powerBR /= 4;
            }

            motorFL.setPower(powerFL);
            motorFR.setPower(powerFR);
            motorBL.setPower(powerBL);
            motorBR.setPower(powerBR);

            SparkFunOTOS.otos_pose2d_t pose = myOdometrySensor.getPosition();

            telemetry.addData("X (inch)", pose.x);
            telemetry.addData("Y (inch)", pose.y);
            telemetry.addData("H (Deg)", pose.h);
            telemetry.update();

            double bx = pose.x;
            double by = pose.y;
            double l = SIDE_LENGTH / 2;

            double[] bxPoints = {l, -l, -l, l};
            double[] byPoints = {l, l, -l, -l};
            double[] axPoints = {0, 0, 0, l / 2, 0, -l / 2};
            double[] ayPoints = {-l, l, l, l / 2, l, l / 2};
            rotatePoints(bxPoints, byPoints, pose.h * Math.PI / 180);
            rotatePoints(axPoints, ayPoints, pose.h * Math.PI / 180);
            for (int i = 0; i < 4; i++)
            {
                bxPoints[i] += bx;
                byPoints[i] += by;
            }
            for (int i = 0; i < 6; i++)
            {
                axPoints[i] += bx;
                ayPoints[i] += by;
            }

            TelemetryPacket packet = new TelemetryPacket();
            packet.fieldOverlay()
                    .setStrokeWidth(1)
                    .setStroke("goldenrod")
                    .setFill("black")
                    .fillPolygon(bxPoints, byPoints)
                    .strokeLine(axPoints[0], ayPoints[0], axPoints[1], ayPoints[1])
                    .strokeLine(axPoints[2], ayPoints[2], axPoints[3], ayPoints[3])
                    .strokeLine(axPoints[4], ayPoints[4], axPoints[5], ayPoints[5]);
            dashboard.sendTelemetryPacket(packet);

            if (gamepad1.x)
            {
                myOdometrySensor.calibrateImu();
            }

            if (gamepad1.y)
            {
                myOdometrySensor.resetTracking();
//                SparkFunOTOS.otos_pose2d_t newPose = myOdometrySensor.new otos_pose2d_t(5, 10, 45);
//                myOdometrySensor.setPosition(newPose);
            }

            sleep(20);
        }
    }
}
