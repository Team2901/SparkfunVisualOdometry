package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@Disabled
@TeleOp
public class OdometrySensorExample extends LinearOpMode
{
    // Define sensor
    SparkFunOTOS myOdometrySensor;

    @Override
    public void runOpMode() throws InterruptedException
    {
        // Get sensor from hardware map
        myOdometrySensor = hardwareMap.get(SparkFunOTOS.class, "sfe_otos");

        // Set offset location of sensor relative to center of robot
        SparkFunOTOS.otos_pose2d_t offsetPose = myOdometrySensor.new otos_pose2d_t(3.7, -.55, 0);
        myOdometrySensor.setOffset(offsetPose);

        // Set scalar correction factors for translation and rotation
        myOdometrySensor.setLinearScalar(1.0);
        myOdometrySensor.setAngularScalar(1.0);

        // Calibrate IMU. Must be stationary and flat during this time! By default, this will take
        // about 612ms, but can be made faster by taking fewer samples
        myOdometrySensor.calibrateImu();

        // Reset tracking algorithm on OTOS
        myOdometrySensor.resetTracking();

        // Set known location on field
        SparkFunOTOS.otos_pose2d_t newPose = myOdometrySensor.new otos_pose2d_t(5, 10, 45);
        myOdometrySensor.setPosition(newPose);

        waitForStart();

        while (opModeIsActive())
        {
            // Get tracked location
            SparkFunOTOS.otos_pose2d_t pose = myOdometrySensor.getPosition();

            telemetry.addData("X (inch)", pose.x);
            telemetry.addData("Y (inch)", pose.y);
            telemetry.addData("H (Deg)", pose.h);
            telemetry.update();
        }
    }
}
