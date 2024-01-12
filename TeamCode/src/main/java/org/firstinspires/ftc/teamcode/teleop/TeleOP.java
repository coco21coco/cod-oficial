package org.firstinspires.ftc.teamcode.teleop;

import android.graphics.Camera;

import com.acmerobotics.dashboard.DashboardCore;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.drive.HMap;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.opencv.core.Mat;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;


@TeleOp(name = "TeleOP", group = "TeleOP")
@Config

public class TeleOP extends LinearOpMode {





    @Override
    public void runOpMode() throws InterruptedException {
        //robot.init(hardwareMap);

        HMap robot = new HMap();
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        FtcDashboard dashboard = FtcDashboard.getInstance();

        Telemetry dashtelemtry = dashboard.getTelemetry();

        MultipleTelemetry Telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        OpenCvWebcam camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                camera.startStreaming(800,448, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode)
            {

            }
        });
        FtcDashboard.getInstance().startCameraStream(camera, 30);




        double speed = 100;

        boolean toggle_unghi = false;
        double unghi_curent = 0;

        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.init(hardwareMap);

        waitForStart();


        while (opModeIsActive()) {



            drive.update();
            Pose2d poseEstimate = drive.getPoseEstimate();


            if (gamepad1.right_bumper) {
                toggle_unghi = true;
                unghi_curent = drive.getExternalHeading();
            }

            if (gamepad1.left_bumper)
                toggle_unghi = false;
            
            drive.setWeightedDrivePower(
                        new Pose2d(
                                -gamepad1.left_stick_y * speed / 100,
                                -gamepad1.left_stick_x * speed / 100,
                                -gamepad1.right_stick_x * speed / 100
                        )
                );


            if (gamepad1.left_bumper) {
                speed = 50;
                gamepad1.rumbleBlips(1);
            }
            if (gamepad1.right_bumper) {
                speed = 85;
                gamepad1.rumbleBlips(1);
            }

            // colectare

            if (gamepad2.b) {
                robot.colectare.setPower(1);
                gamepad2.rumbleBlips(1);
            }
            else
                robot.colectare.setPower(0);

            if (gamepad2.a)
                robot.colectare.setPower(-1);

            // cutie

            if (gamepad2.left_stick_button) {
                gamepad2.rumble(300);
                robot.cutie.setPosition(robot.cutie_deskis);
                sleep(300);
                robot.cutie.setPosition(robot.cutie_inkis);
            }

            if (gamepad2.y) {
                robot.avion.setPosition(robot.avion_tras);
                sleep(200);
                robot.avion.setPosition(robot.avion_armat);
            }
            // glisiere
            if (gamepad2.dpad_up)
                robot.glisiere.setPower(1);
            else
                robot.glisiere.setPower(0);

            if (gamepad2.dpad_down)
                robot.glisiere.setPower(-1);

            Telemetry.addData("viteza coaie: ", speed);
            Telemetry.addData("x", poseEstimate.getX());
            Telemetry.addData("y", poseEstimate.getY());
            Telemetry.addData("heading", Math.toDegrees(drive.getExternalHeading()));
            Telemetry.addData("unghi mentinut", Math.toDegrees(unghi_curent));
            Telemetry.addData("MENTINE", toggle_unghi);
            Telemetry.update();
        }



    }


    public void maintain_angle(double angl, double speed1, SampleMecanumDrive d){
        double unghi = d.getExternalHeading();

        if(Math.toDegrees(unghi) - Math.toDegrees(angl) > Math.toRadians(3))
            d.setMotorPowers(speed1, speed1,   -speed1, -speed1);

        else
        if(Math.toDegrees(unghi) - Math.toDegrees(angl) < -Math.toRadians(3))
            d.setMotorPowers(-speed1, -speed1, speed1, speed1);

    }

}



