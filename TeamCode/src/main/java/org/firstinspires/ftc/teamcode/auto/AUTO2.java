package org.firstinspires.ftc.teamcode.auto;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.drive.HMap;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.ArrayList;


@Autonomous(name = "Albastru Close")
public class AUTO2 extends LinearOpMode {
    /*
    *  drive.trajectorySequenceBuilder(new Pose2d(13, -60, 0))
                                .forward(50)
                                *
                                *
                                *
                                *
                                * drive.trajectorySequenceBuilder(new Pose2d(-37, -57, 0))
                                .forward(40)
                                .splineTo(new Vector2d(35, -11),0)
                                .forward(16)*/


    @Override
    public void runOpMode() {


        OpenCvCamera camera;
        AprilTagDetectionPipeline aprilTagDetectionPipeline;

        HMap robot = new HMap();
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        robot.init(hardwareMap);

        double fx = 578.272;
        double fy = 578.272;
        double cx = 402.145;
        double cy = 221.506;

        // UNITS ARE METERS
        double tagsize = 0.166;

        AprilTagDetection tagOfInterest = null;

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        aprilTagDetectionPipeline = new AprilTagDetectionPipeline(tagsize, fx, fy, cx, cy);

        camera.setPipeline(aprilTagDetectionPipeline);
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

        telemetry.setMsTransmissionInterval(50);


        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        telemetry.update();

        while (!isStarted() && !isStopRequested())
        {
            ArrayList<AprilTagDetection> currentDetections = aprilTagDetectionPipeline.getLatestDetections();

            if(currentDetections.size() != 0)
            {
                boolean tagFound = false;

                for(AprilTagDetection tag : currentDetections)
                {
                    if(tag.id == 1)
                    {
                        tagOfInterest = tag;
                        tagFound = true;
                        break;
                    }
                }

            }
            if(tagOfInterest != null)
            {
                telemetry.addLine("Tag snapshot:\n");
                tagToTelemetry(tagOfInterest);
                telemetry.addData("x: ", tagOfInterest.pose.x);
                telemetry.update();
            }

            telemetry.update();
            sleep(20);
        }

        waitForStart();

        if(tagOfInterest != null) {

            if (tagOfInterest.pose.x < -0.5 && !isStopRequested()) {

                Trajectory traiect = drive.trajectoryBuilder(new Pose2d(11, 60, Math.toRadians(-90)))
                        .lineToSplineHeading(new Pose2d(15, 40, Math.toRadians(-45)))
                        .back(5)
                        .lineToSplineHeading(new Pose2d(11, 11, Math.toRadians(0)))
                        .lineToSplineHeading(new Pose2d(59, 10, Math.toRadians(0)))
                        .build();
                drive.followTrajectory(traiect);

                telemetry.addData("case 1", tagOfInterest.pose.x);
                telemetry.update();
            }

            if (tagOfInterest.pose.x > -0.5 && tagOfInterest.pose.x < 0.5 && !isStopRequested()) {

                Trajectory trajectory= drive.trajectoryBuilder(new Pose2d(11, 60, Math.toRadians(-90)))
                        .lineTo(new Vector2d(11, 30))
                        .back(5)
                        .splineTo(new Vector2d(59, 10), Math.toRadians(0))
                        .build();

                drive.followTrajectory(trajectory);
                telemetry.addData("case 2", tagOfInterest.pose.x);
                telemetry.update();
            }

            if (tagOfInterest.pose.x > 0.5 && !isStopRequested()) {

                Trajectory trjct = drive.trajectoryBuilder(new Pose2d(11, 60, Math.toRadians(-90)))
                        .lineToSplineHeading(new Pose2d(9, 35, Math.toRadians(220)))
                        .back(7)
                        .lineToSplineHeading(new Pose2d(11, 11, Math.toRadians(0)))
                        .lineTo(new Vector2d(59, 10))
                        .build();

                drive.followTrajectory(trjct);
                telemetry.addData("case 3", tagOfInterest.pose.x);
                telemetry.update();
            }
        }
        else
            telemetry.addData("muie", tagOfInterest.toString());
        telemetry.update();





    }

    void tagToTelemetry(AprilTagDetection detection)
    {
        telemetry.addLine(String.format("\nDetected tag ID=%d", detection.id));
    }
}
