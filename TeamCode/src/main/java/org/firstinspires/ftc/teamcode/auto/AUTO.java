package org.firstinspires.ftc.teamcode.auto;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.checkerframework.framework.qual.DefaultInUncheckedCodeFor;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.drive.HMap;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.ArrayList;

@Disabled
@Autonomous(name = "PLA")
public class AUTO extends LinearOpMode {
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

                drive.setPoseEstimate(new Pose2d(13, -60, Math.toRadians(90)));
                TrajectorySequence traiect = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                        .splineTo(new Vector2d(7.5, -35.9), Math.toRadians(130))
                        .back(10)
                        .splineTo(new Vector2d(59, -11), Math.toRadians(0))
                                .build();

                drive.followTrajectorySequence(traiect);

                telemetry.addData("case 1", tagOfInterest.pose.x);
                telemetry.update();
            }

            if (tagOfInterest.pose.x > -0.5 && tagOfInterest.pose.x < 0.5 && !isStopRequested()) {

                drive.setPoseEstimate(new Pose2d(13, -60, Math.toRadians(90)));
                TrajectorySequence trajectory= drive.trajectorySequenceBuilder( drive.getPoseEstimate())

                        .lineTo(new Vector2d(13, -30))
                        .lineTo(new Vector2d(13, -37.5))
                        .splineTo(new Vector2d(59, -11), Math.toRadians(0))
                        .build();

                drive.followTrajectorySequence(trajectory);
                telemetry.addData("case 2", tagOfInterest.pose.x);
                telemetry.update();
            }

            if (tagOfInterest.pose.x > 0.5 && !isStopRequested()) {

                drive.setPoseEstimate(new Pose2d(13, -60, Math.toRadians(90)));
                TrajectorySequence trjct = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                        .splineTo(new Vector2d(15.9, -35.9), Math.toRadians(40))
                        .back(7)
                        .lineToSplineHeading(new Pose2d(11, -11, Math.toRadians(180)))
                        .lineToSplineHeading(new Pose2d(59, -11, Math.toRadians(180)))
                        .build();

                drive.followTrajectorySequence(trjct);
                telemetry.addData("case 3", tagOfInterest.pose.x);
                telemetry.update();
            }
        }
        else {
            drive.setPoseEstimate(new Pose2d(13, -60, Math.toRadians(90)));
            TrajectorySequence traiect = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                    .splineTo(new Vector2d(7.5, -35.9), Math.toRadians(130))
                    .back(10)
                    .splineTo(new Vector2d(59, -11), Math.toRadians(0))
                    .build();

            drive.followTrajectorySequence(traiect);
        }





    }

    void tagToTelemetry(AprilTagDetection detection)
    {
        telemetry.addLine(String.format("\nDetected tag ID=%d", detection.id));
    }
}
