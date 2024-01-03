package org.firstinspires.ftc.teamcode.auto.Albastru;

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
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.ArrayList;


@Autonomous(name = "Albastru Far Case 3")
public class AlbastruFarTrei extends LinearOpMode {



    @Override
    public void runOpMode() {


        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        HMap robot = new HMap();

        robot.init(hardwareMap);


        telemetry.setMsTransmissionInterval(50);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        telemetry.update();


        waitForStart();


        TrajectorySequence traiect = drive.trajectorySequenceBuilder( new Pose2d(-35, 60, Math.toRadians(-90)))
                .lineToSplineHeading(new Pose2d(-40, 35, Math.toRadians(-135)))
                .back(5)
                .lineToSplineHeading(new Pose2d(-35,14, Math.toRadians(180)))
                .lineTo(new Vector2d(59, 10))
                .build();

        drive.followTrajectorySequence(traiect);


        telemetry.update();

    }
}
