package org.firstinspires.ftc.teamcode.auto.Albastru;

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
@Autonomous(name = "Albastru Close Case 1")
public class AlbastruCloseUnu extends LinearOpMode {





    @Override
    public void runOpMode() {

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        HMap robot = new HMap();

        robot.init(hardwareMap);


        telemetry.setMsTransmissionInterval(50);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        telemetry.update();


        waitForStart();

        drive.setPoseEstimate(new Pose2d(11, 60, Math.toRadians(-90)));
        TrajectorySequence traiect = drive.trajectorySequenceBuilder(new Pose2d(11, 60, Math.toRadians(-90)))
                .lineToSplineHeading(new Pose2d(15, 40, Math.toRadians(-45)))
                .back(5)
                .lineToSplineHeading(new Pose2d(11, 11, Math.toRadians(0)))
                .lineToSplineHeading(new Pose2d(59, 10, Math.toRadians(0)))
                .build();

        drive.followTrajectorySequence(traiect);


        telemetry.update();

    }
}
