package org.firstinspires.ftc.teamcode.auto.Rosu;

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


@Autonomous(name = "Rosu Far Case 3")
public class RosuFarTrei extends LinearOpMode {


    @Override
    public void runOpMode() {

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        HMap robot = new HMap();



        robot.init(hardwareMap);


        telemetry.setMsTransmissionInterval(50);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        telemetry.update();


        waitForStart();

        drive.setPoseEstimate(new Pose2d(-35, -60, Math.toRadians(90)));

        TrajectorySequence traiect = drive.trajectorySequenceBuilder( new Pose2d(-35, -60, Math.toRadians(90)))
                .splineTo(new Vector2d(-40, -35), Math.toRadians(130))
                .back(5)
                .turn(1)
                .lineTo(new Vector2d(-36.6, -9))
                .lineTo(new Vector2d(65, -11))
                .build();

        drive.followTrajectorySequence(traiect);


        telemetry.update();

    }
}
