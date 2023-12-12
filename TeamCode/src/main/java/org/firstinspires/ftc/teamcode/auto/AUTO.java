package org.firstinspires.ftc.teamcode.auto;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.drive.HMap;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;


@Autonomous(name = "Auto Close")
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
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        HMap robot = new HMap();

        waitForStart();
        robot.init(hardwareMap);

       Trajectory traiectorie = drive.trajectoryBuilder(new Pose2d(13, -60, 0))
                .forward(70)
                .build();

       drive.followTrajectory(traiectorie);

       robot.colectare.setPower(1);
       sleep(200);
       robot.colectare.setPower(0);

    }
}
