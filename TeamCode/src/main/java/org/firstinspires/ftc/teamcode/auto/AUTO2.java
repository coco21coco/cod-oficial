package org.firstinspires.ftc.teamcode.auto;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;


@Autonomous(name = "Auto departe")
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
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        Trajectory traiectorie = drive.trajectoryBuilder(new Pose2d(-37, -57, 0))
                .forward(40)
                .splineTo(new Vector2d(35, -11),0)
                .forward(16)
                .build();

        drive.followTrajectory(traiectorie);


    }
}
