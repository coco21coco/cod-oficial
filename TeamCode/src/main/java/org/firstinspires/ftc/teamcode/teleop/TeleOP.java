package org.firstinspires.ftc.teamcode.teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.drive.HMap;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;


@TeleOp(name = "TeleOP", group = "TeleOP")
@Config

public class TeleOP extends LinearOpMode {
    //HMap robot = new HMap();

    @Override
    public void runOpMode() throws InterruptedException {
        //robot.init(hardwareMap);

        HMap robot = new HMap();

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        FtcDashboard dashboard = FtcDashboard.getInstance();
        Telemetry dashtelemtry = dashboard.getTelemetry();

        double speed = 100;

        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.init(hardwareMap);

        waitForStart();


        while(opModeIsActive()) {

            drive.setWeightedDrivePower(
                    new Pose2d(
                            gamepad1.left_stick_y * speed / 100,
                            gamepad1.left_stick_x * speed / 100,
                            -gamepad1.right_stick_x * speed / 100
                    )
            );

            drive.update();
            Pose2d poseEstimate = drive.getPoseEstimate();


            if(gamepad1.a)
                speed = 50;
            if(gamepad1.b )
                speed = 85;

            // colectare

            if(gamepad2.b)
                robot.colectare.setPower(1);
            else
                robot.colectare.setPower(0);

            if(gamepad2.a)
                robot.colectare.setPower(-1);

            // cutie

            if(gamepad2.left_stick_button) {
                robot.cutie.setPosition(robot.cutie_deskis);
                sleep(800);
                robot.cutie.setPosition(robot.cutie_inkis);
            }

            if(gamepad2.y){
                robot.avion.setPosition(robot.avion_tras);
                sleep(200);
                robot.avion.setPosition(robot.avion_armat);
            }
            // glisiere
            if(gamepad2.dpad_down)
                robot.glisiere.setPower(1);
            else
                robot.glisiere.setPower(0);

            if(gamepad2.dpad_up)
                robot.glisiere.setPower(-1);



            telemetry.addData("viteza coaie: ", speed);
            telemetry.addData("x", poseEstimate.getX());
            telemetry.addData("y", poseEstimate.getY());
            telemetry.addData("heading", poseEstimate.getHeading());
            telemetry.update();
        }
    }
}
