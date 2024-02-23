package org.firstinspires.ftc.teamcode.teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.PIDglis;
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



    HMap robot = new HMap();

    public static float poz_sus = 800;

    public static double Kp = 0.005,
                        Ki = 0.00001,
                        Kd = 0.00001;
    @Override
    public void runOpMode() throws InterruptedException {
        //robot.init(hardwareMap);


        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        FtcDashboard dashboard = FtcDashboard.getInstance();

        Telemetry dashtelemtry = dashboard.getTelemetry();

        MultipleTelemetry Telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());



        double speed = 85;

        double glisPwr = 0;

        PIDglis pidGlis = new PIDglis(Kp, Ki, Kd);

        double glisPos = 0;

        boolean glis_toggle = false; // automat

        boolean toggle_unghi = false;
        double unghi_curent = 0;

        boolean urca = false;
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

            if(gamepad2.dpad_right){
                //robot.cutie_st.setPosition(0);
                //robot.cutie_dr.setPosition(1);
            }

            if(gamepad2.dpad_left){
                //robot.cutie_st.setPosition(0.15);
                //robot.cutie_dr.setPosition(0.7);
            }

            if (gamepad2.y) {
                robot.trage_avion();
                sleep(500);
                robot.armeaza_avion();
            }
            // glisiere
            glisPos = robot.glisiere_dr.getCurrentPosition();

            if(gamepad2.right_stick_button)
                glis_toggle = false;

            if(gamepad2.left_stick_button) {
                glis_toggle = true;
                urca = false;
            }


            if(gamepad2.right_bumper) {
                pidGlis.setTargetPosition(poz_sus);
                urca = true;
            }

            if(gamepad2.left_bumper) {
                pidGlis.setTargetPosition(0);
                urca = false;
            }

            glisPwr = pidGlis.update(glisPos);

            if(gamepad2.dpad_up){
                robot.glisiere_dr.setPower(1);
                robot.glisiere_st.setPower(1);

                urca = true;
            }
            else if(glis_toggle){
                robot.glisiere_dr.setPower(glisPwr);
                robot.glisiere_st.setPower(glisPwr);

            }

            else {
                robot.glisiere_dr.setPower(0);
                robot.glisiere_st.setPower(0);
            }

            if(gamepad2.dpad_down){
                robot.glisiere_dr.setPower(-1);
                robot.glisiere_st.setPower(-1);

                urca = false;
            }

            if(glisPos >= 500 && urca == true)
                robot.extinde_cutie();

            if(urca == false)
                robot.strange_cutie();

            if(gamepad1.y)
                robot.deschide_cutie();

            if(gamepad1.x)
                robot.inchide_cutie();



            Telemetry.addData("viteza: ", speed);
            Telemetry.addData("heading", Math.toDegrees(drive.getExternalHeading()));
            Telemetry.addData("GLIS ST", robot.glisiere_st.getCurrentPosition());
            Telemetry.addData("GLIS DR", robot.glisiere_dr.getCurrentPosition());
            Telemetry.addData("target poz", poz_sus);
            Telemetry.addData("GLIS AUTOMAT: ", glis_toggle);
            Telemetry.update();
        }



    }



}



