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

        double speed = 85;

        double glisPwr = 0;

        PIDglis pidGlis = new PIDglis(Kp, Ki, Kd);

        double glisPos = 0;


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
                robot.colectare.setPower(0.5);
                gamepad2.rumbleBlips(1);
            }
            else
                robot.colectare.setPower(0);

            if (gamepad2.a)
                robot.colectare.setPower(-0.5);

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
            glisPos = robot.glisiere_dr.getCurrentPosition();

            if(gamepad2.right_bumper)
                pidGlis.setTargetPosition(poz_sus);

            if(gamepad2.left_bumper)
                pidGlis.setTargetPosition(0);

            glisPwr = pidGlis.update(glisPos);

            if(gamepad2.dpad_up){
                robot.glisiere_dr.setPower(1);
                robot.glisiere_st.setPower(1);
            }
            else {
                robot.glisiere_dr.setPower(glisPwr);
                robot.glisiere_st.setPower(glisPwr);
            }

            if(gamepad2.dpad_down){
                robot.glisiere_dr.setPower(-1);
                robot.glisiere_st.setPower(-1);
            }

            Telemetry.addData("viteza coaie: ", speed);
            Telemetry.addData("heading", Math.toDegrees(drive.getExternalHeading()));
            Telemetry.addData("GLIS ST", robot.glisiere_st.getCurrentPosition());
            Telemetry.addData("GLIS DR", robot.glisiere_dr.getCurrentPosition());
            Telemetry.addData("target poz", poz_sus);
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
    public void urca_glis(float pos){
        float err = 70;

        float dif = pos - robot.glisiere_dr.getCurrentPosition();
        float sgn = Math.signum(dif);


        while (robot.glisiere_dr.getCurrentPosition() > pos + err || robot.glisiere_dr.getCurrentPosition() < pos - err){
            robot.glisiere_st.setPower(1 * sgn);
            robot.glisiere_dr.setPower(1 * sgn);

            dif = pos - robot.glisiere_dr.getCurrentPosition();
            sgn = Math.signum(dif);
        }

        robot.glisiere_st.setPower(0);
        robot.glisiere_dr.setPower(0);
    }

    public void cob_glis(){
        float err = 70;

        float dif = robot.glisiere_dr.getCurrentPosition();
        float sgn = Math.signum(dif);


        while (robot.glisiere_dr.getCurrentPosition() > err || robot.glisiere_dr.getCurrentPosition() < - err){
            robot.glisiere_st.setPower(-1 * sgn);
            robot.glisiere_dr.setPower(-1 * sgn);

            dif = robot.glisiere_dr.getCurrentPosition();
            sgn = Math.signum(dif);
        }

        robot.glisiere_st.setPower(0);
        robot.glisiere_dr.setPower(0);
    }

}



