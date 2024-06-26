package org.firstinspires.ftc.teamcode.auto.Rosu;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.teamcode.PIDglis;
import org.firstinspires.ftc.teamcode.drive.HMap;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.tfod.TfodProcessor;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

import java.util.List;

@Autonomous(name = "Rosu Close Colt")
public class RosuClloseColt extends LinearOpMode {
    private static final boolean USE_WEBCAM = true;  // true for webcam, false for phone camera
    private static final String TFOD_MODEL_FILE = "/sdcard/FIRST/tflitemodels/RED_TSE.tflite";
    private static final String[] LABELS = {
            "TSE"
    };

    private TfodProcessor tfod;

    private VisionPortal visionPortal;

    public static double Kp = 0.005,
            Ki = 0.00001,
            Kd = 0.00001;

    @Override
    public void runOpMode() {

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        HMap robot = new HMap();


        robot.init(hardwareMap);


        telemetry.setMsTransmissionInterval(50);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        telemetry.update();

        initTfod();

        while(!opModeIsActive() && !isStopRequested()) {

            List<Recognition> currentRecognition = tfod.getRecognitions();

            for (Recognition TSE : currentRecognition) {


                if (TSE.getLeft() < 200) {
                    telemetry.addData("caz 1", TSE.getLeft());
                    telemetry.update();
                }

                if (TSE.getLeft() >= 200) {
                    telemetry.addData("caz 2", TSE.getLeft());
                    telemetry.update();
                }

            }

            if (currentRecognition.size() == 0) {
                telemetry.addData("caz3", " ");
                telemetry.update();
            }
        }

        waitForStart();

        visionPortal.close();

        if(opModeIsActive() && !isStopRequested()){
            List<Recognition> currentRecognition = tfod.getRecognitions();

            for (Recognition TSE : currentRecognition) {

                //caz 1
                if (TSE.getLeft() < 200) {
                    drive.setPoseEstimate(new Pose2d(13, -60, Math.toRadians(90)));

                    TrajectorySequence traiect = drive.trajectorySequenceBuilder( new Pose2d(13, -60, Math.toRadians(90)))
                            .lineToSplineHeading(new Pose2d(11, -36, Math.toRadians(130)))
                            .addTemporalMarker(()-> robot.colectare.setPower(1))
                            .waitSeconds(1)
                            .lineToSplineHeading(new Pose2d(42, -35, Math.toRadians(180)))
                            .addDisplacementMarker(()->robot.colectare.setPower(-1))
                            .build();

                    TrajectorySequence panou = drive.trajectorySequenceBuilder(new Pose2d(42, -35, Math.toRadians(180)))
                            .addDisplacementMarker(0, ()->coboara(robot))
                            .strafeLeft(28)
                            .back(10)
                            .build();

                    drive.followTrajectorySequence(traiect);
                    robot.colectare.setPower(0);
                    urca(robot);
                    robot.extinde_cutie();
                    sleep(500);
                    robot.deschide_cutie();
                    sleep(1000);
                    robot.strange_cutie();
                    coboara(robot);
                    drive.followTrajectorySequence(panou);


                    telemetry.update();
                }

                //caz 2
                if (TSE.getLeft() >= 200) {

                    drive.setPoseEstimate(new Pose2d(13, -60, Math.toRadians(90)));

                    TrajectorySequence traiect = drive.trajectorySequenceBuilder( new Pose2d(13, -60, Math.toRadians(90)))
                            .forward(14)
                            .addTemporalMarker(()-> robot.colectare.setPower(1))
                            .waitSeconds(1)
                            .back(3)
                            .lineToSplineHeading(new Pose2d(45, -43, Math.toRadians(180)))
                            .build();

                    TrajectorySequence panou = drive.trajectorySequenceBuilder(new Pose2d(46, -35, Math.toRadians(180)))
                            .strafeLeft(25)
                            .back(6)
                            .build();

                    drive.followTrajectorySequence(traiect);
                    robot.colectare.setPower(0);
                    urca(robot);
                    robot.extinde_cutie();
                    sleep(500);
                    robot.deschide_cutie();
                    sleep(1000);
                    robot.strange_cutie();
                    coboara(robot);
                    drive.followTrajectorySequence(panou);
                }

            }

            //caz 3
            if (currentRecognition.size() == 0) {

                drive.setPoseEstimate(new Pose2d(13, -60, Math.toRadians(90)));

                TrajectorySequence traiect = drive.trajectorySequenceBuilder( new Pose2d(13, -60, Math.toRadians(90)))
                        .lineToLinearHeading(new Pose2d(11, -38, Math.toRadians(40)))
                        .addTemporalMarker(()-> robot.colectare.setPower(1))
                        .waitSeconds(1)
                        .back(4)
                        .lineToSplineHeading(new Pose2d(45, -47, Math.toRadians(180)))
                        .build();

                TrajectorySequence panou = drive.trajectorySequenceBuilder(new Pose2d(46, -30, Math.toRadians(180)))
                        .strafeLeft(25)
                        .back(10)
                        .build();

                drive.followTrajectorySequence(traiect);
                robot.colectare.setPower(0);
                urca(robot);
                robot.extinde_cutie();
                sleep(500);
                robot.deschide_cutie();
                sleep(1000);
                robot.strange_cutie();
                coboara(robot);
                drive.followTrajectorySequence(panou);
            }
        }



    }   // end runOpMode()

    private void initTfod() {


        tfod = new TfodProcessor.Builder()


                .setModelFileName(TFOD_MODEL_FILE)
                .setModelLabels(LABELS)

                .setIsModelQuantized(true)
                .build();

        VisionPortal.Builder builder = new VisionPortal.Builder();


        if (USE_WEBCAM) {
            builder.setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"));
        } else {
            builder.setCamera(BuiltinCameraDirection.BACK);
        }

        builder.addProcessor(tfod);

        visionPortal = builder.build();



    }

    private void telemetryTfod() {

        List<Recognition> currentRecognitions = tfod.getRecognitions();
        telemetry.addData("# Objects Detected", currentRecognitions.size());

        // Step through the list of recognitions and display info for each one.
        for (Recognition recognition : currentRecognitions) {
            double x = (recognition.getLeft() + recognition.getRight()) / 2 ;
            double y = (recognition.getTop()  + recognition.getBottom()) / 2 ;

            telemetry.addData(""," ");
            telemetry.addData("Image", "%s (%.0f %% Conf.)", recognition.getLabel(), recognition.getConfidence() * 100);
            telemetry.addData("- Position", "%.0f / %.0f", x, y);
            telemetry.addData("- Size", "%.0f x %.0f", recognition.getWidth(), recognition.getHeight());
        }   // end for() loop

    }   // end method telemetryTfod()

    public void urca(HMap r){

        PIDglis pidGlis = new PIDglis(Kp, Ki, Kd);

        double glisPoz =0;
        double glisPwr = 0;

        pidGlis.setTargetPosition(1000);

        while(glisPoz <= 800 && opModeIsActive()){
            glisPoz = r.glisiere_dr.getCurrentPosition();

            r.glisiere_st.setPower(glisPwr);
            r.glisiere_dr.setPower(glisPwr);

            glisPwr = pidGlis.update(glisPoz);
        }


    }

    public void coboara(HMap r){

        PIDglis pidGlis = new PIDglis(Kp, Ki, Kd);

        double glisPoz = 1000;
        double glisPwr = 0;

        pidGlis.setTargetPosition(0);

        while(glisPoz >= 0 && opModeIsActive()){
            glisPoz = r.glisiere_dr.getCurrentPosition();

            r.glisiere_st.setPower(glisPwr);
            r.glisiere_dr.setPower(glisPwr);

            glisPwr = pidGlis.update(glisPoz);
        }

    }
}
