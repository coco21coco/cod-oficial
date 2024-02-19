package org.firstinspires.ftc.teamcode.auto.Rosu;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.teamcode.PIDglis;
import org.firstinspires.ftc.teamcode.drive.HMap;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.tfod.TfodProcessor;

import java.util.List;

@Autonomous(name = "Rosu Close Mjl")
public class RosuClose extends LinearOpMode {
    private static final boolean USE_WEBCAM = true;  // true for webcam, false for phone camera
    private static final String TFOD_MODEL_FILE = "/sdcard/FIRST/tflitemodels/RED_TSE.tflite";
    private static final String[] LABELS = {
            "TSE"
    };

    private TfodProcessor tfod;

    private VisionPortal visionPortal;


    public  static int INATLTIME_GLIS = 100;

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
                            .lineToSplineHeading(new Pose2d(10, -35.9, Math.toRadians(130)))
                            .back(0.1)
                            .lineToSplineHeading(new Pose2d(46, -35, Math.toRadians(180)))
                            .back(1)
                            .strafeRight(18)
                            .back(10)
                            .build();

                    drive.followTrajectorySequence(traiect);


                    telemetry.update();
                }

                //caz 2
                if (TSE.getLeft() >= 200) {

                    //se incearca a se pune in auto

                    drive.setPoseEstimate(new Pose2d(13, -60, Math.toRadians(90)));

                    TrajectorySequence traiect = drive.trajectorySequenceBuilder( new Pose2d(13, -60, Math.toRadians(90)))
                            .forward(17.5)
                            .back(3)
                            .lineToSplineHeading(new Pose2d(46, -40, Math.toRadians(180)))
                            .build();


                    TrajectorySequence panou = drive.trajectorySequenceBuilder(new Pose2d(46, -40, Math.toRadians(180)))
                            .addDisplacementMarker(0, ()->coboara(robot))
                            .strafeRight(18)
                            .back(10)
                                    .build();
                     drive.followTrajectorySequence(traiect);
                     urca(robot);
                     sleep(1000);
                     drive.followTrajectorySequence(panou);
                }

            }

            //caz 3
            if (currentRecognition.size() == 0) {

                drive.setPoseEstimate(new Pose2d(13, -60, Math.toRadians(90)));

                TrajectorySequence traiect = drive.trajectorySequenceBuilder( new Pose2d(13, -60, Math.toRadians(90)))
                        .lineToSplineHeading(new Pose2d(17, -35.9, Math.toRadians(40)))
                        .back(4)
                        .lineToSplineHeading(new Pose2d(46, -35, Math.toRadians(180)))
                        .back(1)
                        .strafeRight(18)
                        .back(10)
                        .build();

                drive.followTrajectorySequence(traiect);
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

       while(glisPoz <= 800){
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

       while(glisPoz >= 0){
           glisPoz = r.glisiere_dr.getCurrentPosition();

           r.glisiere_st.setPower(glisPwr);
           r.glisiere_dr.setPower(glisPwr);

           glisPwr = pidGlis.update(glisPoz);
       }

   }
}
