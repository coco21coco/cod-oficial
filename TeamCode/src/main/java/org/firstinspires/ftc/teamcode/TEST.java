package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@TeleOp(name = "test")
public class TEST extends LinearOpMode {

    @Override
    public void runOpMode(){

        DcMotorEx test = hardwareMap.get(DcMotorEx.class, "test");

        waitForStart();
        while(opModeIsActive()){

            if(gamepad1.x)
                test.setPower(1);
            else
                test.setPower(0);

            if(gamepad1.y)
                test.setPower(-1);
        }
    }
}
