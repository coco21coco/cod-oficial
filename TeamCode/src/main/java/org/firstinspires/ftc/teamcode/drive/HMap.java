package org.firstinspires.ftc.teamcode.drive;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.acmerobotics.dashboard.config.Config;
@Config
public class HMap {


    public BNO055IMU imu = null;
    public DcMotorEx colectare = null,
                     glisiere = null;

    public Servo cutie = null,
                avion = null;

    public static double cutie_inkis = 0.25,
                    cutie_deskis = 0,

                    avion_armat = 0.45,
                    avion_tras = 0.7;
   public void init(HardwareMap hmap){

       imu = hmap.get(BNO055IMU.class, "imu");

      colectare = hmap.get(DcMotorEx.class, "colectare");
      glisiere = hmap.get(DcMotorEx.class, "glisiere");

      glisiere.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

      cutie = hmap.get(Servo.class, "cutie");
      avion = hmap.get(Servo.class, "avion");

      avion.setPosition(avion_armat);
      cutie.setPosition(cutie_inkis);
   }


}
