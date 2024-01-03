package org.firstinspires.ftc.teamcode.drive;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.acmerobotics.dashboard.config.Config;
@Config
public class HMap {



    public DcMotorEx colectare = null,
                     glisiere = null;

    public Servo cutie = null,
                avion = null;

    public static double cutie_inkis = 0.7,
                    cutie_deskis = 0.5,

                    avion_armat = 59,
                    avion_tras = 0.7;
   public void init(HardwareMap hmap){

       

      colectare = hmap.get(DcMotorEx.class, "colectare");
      glisiere = hmap.get(DcMotorEx.class, "glisiere");

      glisiere.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

      cutie = hmap.get(Servo.class, "cutie");
      avion = hmap.get(Servo.class, "avion");

      avion.setPosition(avion_armat);
      cutie.setPosition(cutie_inkis);
   }


}
