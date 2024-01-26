package org.firstinspires.ftc.teamcode.drive;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.acmerobotics.dashboard.config.Config;
@Config
public class HMap {



    public DcMotorEx colectare = null,
                     glisiere_dr = null,
                     glisiere_st = null;

    public Servo cutie = null,
                avion = null;

    public static double cutie_inkis = 0.7,
                    cutie_deskis = 0.5,

                    avion_armat = 0.54,
                    avion_tras = 0.7;
   public void init(HardwareMap hmap){

       

      colectare = hmap.get(DcMotorEx.class, "colectare");
      glisiere_dr = hmap.get(DcMotorEx.class, "glisiere_dr");
      glisiere_st = hmap.get(DcMotorEx.class, "glisiere_st");

      glisiere_dr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
      glisiere_st.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

      glisiere_dr.setDirection(DcMotorSimple.Direction.REVERSE);

       glisiere_st.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
       glisiere_dr.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

      glisiere_dr.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
      glisiere_st.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


      cutie = hmap.get(Servo.class, "cutie");
      avion = hmap.get(Servo.class, "avion");

      avion.setPosition(avion_armat);
      cutie.setPosition(cutie_inkis);
   }


}
