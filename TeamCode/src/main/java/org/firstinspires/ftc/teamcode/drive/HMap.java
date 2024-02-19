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

    public Servo cutie_dr = null,
                cutie_st = null,
                avion = null,
                cutie = null;

    public static double cutie_inkis = 0.7,
                    cutie_deskis = 0.3,

                    avion_armat = 0.54,
                    avion_tras = 0.7,
                    cutie_dr_extins = 0.7,
                    cutie_st_extins = 0.3,
                    cutie_dr_strans = 0.95,
                    cutie_st_strans  = 0.05;
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


       cutie_dr = hmap.get(Servo.class, "cutie_dr");
       cutie_st = hmap.get(Servo.class, "cutie_st");

       cutie = hmap.get(Servo.class, "cutie");
       strange_cutie();
       deschide_cutie();


   }

   public void extinde_cutie(){
       cutie_dr.setPosition(cutie_dr_extins);
       cutie_st.setPosition(cutie_st_extins);
   }

    public void strange_cutie(){
        cutie_dr.setPosition(cutie_dr_strans);
        cutie_st.setPosition(cutie_st_strans);
    }

    public void deschide_cutie()
    {
        cutie.setPosition(cutie_deskis);
    }

    public void inchide_cutie()
    {
        cutie.setPosition(cutie_inkis);
    }

    public void trage_avion(){
       avion.setPosition(avion_tras);
    }

    public void armeaza_avion(){
       avion.setPosition(avion_armat);
    }

}
