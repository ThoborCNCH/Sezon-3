package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.vuforia.CameraDevice;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import java.util.List;
import java.util.Random;

/**
 * COPYRIGHT THOBOR 2018 - 2019 ROVER RUCKUS
 */

@Autonomous(name = "Cuva_BRIDGE_ALBASTRU", group = "CUVA")
//@Disabled
public class Cuva_BRIDGE_ALBASTRU extends LinearOpMode {

    private static final String VUFORIA_KEY = "AT0wukP/////AAABmf2bN3XJaEQjssVFQTY/tbpBmxq2HJn4pooNk0lDszeT1PG7XhV17uBFclV5ILXSgsxUTc0Fepi7bAL5sJD0KW7GWxnoBiLUM67TF6mcD5dNi0vYVJ6iTgWSuHyTdab/nm9jDuZenQpMs6/XHL7nva/NbepfRn49kIrVq0rGQvH51c3z5OqDlxNMUg5Q60g0UroDxmIVLJsPbRPj8HLC3zQEfvz5uXfCWg853pF06UrCRDILcTXz2WUCEJwQMiniJyWSrz6tN96YT1k8dhScMIjpyc7KtsAm0lxYApq23rTieciVzRB7+OMFbgFEuA/BetmBt8+z2ybMZPLgR0veOoU2uT1qu2wBcj+GXKHjRCeX";

    /**
     * {@link #vuforia} is the variable we will use to store our instance of the Vuforia
     * localization engine.
     */
    private VuforiaLocalizer vuforia;

    /**
     * {@link #tfod} is the variable we will use to store our instance of the Tensor Flow Object
     * Detection engine.
     */
    private TFObjectDetector tfod;

    /* Declare OpMode members. */

    private DcMotor RF, LF, LR, RR;
    public ElapsedTime mRunTime = new ElapsedTime();
    Servo fdr, fst;
    DcMotor absortie1;
    DcMotor absortie2;
    DcMotor ridicare2;
    Servo gheara;
    Servo gheara1, gheara2;
    Servo capstone;
    DcMotor ridicare;
    CRServo extindere = null;


    double diametru =64;
    double raza= diametru / 2;


    @Override
    public void runOpMode() throws InterruptedException {

        //initializare hard
        initializare_hard();
        
        /** Wait for the game to begin */
        telemetry.addData(">", "Press Play to start tracking");
        telemetry.update();

        // Do not use waitForStart() if you have Motorola E4 phones.
        //waitForStart();
        while (!opModeIsActive() && !isStopRequested()) {

            telemetry.update();
        }

        if (opModeIsActive()) {

            capstone.setPosition(-0.6);
            miscare_fata(1500);
            miscare_stanga(2000);
            miscare_fata(600);
            oprire(300);
            apucare_cuva();
            oprire(1000);
            
            miscare_spate(2650);
            ridicare_cuva();
            oprire(1000);
            miscare_fata(50);
            
            miscare_dreapta(2100);
            miscare_fata(700);
            miscare_stanga(650);
            miscare_fata(1130);
            miscare_dreapta(1900);
            
            apucare_cuva();
           
            

        }

    }


    /**
     * Initialize the Vuforia localization engine.
     */
    private void initVuforia() {
        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         */
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        // Loading trackables is not necessary for the Tensor Flow Object Detection engine.
    }

    /**
     * Initialize the Tensor Flow Object Detection engine.
     */


    public void initializare_hard() {

        //motoare roti
        LF = hardwareMap.dcMotor.get("leftFront");
        RF = hardwareMap.dcMotor.get("rightFront");
        RR = hardwareMap.dcMotor.get("rightRear");
        LR = hardwareMap.dcMotor.get("leftRear");
        LF.setDirection(DcMotor.Direction.FORWARD);
        RF.setDirection(DcMotor.Direction.REVERSE);
        LR.setDirection(DcMotor.Direction.FORWARD);
        RR.setDirection(DcMotor.Direction.REVERSE);
        //gheare cuva
        gheara2 = hardwareMap.get(Servo.class,"fst");
        gheara1 = hardwareMap.get(Servo.class,"fdr");
      
        capstone = hardwareMap.get(Servo.class,"capstone");
        gheara1.setPosition(-.5);
        gheara2.setPosition(1);
        capstone.setPosition(-0.6);
        
        /*

        bratdreapta = hardwareMap.get(DcMotor.class,"brat1");
        bratstanga = hardwareMap.get(DcMotor.class,"brat2");  */



    }

    public void deplasare(double lm1, double lm2, double rm1, double rm2) {
        
        LF.setMode(DcMotor.RunMode.RUN_USING_ENCODERS);
        RF.setMode(DcMotor.RunMode.RUN_USING_ENCODERS);
        LR.setMode(DcMotor.RunMode.RUN_USING_ENCODERS);
        RR.setMode(DcMotor.RunMode.RUN_USING_ENCODERS);
        
        LF.setPower(lm1);
        RF.setPower(lm2);
        LR.setPower(rm1);
        RR.setPower(rm2);
    }

    public void run_without_encoders()
    {LF.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        RF.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        LR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        RR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        deplasare(0,0,0,0);
    }
    public void oprire_encoder()
    {
        LF.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RF.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        LF.setMode(DcMotor.RunMode.RUN_USING_ENCODERS);
        RF.setMode(DcMotor.RunMode.RUN_USING_ENCODERS);
        LR.setMode(DcMotor.RunMode.RUN_USING_ENCODERS);
        RR.setMode(DcMotor.RunMode.RUN_USING_ENCODERS);
        
        //deplasare(0,0,0,0) ;
        // Set power to 0
    }

    public void deplasare_encoder(int lm1, int lm2, int rm1, int rm2) 
    {
        oprire_encoder();
        
        LF.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        RF.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        LR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        RR.setMode(DcMotor.RunMode.RUN_TO_POSITION); 

        LF.setTargetPosition(lm1);
        RF.setTargetPosition(lm2);
        LR.setTargetPosition(rm1);
        RR.setTargetPosition(rm2);
    }

   public void miscare_fata(long milisecunde) {
        deplasare(0.5, 0.5, 0.5, 0.5);
        sleep(milisecunde);
    }

    public void miscare_dreapta(long milisecunde) {
        deplasare(0.5,-0.5,-0.5,0.5);
        sleep(milisecunde);
    }

    public void miscare_stanga(long milisecunde) {
        deplasare(-0.5,+0.5,+0.5,-0.5);
        sleep(milisecunde);
    }

 

    public void miscare_spate(long milisecunde) {
        deplasare(-0.5,-0.5,-0.5,-0.5);
        sleep(milisecunde);
    }
    public void oprire(long milisecunde) {
        deplasare(0,0,0,0) ;
        sleep(milisecunde);
    }
    public void ridicare_cuva ()
    { gheara1.setPosition(-.5);
        gheara2.setPosition(1);
    }
    public void apucare_cuva()
    {gheara1.setPosition(1);
     gheara2.setPosition(0); }
     
     
      
    

    void extindere(long milisecunde)
    {
        extindere.setPower(-1);
        sleep(milisecunde);
        extindere.setPower(0);
    }
    

    public void rotire_poz_dr_neg_st(double alfa)
    {   
        oprire_encoder();
        
        double  distanta =( 2 * 3.14 * raza * (alfa / 360.)) / 2.54;   
        
        double circumference = 3.14 * 3.937;
        double rotationNeeded = distanta/circumference;
        int encoderDrivingTarget = (int)(rotationNeeded * 1120);
        
        deplasare_encoder(encoderDrivingTarget, encoderDrivingTarget, encoderDrivingTarget, encoderDrivingTarget);
        
        LF.setPower(1);
        RF.setPower(1);
        LR.setPower(1);
        RR.setPower(1);
        
         while(LF.isBusy()||RF.isBusy()||LR.isBusy()||RR.isBusy())
        { 
            telemetry.addData("Parcurcand distanta",distanta);
            telemetry.addData("[*]Distanta Calcul", "Running to %7d :%7d", encoderDrivingTarget, encoderDrivingTarget);
            telemetry.addData("[**]Distanta spate", "Running at %7d :%7d", LR.getCurrentPosition(), RR.getCurrentPosition());
            telemetry.addData("[***]Distanta fata", "Running at %7d :%7d", LF.getCurrentPosition(), RF.getCurrentPosition());
             
            telemetry.update();
            
        }
        //oprire_encoder();
        

    }


    public void stop_move() 
    {
        deplasare(0,0,0,0) ;

    }


    public void oprire_vuforia() {
        CameraDevice.getInstance().setFlashTorchMode(false);
        if (tfod != null) {
            tfod.shutdown();
        }
    }

}
