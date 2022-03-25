package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

import java.io.InterruptedIOException;

@Autonomous(name="Autonom_GYRO", group="Autonom_GYRO")
public class Autonom_GYRO extends LinearOpMode {
    public ElapsedTime runtime = new ElapsedTime();
    public DcMotor leftfwheel, leftbwheel, rightfwheel, rightbwheel;
     BNO055IMU imu;
     Orientation angles;


    @Override
    public void runOpMode() throws InterruptedException {
        BNO055IMU.Parameters parameters= new BNO055IMU.Parameters();
        parameters.angleUnit= BNO055IMU.AngleUnit.DEGREES;

        imu=hardwareMap.get(BNO055IMU.class, "imu 1");
        imu.initialize(parameters);

        leftfwheel=hardwareMap.get(DcMotor.class, "leftFront");
        leftbwheel=hardwareMap.get(DcMotor.class, "leftRear");
        rightfwheel=hardwareMap.get(DcMotor.class, "rightFront");
        rightbwheel=hardwareMap.get(DcMotor.class, "rightRear");

        leftfwheel.setDirection(DcMotor.Direction.REVERSE);
        leftbwheel.setDirection(DcMotor.Direction.REVERSE);
        rightfwheel.setDirection(DcMotor.Direction.FORWARD);
        rightbwheel.setDirection(DcMotor.Direction.FORWARD);

        waitForStart();
        while(opModeIsActive() && !isStopRequested()){
            angles=imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            telemetry.addData("Heading:", angles.firstAngle);
            telemetry.addData("roll:", angles.secondAngle);
            telemetry.addData("pintch:", angles.thirdAngle);
            telemetry.update();

            /*angles=imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
             if(angles.firstAngle>-1 && angles.firstAngle<1)
             { leftfwheel.setPower(-0.4);
            leftbwheel.setPower(-0.4);
            rightfwheel.setPower(-0.4);
            rightbwheel.setPower(-0.4);}
            if(angles.firstAngle>1)
            
                {leftfwheel.setPower(-0.2);
                leftbwheel.setPower(-0.2);
                rightfwheel.setPower(0.2);
                rightbwheel.setPower(0.2); }
                
            if(angles.firstAngle<-1)
                {leftfwheel.setPower(0.2);
                leftbwheel.setPower(0.2);
                rightfwheel.setPower(-0.2);
                rightbwheel.setPower(-0.2); }  
                //cod rotire stanga 90 grade
               /* if(angles.firstAngle <70)
                {leftfwheel.setPower(0.7);
                leftbwheel.setPower(0.7);
                rightfwheel.setPower(-0.7);
                rightbwheel.setPower(-0.7); }
                else  if(angles.firstAngle<90)
                {leftfwheel.setPower(0.15);
                leftbwheel.setPower(0.15);
                rightfwheel.setPower(-0.15);
                rightbwheel.setPower(-0.15); }
                    else
                {leftfwheel.setPower(0);
                leftbwheel.setPower(0);
                rightfwheel.setPower(0);
                rightbwheel.setPower(0); } */
                
                //cod rotire dreapta 90 grade
                /*if(angles.firstAngle >-70)
                {leftfwheel.setPower(-0.7);
                leftbwheel.setPower(-0.7);
                rightfwheel.setPower(0.7);
                rightbwheel.setPower(0.7); }
                else  if(angles.firstAngle>-90)
                {leftfwheel.setPower(-0.15);
                leftbwheel.setPower(-0.15);
                rightfwheel.setPower(0.15);
                rightbwheel.setPower(0.15); }
                    else
                {leftfwheel.setPower(0);
                leftbwheel.setPower(0);
                rightfwheel.setPower(0);
                rightbwheel.setPower(0); } */
                
               
                
               
                 
                
                
                        
            
            

    
    }
    
}
}
