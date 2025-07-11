package org.firstinspires.ftc.teamcode.Game;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import java.time.Instant;

@TeleOp
public class test extends LinearOpMode {

    public static double handlerange(double x,double a,double b){
        if(x>a){
            return a;
        }else if(x<b){
            return b;
        }else{
            return x;
        }
    }



    public void runOpMode(){
        DcMotor horizmotor = hardwareMap.get(DcMotor.class, "motor4");
        Servo jiaoduServo = hardwareMap.get(Servo.class, "servo0");

        DcMotor vertmotor = hardwareMap.get(DcMotor.class, "motor5");
        Servo jiaoduclaw = hardwareMap.get(Servo.class,"servo2");
        CRServo chelunServo = hardwareMap.get(CRServo.class, "servo5");
        horizmotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        horizmotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        vertmotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        vertmotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        waitForStart();
        int hlp = horizmotor.getCurrentPosition();
        int vlp = vertmotor.getCurrentPosition();
        int h_upper_limit = 3855;
        int h_lower_limit = 0;
        int v_upper_limit = 0;
        int v_lower_limit = 0;





        while (opModeIsActive()){
            if(gamepad2.a){
                chelunServo.setPower(1);
            }
            jiaoduServo.setPosition(gamepad1.right_trigger);
            if((gamepad2.left_trigger + gamepad2.right_trigger) == 0){
                horizmotor.setTargetPosition(hlp);
                horizmotor.setPower(1);
                horizmotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            }else{
                hlp = horizmotor.getCurrentPosition();
                horizmotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                horizmotor.setPower(gamepad2.left_trigger - gamepad2.right_trigger);
            }



            //if(hlp>=h_upper_limit){
            //    horizmotor.setPower(handlerange(gamepad2.left_trigger-gamepad2.right_trigger,0,-1));
            //}else{
            //    if(hlp<=h_lower_limit){
            //        horizmotor.setPower(handlerange(gamepad2.left_trigger-gamepad2.right_trigger,1,0));
            //    }else{
            //        horizmotor.setPower(handlerange(gamepad2.left_trigger-gamepad2.right_trigger,1,-1));
            //   }
            // }



            if(gamepad2.left_stick_y == 0){
                vertmotor.setTargetPosition(vlp);
                vertmotor.setPower(1);
                vertmotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            }else{
                vlp = vertmotor.getCurrentPosition();
                vertmotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                vertmotor.setPower(gamepad2.left_stick_y);
            }

            //if(vlp>=v_upper_limit){
            //    vertmotor.setPower(handlerange(gamepad2.left_stick_y,-1,0));
            //}else{
            //    if(vlp<=v_lower_limit){
            //        vertmotor.setPower(handlerange(gamepad2.left_stick_y,0,1));
            //    }else{
            //        vertmotor.setPower(handlerange(gamepad2.left_stick_y,-1,1));
            //    }
            //}





            if(gamepad2.left_bumper){
                jiaoduclaw.setPosition(1);
            }else if(gamepad2.right_bumper){
                jiaoduclaw.setPosition(0.5);
            }

            telemetry.addData("horizontal motor",horizmotor.getCurrentPosition());
            telemetry.addData("vertical motor",vertmotor.getCurrentPosition());
            telemetry.addData("j p",jiaoduServo.getPosition());


            telemetry.update();
        }









    }
}
