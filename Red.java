package org.firstinspires.ftc.teamcode.Game;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

 import java.time.Instant;

@TeleOp
public class Red extends LinearOpMode {

    //ColorSensor color;

    public static double handlerange(double x,double a,double b){
        if(x>a){
            return a;
        }else if(x<b){
            return b;
        }else{
            return x;
        }
    }

    long chelunIn= -1;
    long chelunOut= 1;

    boolean get = false;
    boolean input = false;
    boolean output = false;
    long startTime1 = 0;
    long currentTime1 = 0;
    long startTime2 = 0;
    long currentTime2 = 0;


    public void runOpMode(){
        //color = hardwareMap.colorSensor.get("color");
        Servo jiaoduServo = hardwareMap.get(Servo.class, "servo0");
        CRServo chelunServo = hardwareMap.get(CRServo.class, "servo5");
        DcMotor frontLeftMotor = hardwareMap.get(DcMotor.class, "motor1");
        DcMotor backLeftMotor = hardwareMap.get(DcMotor.class, "motor2");
        DcMotor frontRightMotor = hardwareMap.get(DcMotor.class, "motor0");
        DcMotor backRightMotor = hardwareMap.get(DcMotor.class, "motor3");
        DcMotor shengsuobi = hardwareMap.get(DcMotor.class, "motor4");
        DcMotor shengjiangbi = hardwareMap.get(DcMotor.class, "motor5");
        shengsuobi.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shengsuobi.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shengjiangbi.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shengjiangbi.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        shengsuobi.setDirection(DcMotorSimple.Direction.REVERSE);
        shengjiangbi.setDirection(DcMotorSimple.Direction.REVERSE);
        frontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);



        waitForStart();





        int hlp = shengsuobi.getCurrentPosition();
        int vlp = shengjiangbi.getCurrentPosition();

        int h_upper_limit = 2000;
        int h_lower_limit = 0;
        int v_upper_limit = 3000;
        int v_lower_limit = 0;
        double jiaoduServoUP = 0;
        double jiaoduServoPING = 0.37;
        double jiaoduServoDOWN = 0.447;
        long startTime = System.currentTimeMillis();

        //jiaoduServo.setPosition(gamepad2.right_trigger);


      //  guaSampleClaw.setPosition(0.3);
        boolean claw_open = false;
        while(opModeIsActive()){



            double x = gamepad1.left_stick_x;
            double y = -gamepad1.left_stick_y; // Remember, Y stick value is reversed
            double rx = gamepad1.right_stick_x;

            double theta = Math.atan2(y, x);
            double power = Math.hypot(x,y);

            double sin = Math.sin(theta - Math.PI/4);
            double cos = Math.cos(theta - Math.PI/4);
            double max = Math.max(Math.abs(sin), Math.abs(cos));


            double frontLeftPower = power * cos/max + rx;
            double frontRightPower = power * sin/max - rx;
            double backLeftPower = power * sin/max + rx;
            double backRightPower = power * cos/max - rx;

            if ((power + Math.abs(rx)) > 1){
                frontLeftPower   /= power + Math.abs(rx);
                frontRightPower /= power + Math.abs(rx);
                backLeftPower    /= power + Math.abs(rx);
                backRightPower  /= power + Math.abs(rx);
            }



            if(input){
                if(get){
                    currentTime1 = System.currentTimeMillis() - startTime1;
                    jiaoduServo.setPosition(jiaoduServoUP);
                    shengsuobi.setTargetPosition(0);
                    shengsuobi.setPower(-1);
                    shengsuobi.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    hlp = shengsuobi.getCurrentPosition();
                    chelunServo.setPower(chelunIn);
                    if(currentTime1 >2000){
                        get = false;
                    }

                }else{
                    chelunServo.setPower(chelunIn);
                }
            }

            if(output){
                currentTime2 = System.currentTimeMillis() - startTime2;
                if(currentTime2 <1000){
                    jiaoduServo.setPosition(jiaoduServoPING);
                    chelunServo.setPower(chelunOut);
                }else{
                    output = false;
                }

            }

            if(!input && !output){
                chelunServo.setPower(0);
            }

            if(gamepad2.left_bumper){
                input = true;
            }
            if(gamepad2.x){
                get = true;
                startTime1 = System.currentTimeMillis();
            }
            if(gamepad2.right_bumper){
                input = false;
                get = false;
                output = true;
                startTime2 = System.currentTimeMillis();
            }


            if(gamepad2.a){
                jiaoduServo.setPosition(jiaoduServoUP);
            }
//            if(gamepad2.b){
//                jiaoduServo.setPosition(jiaoduServoDOWN);
//            }
            if(gamepad2.b){
                jiaoduServo.setPosition(jiaoduServoPING);
            }
////
//            jiaoduServo.setPosition(gamepad2.left_trigger);


           // if(gamepad1.left_bumper){
         //       chelunServo.setPower(-1);
        //    }

            //if(gamepad1.x){
          //      chelunServo.setPower(0);
         //   }
//
//            if(input){
//                xiSample.setPower(1);
////             //   if(color.blue()<00){
////               //     xiSample.setPower(1);
////               // }else{
////               //     input = false;
////              //  }
//            }else{
//                xiSample.setPower(0);
//            }

     //       if(gamepad1.right_bumper){
        //        chelunServo.setPower(1);
        //    }

            if(!get) {
                if (((gamepad2.left_trigger + gamepad2.right_trigger) == 0)) {
                    shengsuobi.setTargetPosition(hlp);
                    shengsuobi.setPower(1);
                    shengsuobi.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                } else {
                    hlp = shengsuobi.getCurrentPosition();
                    shengsuobi.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    if (shengsuobi.getCurrentPosition() <= h_upper_limit && shengsuobi.getCurrentPosition() >= h_lower_limit) {
                        shengsuobi.setPower(gamepad2.left_trigger - gamepad2.right_trigger);
                    } else if (shengsuobi.getCurrentPosition() >= h_upper_limit) {
                        shengsuobi.setPower(handlerange(gamepad2.left_trigger - gamepad2.right_trigger, 0, -1));
                    } else if (shengsuobi.getCurrentPosition() <= h_lower_limit) {
                        shengsuobi.setPower(handlerange(gamepad2.left_trigger - gamepad2.right_trigger, 1, 0));
                    }
                }
            }



            if(gamepad1.left_trigger + gamepad1.right_trigger == 0){
                shengjiangbi.setTargetPosition(vlp);
                shengjiangbi.setPower(1);
                shengjiangbi.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            }else{
                vlp = shengjiangbi.getCurrentPosition();
                shengjiangbi.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//                shengjiangbi.setPower(-gamepad2.left_stick_y);
                if(vlp <= v_upper_limit && vlp >= v_lower_limit){
                    shengjiangbi.setPower(gamepad1.left_trigger - gamepad1.right_trigger);
                }else if(vlp >= v_upper_limit){
                    shengjiangbi.setPower(handlerange(gamepad1.left_trigger - gamepad1.right_trigger,-1,0));
                } else if (vlp <= v_lower_limit) {
                    shengjiangbi.setPower(handlerange(gamepad1.left_trigger - gamepad1.right_trigger,1,0));
                }
            }
//            if(shengjiangbi.getCurrentPosition()>=v_upper_limit){
//                shengjiangbi.setPower(handlerange(gamepad2.left_stick_y,0,-1));
//            }else{
//                if(shengjiangbi.getCurrentPosition()<=v_lower_limit){
//                    shengjiangbi.setPower(handlerange(gamepad2.left_stick_y,1,0));
//                }else{
//                    shengjiangbi.setPower(handlerange(gamepad2.left_stick_y,1,-1));
//                }
//            }


            //if(gamepad2.left_stick_y == 0){
            //    vertmotor.setTargetPosition(vlp);
            //    vertmotor.setPower(1);
            //    vertmotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            //}else{
            //    vlp = vertmotor.getCurrentPosition();
            //    vertmotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            //    vertmotor.setPower(gamepad2.left_stick_y);
            //}

//            if(vlp>=v_upper_limit){
//                vertmotor.setPower(handlerange(gamepad2.left_stick_y,-1,0));
//            }else{
//                if(vlp<=v_lower_limit){
//                    vertmotor.setPower(handlerange(gamepad2.left_stick_y,0,1));
//                }else{
//                    vertmotor.setPower(handlerange(gamepad2.left_stick_y,-1,1));
//                }
//            }


            frontLeftMotor.setPower(frontLeftPower);
            backLeftMotor.setPower(backLeftPower);
            frontRightMotor.setPower(frontRightPower);
            backRightMotor.setPower(backRightPower);
            //telemetry.addData("red:",color.red());
            //telemetry.addData("green:",color.green());
            //telemetry.addData("blue:",color.blue());
            //telemetry.addData("alpha:",color.alpha());
            //telemetry.addData("gua claw",guaSampleClaw.getPosition());
            //telemetry.addData("xi",xiSample.getPower());
            telemetry.addData("horizontal shengsuobi",shengsuobi.getCurrentPosition());
            telemetry.addData("vertical shengjiangbi",shengjiangbi.getCurrentPosition());
            telemetry.addData("jiaodu servo",jiaoduServo.getPosition());
            telemetry.addData("chelunServo",chelunServo.getPower());
            telemetry.addData("input",input);
            telemetry.addData("get",get);
            telemetry.addData("output",output);


            telemetry.update();
        }









    }
}
