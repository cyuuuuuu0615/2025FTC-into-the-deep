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
public class test_motor extends LinearOpMode {

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
    boolean sheng = false;
    boolean jiang = false;
    boolean xiadaozuixia = false;
    boolean reset = false;
    boolean get = false;
    boolean input = false;
    boolean output = false;
    boolean saoSemple = false;
    long startTime1 = 0;
    long currentTime1 = 0;
    long startTime2 = 0;
    long currentTime2 = 0;
    long startTime3 = 0;
    long currentTime3 = 0;
    long startTime4 = 0;
    long currentTime4 = 0;
    long startTime5 = 0;
    long currentTime5 = 0;
    long startTime6 = 0;
    long currentTime6 = 0;
    long startTime7 = 0;
    long currentTime7 = 0;
    long shengsuobiPower = 0;


    public void runOpMode(){
        //color = hardwareMap.colorSensor.get("color");
        Servo jiaoduServo = hardwareMap.get(Servo.class, "servo0");
        Servo saoServo = hardwareMap.get(Servo.class, "servo4");
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
        saoServo.setPosition(0.4);
        while(opModeIsActive()){
            if (((gamepad2.left_trigger + gamepad2.right_trigger) == 0)) {
                shengsuobi.setTargetPosition(hlp);
                shengsuobi.setPower(1);
                shengsuobi.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            } else {
                hlp = shengsuobi.getCurrentPosition();
                shengsuobi.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                if (shengsuobi.getCurrentPosition() <= h_upper_limit && shengsuobi.getCurrentPosition() >= h_lower_limit) {
                    shengsuobi.setPower(handlerange(gamepad2.left_trigger - gamepad2.right_trigger,1,-1));

                } else if (shengsuobi.getCurrentPosition() >= h_upper_limit) {
                    shengsuobi.setPower(handlerange(gamepad2.left_trigger - gamepad2.right_trigger, 0, -1));
                } else if (shengsuobi.getCurrentPosition() <= h_lower_limit) {
                    shengsuobi.setPower(handlerange(gamepad2.left_trigger - gamepad2.right_trigger, 1, 0));
                }
            }
            //frontLeftMotor.setPower(frontLeftPower);
            //backLeftMotor.setPower(backLeftPower);
            //frontRightMotor.setPower(frontRightPower);
            //backRightMotor.setPower(backRightPower);
            //telemetry.addData("red:",color.red());
            //telemetry.addData("green:",color.green());
            //telemetry.addData("blue:",color.blue());
            //telemetry.addData("alpha:",color.alpha());
            //telemetry.addData("gua claw",guaSampleClaw.getPosition());
            //telemetry.addData("xi",xiSample.getPower());
            telemetry.addData("horizontal shengsuobi",shengsuobi.getCurrentPosition());
//            telemetry.addData("vertical shengjiangbi",shengjiangbi.getCurrentPosition());
//            telemetry.addData("jiaodu servo",jiaoduServo.getPosition());
//            telemetry.addData("chelun servo",chelunServo.getPower());
//            telemetry.addData("input",input);
//            telemetry.addData("get",get);
//            telemetry.addData("output",output);
//            telemetry.addData("sao servo",saoServo.getPosition());
//            telemetry.addData("saoSample",saoSemple);

            telemetry.update();
        }









    }
}
