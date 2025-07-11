package org.firstinspires.ftc.teamcode.Game;
import android.app.Activity;
import android.graphics.Color;
import android.view.View;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.Trajectory;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;

// Non-RR imports
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotor.RunMode;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import org.firstinspires.ftc.teamcode.MecanumDrive;

import com.acmerobotics.roadrunner.ParallelAction;;

@Config
@Autonomous
public class auto extends LinearOpMode {
    double jiaoduservoUp = 0;
    double jiaoduservoDown = 0.37;
    double jiaoduservoPing = 0.3;


    @Override
    public void runOpMode() throws InterruptedException {
        Pose2d beginPose = new Pose2d(0, 0, 0);
        Servo jiaoduServo = hardwareMap.get(Servo.class, "servo0");
        CRServo chelunServo = hardwareMap.get(CRServo.class, "servo5");


        MecanumDrive drive = new MecanumDrive(hardwareMap, beginPose);

        waitForStart();
//        jiaoduServo.setPosition(jiaoduservoUp);

        Actions.runBlocking(new SequentialAction(drive.actionBuilder(beginPose)
                        //strafeTo
                        //.splineToLinearHeading()

//                .stopAndAdd(new ParallelAction(drive.actionBuilder(beginPose)
//                        .strafeTo(new Vector2d(-26, 0))
//                        .stopAndAdd(new shengjiangguaSample())
//                        .build()
//                ))
                // ------------------------------------------------------
                        .stopAndAdd(new shengguaSample())
                        .strafeTo(new Vector2d(-30, 0))

                        .stopAndAdd(new jiangguaSample())
// **               .strafeTo(new Vector2d(-27,0))
                               .strafeTo(new Vector2d(-20, 0))

                        .stopAndAdd(new bifangdaozuixia())
                        .splineToLinearHeading(new Pose2d(-5,20,Math.PI),0)
                .strafeTo(new Vector2d(2,20))
                        .stopAndAdd(new naSample())
                .splineToLinearHeading(new Pose2d(-30,5,0),Math.PI)
                        .stopAndAdd(new jiangguaSample())
// **                       .strafeTo(new Vector2d(-27,5))
                        .strafeTo(new Vector2d(-20,5))

                .stopAndAdd(new bifangdaozuixia())



//-------------------------------------------------------------------
                        .splineToLinearHeading(new Pose2d(-28, 15, Math.PI*0.6), 0)
                .stopAndAdd(new xiSample())
                .splineToLinearHeading(new Pose2d(-28,25,Math.PI*0.2),Math.PI*0.6)
//                .turnTo(Math.PI*0.2)
                        .stopAndAdd(new tuSample())
                //18s
                //-----------------------------------------------------------
                .stopAndAdd(new shouhuishengsuobi())
                     .splineToLinearHeading(new Pose2d(-5,20,Math.PI),Math.PI*0.3)
                .strafeTo(new Vector2d(2,20))
                        .stopAndAdd(new naSample())
                .splineToLinearHeading(new Pose2d(-30,10,0),Math.PI)
                        .stopAndAdd(new jiangguaSample())
//        **                .strafeTo(new Vector2d(-28,10))
                        .strafeTo(new Vector2d(-20,10))

                .stopAndAdd(new bifangdaozuixia())
                        //---------------------------------------------------------------------------------------
//                .splineToLinearHeading(new Pose2d(-19,23,Math.PI*0.64),Math.PI*0.26)
//                .stopAndAdd(new xidierkuaiSample())
//                .turnTo(Math.PI*0.3)
//                .stopAndAdd(new tuSample())

                //---------------------------------------------------------------------
//                .splineToLinearHeading(new Pose2d(-19,32,Math.PI*.7),Math.PI*.4)
//                .stopAndAdd(new xiSample())
//                        .turnTo(Math.PI*.5)
//                .stopAndAdd(new tuSample())

                //--------------------------------------------------------------------------
//
//
//
//                        .splineToLinearHeading(new Pose2d(-5,20,Math.PI),0)
//                        .strafeTo(new Vector2d(5,20))
//                        .stopAndAdd(new naSample())
//                        .splineToLinearHeading(new Pose2d(-30,15,0),Math.PI)
//                        .stopAndAdd(new jiangguaSample())
//                        .strafeTo(new Vector2d(-20, 5))
//                        .stopAndAdd(new bifangdaozuixia())
                        //---------------------------------------------------------------
                .strafeTo(new Vector2d(0,30))
                        .build())


        );
    }

    public class xiSample implements Action {
        CRServo chelunServo = hardwareMap.get(CRServo.class, "servo5");
        DcMotor shengsuobi = hardwareMap.get(DcMotor.class, "motor4");
        Servo jiaoduServo = hardwareMap.get(Servo.class, "servo0");


        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            shengsuobi.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            shengsuobi.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            shengsuobi.setDirection(DcMotorSimple.Direction.REVERSE);

            long startTime = System.currentTimeMillis();
            while (true) {
                long currentTime = System.currentTimeMillis() - startTime;
//                chelunServo.setPower(-1);
//                shengsuobi.setTargetPosition(2100);
//                shengsuobi.setPower(1);
//                shengsuobi.setMode(RunMode.RUN_TO_POSITION);
//                if (currentTime > 5000) {
//                    return false;
//                }
                chelunServo.setPower(-1);
                shengsuobi.setTargetPosition(2100);
                shengsuobi.setPower(1);
                shengsuobi.setMode(RunMode.RUN_TO_POSITION);
                jiaoduServo.setPosition(jiaoduservoDown);
//                if((currentTime > 1600)&&(currentTime < 2000)){
//                    jiaoduServo.setPosition(jiaoduservoDown);
//                }else{
//                    jiaoduServo.setPosition(jiaoduservoPing);
//                }
                //-------------------------------------------------
//                if(currentTime < 2000){
//                    jiaoduServo.setPosition(jiaoduservoDown);
//                }else if(currentTime<2100){
//                    jiaoduServo.setPosition(jiaoduservoPing);
//
//                }else {
//                    return false;
//                }
                if(currentTime > 1800){
                    jiaoduServo.setPosition(jiaoduservoPing);
                    return false;
                }

                telemetry.addData("sengjiangbi", shengsuobi.getCurrentPosition());
                telemetry.addData("chelunServo", chelunServo.getPower());
                telemetry.addData("jiaoduServo", jiaoduServo.getPosition());
                telemetry.update();

            }
        }

    }

    public class shouhuishengsuobi implements Action {

        DcMotor shengsuobi = hardwareMap.get(DcMotor.class, "motor4");


        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {

            long startTime = System.currentTimeMillis();
            while (true) {
                long currentTime = System.currentTimeMillis() - startTime;

                shengsuobi.setTargetPosition(0);
                shengsuobi.setPower(-1);
                shengsuobi.setMode(RunMode.RUN_TO_POSITION);

//                if(currentTime > 1000){
//                    return false;
//                }


                telemetry.addData("sengjiangbi", shengsuobi.getCurrentPosition());
                telemetry.update();
                return false;

            }
        }

    }




    public class tuSample implements Action {
        CRServo chelunServo = hardwareMap.get(CRServo.class, "servo5");
        DcMotor shengsuobi = hardwareMap.get(DcMotor.class, "motor4");
        Servo jiaoduServo = hardwareMap.get(Servo.class, "servo0");


        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {


            long startTime = System.currentTimeMillis();
            while (true) {
                long currentTime = System.currentTimeMillis() - startTime;
                jiaoduServo.setPosition(jiaoduservoPing);
                chelunServo.setPower(1);

                if(currentTime > 600){
                    return false;
                }

//                if(currentTime < 00){
//                    shengsuobi.setTargetPosition(2100);
//                    shengsuobi.setPower(1);
//                    shengsuobi.setMode(RunMode.RUN_TO_POSITION);
//                } else if (currentTime < 00) {
//                    chelunServo.setPower(1);
//                }else {
//                    return false;
//                }
//                if((currentTime > 00)&&(currentTime < 00)){
//                    jiaoduServo.setPosition(jiaoduservoDown);
//
//                }else {
//                    jiaoduServo.setPosition(jiaoduservoUp);
//
//                }
                telemetry.addData("sengjiangbi", shengsuobi.getCurrentPosition());
                telemetry.addData("chelunServo", chelunServo.getPower());
                telemetry.addData("jiaoduServo", jiaoduServo.getPosition());
                telemetry.update();

            }
        }

    }

    public class shengguaSample implements Action {
        DcMotor shengjiangbi = hardwareMap.get(DcMotor.class, "motor5");

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            shengjiangbi.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            shengjiangbi.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            shengjiangbi.setDirection(DcMotorSimple.Direction.REVERSE);


            long startTime = System.currentTimeMillis();
            while (true) {
                long currentTime = System.currentTimeMillis() - startTime;

                shengjiangbi.setTargetPosition(2034);
                shengjiangbi.setPower(1);
                shengjiangbi.setMode(RunMode.RUN_TO_POSITION);


                telemetry.addData("shengjiangbi", shengjiangbi.getCurrentPosition());
                telemetry.update();
                return false;

            }
        }
    }

    public class naSample implements Action {
        DcMotor shengjiangbi = hardwareMap.get(DcMotor.class, "motor5");

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            shengjiangbi.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            shengjiangbi.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            shengjiangbi.setDirection(DcMotorSimple.Direction.REVERSE);


            long startTime = System.currentTimeMillis();
            while (true) {
                long currentTime = System.currentTimeMillis() - startTime;

                shengjiangbi.setTargetPosition(2034);
                shengjiangbi.setPower(1);
                shengjiangbi.setMode(RunMode.RUN_TO_POSITION);


                telemetry.addData("shengjiangbi", shengjiangbi.getCurrentPosition());
                telemetry.update();
                if (currentTime > 500) {
                    return false;
                }


            }
        }
    }

    public class xiangshangsongkaiSample implements Action {
        DcMotor shengjiangbi = hardwareMap.get(DcMotor.class, "motor5");

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            shengjiangbi.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            shengjiangbi.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            shengjiangbi.setDirection(DcMotorSimple.Direction.REVERSE);


            long startTime = System.currentTimeMillis();
            while (true) {
                long currentTime = System.currentTimeMillis() - startTime;

                shengjiangbi.setTargetPosition(2034-1000);
                shengjiangbi.setPower(1);
                shengjiangbi.setMode(RunMode.RUN_TO_POSITION);


                telemetry.addData("shengjiangbi", shengjiangbi.getCurrentPosition());
                telemetry.update();
                if (currentTime > 1000) {
                    return false;
                }


            }
        }
    }

    public class jiangguaSample implements Action {
        DcMotor shengjiangbi = hardwareMap.get(DcMotor.class, "motor5");

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            shengjiangbi.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            shengjiangbi.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            shengjiangbi.setDirection(DcMotorSimple.Direction.REVERSE);

            long startTime = System.currentTimeMillis();
            while (true) {
                long currentTime = System.currentTimeMillis() - startTime;

//                if(currentTime<1000){
//                    shengjiangbi.setTargetPosition(1300 - 2034);
//                    shengjiangbi.setPower(-1);
//                    shengjiangbi.setMode(RunMode.RUN_TO_POSITION);
//                }else if(currentTime<1500){
//                    shengjiangbi.setTargetPosition(1648 - 2034);
//                    shengjiangbi.setPower(1);
//                    shengjiangbi.setMode(RunMode.RUN_TO_POSITION);

//                }else if(currentTime >1500){
//                    return false;
//                }

                shengjiangbi.setTargetPosition(900 - 2034);
                    shengjiangbi.setPower(-1);
                    shengjiangbi.setMode(RunMode.RUN_TO_POSITION);

                telemetry.addData("shengjiangbi", shengjiangbi.getCurrentPosition());
                telemetry.update();


                if (currentTime > 1000) {
                    return false;
                }

            }
        }
    }

    public class bifangdaozuixia implements Action {
        DcMotor shengjiangbi = hardwareMap.get(DcMotor.class, "motor5");

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            shengjiangbi.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            shengjiangbi.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            shengjiangbi.setDirection(DcMotorSimple.Direction.REVERSE);

            long startTime = System.currentTimeMillis();
            while (true) {
                long currentTime = System.currentTimeMillis() - startTime;

                shengjiangbi.setTargetPosition(0 - 900);
                shengjiangbi.setPower(-1);
                shengjiangbi.setMode(RunMode.RUN_TO_POSITION);


                telemetry.addData("shengjiangbi", shengjiangbi.getCurrentPosition());
                telemetry.update();

                return false;
//


            }
        }


    }
}