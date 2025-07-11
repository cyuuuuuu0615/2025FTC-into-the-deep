package org.firstinspires.ftc.teamcode.Game;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotor;


import org.firstinspires.ftc.teamcode.MecanumDrive;

@Autonomous(name="First Roaderrunner Auton")
public class FirstRoadRunnerAuto extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Pose2d beginPose = new Pose2d(50 - 25 - 25 - 1, 75 - 25 - 50, Math.PI * 1.5);
        MecanumDrive drive = new MecanumDrive(hardwareMap, beginPose);
        Servo baseclaw = hardwareMap.servo.get("servo0");
        Servo position = hardwareMap.servo.get("servo1");
        //DcMotor bigclaw = hardwareMap.get(DcMotor.class, "motor1");
        //DcMotor smallclaw = hardwareMap.get(DcMotor.class, "motor2");
        //DcMotor frontLeftMotor = hardwareMap.get(DcMotor.class, "motor0");
        //DcMotor backLeftMotor = hardwareMap.get(DcMotor.class, "motor0");
        //DcMotor frontRightMotor = hardwareMap.get(DcMotor.class, "motor0");
        //DcMotor backRightMotor = hardwareMap.get(DcMotor.class, "motor0");
        DcMotor horizmotor = hardwareMap.get(DcMotor.class, "motor0");
        DcMotor vertmotor = hardwareMap.get(DcMotor.class, "motor1");

        waitForStart();

        //Actions.runBlocking(
            // drive.actionBuilder(beginPose)
                //.lineTox()
                //.stopAndAdd()
                //.waitSeconds(t:0)
                //.lineToX()
                //.stopAndAdd()
                //.waitSecond(t:0)
                //.build());
    }

    public class guasample implements Action {

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            long startTime = System.currentTimeMillis();
            boolean start = true;

            while (start) {
                long currentTime = System.currentTimeMillis() - startTime;

                if (currentTime <= 00) {
                    //horizmotor.setPosition();
                }else if (currentTime > 00) {
                    //baseclaw.setPosition(0.3);
                }else{
                    //start = false;
                }
            }


            return false;
        }
    }
    public class jiasample implements Action {

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            long startTime = System.currentTimeMillis();
            boolean start = true;

            while (start) {
                long currentTime = System.currentTimeMillis() - startTime;

                if (currentTime <= 00) {
                    //horizmotor.setPosition();
                }else if (currentTime <= 00) {
                    //baseclaw.setPosition(0.3);
                }else if(currentTime <= 00){
                    //horizmotor.setPosition();
                }else if(currentTime <= 00){
                    //baseclaw.setPosition(0.6);
                }else{
                    //start = false;
                }
            }


            return false;
        }
    }
}
