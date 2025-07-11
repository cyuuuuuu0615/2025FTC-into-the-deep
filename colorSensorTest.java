package org.firstinspires.ftc.teamcode.Game;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import com.qualcomm.robotcore.hardware.ColorSensor;

public class colorSensorTest {
    private ColorSensor colorSensor;
    private double redValue;
    private double greenValue;
    private double blueValue;
    private double alphaValue;

    public void runOpMode()throws InterruptedException {
        initHardware();
        
    }

    private void initHardware() {
        initColorSensor();
    }

    private void initColorSensor() {
        colorSensor = hardwareMap.get(ColorSensor.class,"colorSensor");
    }

    private void getColor(){
        redValue = colorSensor.red();
        greenValue = colorSensor.green();
        blueValue = colorSensor.blue();
        alphaValue = colorSensor.alpha();
    }


}

