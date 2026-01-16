package org.firstinspires.ftc.teamcode.testing;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.gamepad1;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import com.qualcomm.robotcore.hardware.CRServo;

public class ServoTesterTeleop {
    CRServo leftFeeder;
    CRServo rightFeeder;
    double feederNumber;
    public void init() {
        leftFeeder = hardwareMap.get(CRServo.class, "left_feeder");
        rightFeeder = hardwareMap.get(CRServo.class, "right_feeder");
        feederNumber = 0;
    }
    public void loop() {
        if (gamepad1.aWasPressed()) {
            feederNumber += 1;
            feederNumber = feederNumber % 1;
        }
        leftFeeder.setPower(feederNumber);
        rightFeeder.setPower(feederNumber);
    }
}
