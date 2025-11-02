package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp
@Disabled
public class HelloWorld extends OpMode {
    @Override
    public void init() {
        telemetry.addData("Hello", "Aayush");
        telemetry.update();
    }

    @Override
    public void loop() {

    }
}
