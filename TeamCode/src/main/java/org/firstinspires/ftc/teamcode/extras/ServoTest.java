package org.firstinspires.ftc.teamcode.extras;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(group = "ServoTest")
@Config
public class ServoTest extends LinearOpMode {
    public Servo servo;
    public static double servoPos;
    @Override
    public void runOpMode() throws InterruptedException {
        servo = hardwareMap.get(Servo.class, "servo");
        waitForStart();

        while (opModeIsActive()) {
            if (gamepad1.a){
                servo.setPosition(0);
            }
            if (gamepad1.b){
                servo.setPosition(0.5);
            }
            if (gamepad1.x){
                servo.setPosition(1);
            }
            if (gamepad1.y){
                servo.setPosition(servoPos);
            }
            telemetry.addData("ServoPosition", servo.getPosition());
            telemetry.update();
        }
    }
}
