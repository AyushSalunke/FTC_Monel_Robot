package org.firstinspires.ftc.teamcode.auto;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.Arm;
import org.firstinspires.ftc.teamcode.subsystems.Hanger;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Slider;

@Autonomous(group = "RedAuto1_Side2")
@Config
public class RedAutoOne_Side2 extends LinearOpMode {
    SampleMecanumDrive drive = null;
    Slider slider = null;
    Arm arm = null;
    Hanger hanger = null;
    Intake intake = null;
    @Override
    public void runOpMode() throws InterruptedException {
        drive = new SampleMecanumDrive(hardwareMap);
        slider = new Slider(hardwareMap, telemetry);
        arm = new Arm(hardwareMap, telemetry);
        hanger = new Hanger(hardwareMap, telemetry);
        intake = new Intake(hardwareMap, telemetry);

        slider.extendToHome();
        Arm.SetArmPosition(0.15,0.73);
        Intake.SetArmPosition(0.4,0.65);
        Intake.IntakePixel(0.75);
        Arm.DropPixel(0.5);
        Intake.CrankPosition(0.67);

        Pose2d startPose=new Pose2d(14, -62, -Math.PI);
        drive.setPoseEstimate(startPose);

    }
}
