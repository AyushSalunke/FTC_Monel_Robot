package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeBlueDark;
import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeBlueLight;
import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeRedDark;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.SampleMecanumDrive;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(650);

        RoadRunnerBotEntity myFirstBot = new DefaultBotBuilder(meepMeep)
                .setColorScheme(new ColorSchemeRedDark())
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 11.71)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(15, -64, -Math.PI))
                                //backdrop
                                .lineToConstantHeading(new Vector2d(30 , -36))
                                .splineToConstantHeading(new Vector2d(46.5,-30), 0)
                                .waitSeconds(1)
                                .setReversed(false)

                                //pixel intake // round 1
                                .splineToConstantHeading(new Vector2d(18,-7),-Math.PI)
                                .splineToConstantHeading(new Vector2d(-29,-12),-Math.PI)
                                .splineToConstantHeading(new Vector2d(-51,-12),-Math.PI)
                                .waitSeconds(1)
                                .setReversed(true)
                                //backdrop
                                .splineToConstantHeading(new Vector2d(-34,-12),0)
                                .splineToConstantHeading(new Vector2d(18,-7),0)
                                .splineToConstantHeading(new Vector2d(47,-38),0)
                                .lineToConstantHeading(new Vector2d(47.3, -35))
                                .waitSeconds(1)
                                .setReversed(false)

                                //pixel intake // round 2
                                .splineToConstantHeading(new Vector2d(18,-5),-Math.PI)
                                .splineToConstantHeading(new Vector2d(-29,-12),-Math.PI)
                                .splineToConstantHeading(new Vector2d(-31, -12), -Math.PI)
                                .lineToConstantHeading(new Vector2d(-51.7,-12))
                                .waitSeconds(1)
                                .setReversed(true)
                                //backdrop
                                .splineToConstantHeading(new Vector2d(-34,-12),0)
                                .splineToConstantHeading(new Vector2d(18,-7),0)
                                .splineToConstantHeading(new Vector2d(47,-33),0)
                                .lineToConstantHeading(new Vector2d(47.3, -30))
                                .waitSeconds(1)
                                .setReversed(false)

                                //pixel intake // round 3
                                .splineToConstantHeading(new Vector2d(18,-5),-Math.PI)
                                .splineToConstantHeading(new Vector2d(-29,-12),-Math.PI)
                                .splineToConstantHeading(new Vector2d(-31,-12),-Math.PI)
                                .lineToConstantHeading(new Vector2d(-51.7,-12))
                                .waitSeconds(1)
                                .setReversed(true)
                                //backdrop
                                .splineToConstantHeading(new Vector2d(-34,-12),0)
                                .splineToConstantHeading(new Vector2d(18,-7),0)
                                .splineToConstantHeading(new Vector2d(47,-33),0)
                                .lineToConstantHeading(new Vector2d(47.3, -30))
                                .waitSeconds(1)
                                .setReversed(false)
                                .build()
                );

        RoadRunnerBotEntity mySecondBot = new DefaultBotBuilder(meepMeep)
                .setColorScheme(new ColorSchemeBlueDark())
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(253), Math.toRadians(253), 11.47)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(15, -64, -Math.PI))
                                //backdrop
                                .lineToConstantHeading(new Vector2d(30 , -36))
                                .setConstraints(SampleMecanumDrive.getVelocityConstraint(35, Math.toRadians(136.52544), 11.47), SampleMecanumDrive.getAccelerationConstraint(35))
                                .splineToConstantHeading(new Vector2d(47.5,-30), 0)
                                .resetConstraints()
                                .setReversed(false)

                                //pixel intake // round 1
                                .splineToConstantHeading(new Vector2d(18,-10),-Math.PI)
                                .splineToConstantHeading(new Vector2d(-34,-10),-Math.PI)
                                .setConstraints(SampleMecanumDrive.getVelocityConstraint(35, Math.toRadians(136.52544), 11.47), SampleMecanumDrive.getAccelerationConstraint(35))
                                .splineToConstantHeading(new Vector2d(-51.7,-12.5),-Math.PI)
                                .resetConstraints()
                                .setReversed(true)
                                //backdrop
                                .splineToConstantHeading(new Vector2d(-34,-10),0)
                                .splineToConstantHeading(new Vector2d(18,-10),0)
                                .setConstraints(SampleMecanumDrive.getVelocityConstraint(35, Math.toRadians(136.52544), 11.47), SampleMecanumDrive.getAccelerationConstraint(35))
                                .splineToConstantHeading(new Vector2d(47.5,-30),0)
                                .resetConstraints()
                                .setReversed(false)

                                //pixel intake // round 2
                                .splineToConstantHeading(new Vector2d(18,-10),-Math.PI)
                                .splineToConstantHeading(new Vector2d(-34, -10), -Math.PI)
                                .setConstraints(SampleMecanumDrive.getVelocityConstraint(35, Math.toRadians(136.52544), 11.47), SampleMecanumDrive.getAccelerationConstraint(35))
                                .splineToConstantHeading(new Vector2d(-51.7,-12.5), -Math.PI)
                                .resetConstraints()
                                .setReversed(true)
                                //backdrop
                                .splineToConstantHeading(new Vector2d(-34,-10),0)
                                .splineToConstantHeading(new Vector2d(18,-10),0)
                                .setConstraints(SampleMecanumDrive.getVelocityConstraint(35, Math.toRadians(136.52544), 11.47), SampleMecanumDrive.getAccelerationConstraint(35))
                                .splineToConstantHeading(new Vector2d(47.5,-30),0)
                                .resetConstraints()
                                .setReversed(false)

                                //pixel intake // round 3
                                .splineToConstantHeading(new Vector2d(18,-8),-Math.PI)
                                .splineToConstantHeading(new Vector2d(-31,-8),-Math.PI)
                                .setConstraints(SampleMecanumDrive.getVelocityConstraint(35, Math.toRadians(136.52544), 11.47), SampleMecanumDrive.getAccelerationConstraint(35))
                                .splineToConstantHeading(new Vector2d(-51.7,-24.6),-Math.PI)
                                .resetConstraints()
                                .setReversed(true)
                                //backdrop
                                .splineToConstantHeading(new Vector2d(-31,-10),0)
                                .splineToConstantHeading(new Vector2d(18,-10),0)
                                .setConstraints(SampleMecanumDrive.getVelocityConstraint(35, Math.toRadians(136.52544), 11.47), SampleMecanumDrive.getAccelerationConstraint(35))
                                .splineToConstantHeading(new Vector2d(47.5, -30), 0)
                                .resetConstraints()
                                .setReversed(false)

                                //pixel intake // round 4
                                .splineToConstantHeading(new Vector2d(18,-8),-Math.PI)
                                .splineToConstantHeading(new Vector2d(-31,-8),-Math.PI)
                                .setConstraints(SampleMecanumDrive.getVelocityConstraint(35, Math.toRadians(136.52544), 11.47), SampleMecanumDrive.getAccelerationConstraint(35))
                                .splineToConstantHeading(new Vector2d(-51.7,-24.6),-Math.PI)
                                .resetConstraints()
                                .setReversed(true)
                                //backdrop
                                .splineToConstantHeading(new Vector2d(-31,-10),0)
                                .splineToConstantHeading(new Vector2d(18,-10),0)
                                .setConstraints(SampleMecanumDrive.getVelocityConstraint(35, Math.toRadians(136.52544), 11.47), SampleMecanumDrive.getAccelerationConstraint(35))
                                .splineToConstantHeading(new Vector2d(47.5, -30), 0)
                                .resetConstraints()
                                .setReversed(false)
                                .build()
                );


        RoadRunnerBotEntity myThirdBot = new DefaultBotBuilder(meepMeep)
                .setColorScheme(new ColorSchemeBlueLight())
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(253), Math.toRadians(253), 11.47)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(15, -64, -Math.PI))
                                //backdrop
                                .lineToConstantHeading(new Vector2d(30 , -36))
                                .setConstraints(SampleMecanumDrive.getVelocityConstraint(35, Math.toRadians(136.52544), 11.47), SampleMecanumDrive.getAccelerationConstraint(35))
                                .splineToConstantHeading(new Vector2d(47.5,-30), 0)
                                .resetConstraints()
                                .setReversed(false)

                                //pixel intake // round 1
                                .splineToConstantHeading(new Vector2d(18,-12.5),-Math.PI)
                                .splineToConstantHeading(new Vector2d(-34,-12.5),-Math.PI)
                                .setConstraints(SampleMecanumDrive.getVelocityConstraint(35, Math.toRadians(136.52544), 11.47), SampleMecanumDrive.getAccelerationConstraint(35))
                                .splineToConstantHeading(new Vector2d(-51.7,-12.5),-Math.PI)
                                .resetConstraints()
                                .setReversed(true)
                                //backdrop
                                .splineToConstantHeading(new Vector2d(-34,-12.5),0)
                                .splineToConstantHeading(new Vector2d(18,-12.5),0)
                                .setConstraints(SampleMecanumDrive.getVelocityConstraint(35, Math.toRadians(136.52544), 11.47), SampleMecanumDrive.getAccelerationConstraint(35))
                                .splineToConstantHeading(new Vector2d(47.5,-30),0)
                                .resetConstraints()
                                .setReversed(false)

                                //pixel intake // round 2
                                .splineToConstantHeading(new Vector2d(18,-12.5),-Math.PI)
                                .splineToConstantHeading(new Vector2d(-34, -12.5), -Math.PI)
                                .setConstraints(SampleMecanumDrive.getVelocityConstraint(35, Math.toRadians(136.52544), 11.47), SampleMecanumDrive.getAccelerationConstraint(35))
                                .splineToConstantHeading(new Vector2d(-51.7,-12.5), -Math.PI)
                                .resetConstraints()
                                .setReversed(true)
                                //backdrop
                                .splineToConstantHeading(new Vector2d(-34,-12.5),0)
                                .splineToConstantHeading(new Vector2d(18,-12.5),0)
                                .setConstraints(SampleMecanumDrive.getVelocityConstraint(35, Math.toRadians(136.52544), 11.47), SampleMecanumDrive.getAccelerationConstraint(35))
                                .splineToConstantHeading(new Vector2d(47.5,-30),0)
                                .resetConstraints()
                                .setReversed(false)

                                //pixel intake // round 3
                                .splineToConstantHeading(new Vector2d(18,-10),-Math.PI)
                                .splineToConstantHeading(new Vector2d(-31,-10),-Math.PI)
                                .setConstraints(SampleMecanumDrive.getVelocityConstraint(35, Math.toRadians(136.52544), 11.47), SampleMecanumDrive.getAccelerationConstraint(35))
                                .splineToConstantHeading(new Vector2d(-51.7,-24.6),-Math.PI)
                                .resetConstraints()
                                .setReversed(true)
                                //backdrop
                                .splineToConstantHeading(new Vector2d(-31,-12.5),0)
                                .splineToConstantHeading(new Vector2d(18,-12.5),0)
                                .setConstraints(SampleMecanumDrive.getVelocityConstraint(35, Math.toRadians(136.52544), 11.47), SampleMecanumDrive.getAccelerationConstraint(35))
                                .splineToConstantHeading(new Vector2d(47.5, -30), 0)
                                .resetConstraints()
                                .setReversed(false)

                                //pixel intake // round 4
                                .splineToConstantHeading(new Vector2d(18,-10),-Math.PI)
                                .splineToConstantHeading(new Vector2d(-31,-10),-Math.PI)
                                .setConstraints(SampleMecanumDrive.getVelocityConstraint(35, Math.toRadians(136.52544), 11.47), SampleMecanumDrive.getAccelerationConstraint(35))
                                .splineToConstantHeading(new Vector2d(-51.7,-24.6),-Math.PI)
                                .resetConstraints()
                                .setReversed(true)
                                //backdrop
                                .splineToConstantHeading(new Vector2d(-31,-12.5),0)
                                .splineToConstantHeading(new Vector2d(18,-12.5),0)
                                .setConstraints(SampleMecanumDrive.getVelocityConstraint(35, Math.toRadians(136.52544), 11.47), SampleMecanumDrive.getAccelerationConstraint(35))
                                .splineToConstantHeading(new Vector2d(47.5, -30), 0)
                                .resetConstraints()
                                .setReversed(false)
                                .build()
                );

        meepMeep.setBackground(MeepMeep.Background.FIELD_CENTERSTAGE_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
//                .addEntity(myFirstBot)
//                .addEntity(mySecondBot)
                .addEntity(myThirdBot)
                .start();
    }
}

