package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import org.rowlandhall.meepmeep.MeepMeep;
import org.rowlandhall.meepmeep.roadrunner.DefaultBotBuilder;
import org.rowlandhall.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting {
    public static void main(String[] args) {

        MeepMeep meepMeep = new MeepMeep(800);

        Pose2d startPose = new Pose2d(-63, 12, Math.toRadians(0));

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                .setConstraints(
                        60, 60,
                        Math.toRadians(180), Math.toRadians(180),
                        15
                )
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(startPose)

                                // 1️⃣ First segment: strafe + turn (shooting)
                                .strafeTo(new Vector2d(-50, 12))
                                .turn(Math.toRadians(20))

                                // 2️⃣ Second segment: turn + move to stack
                                .turn(Math.toRadians(-110))
                                .strafeTo(new Vector2d(-43, 12))

                                // 3️⃣ Move upfield
                                .strafeTo(new Vector2d(-43, 64))

                                // Minor adjustment
                                .strafeTo(new Vector2d(-43, 58))

                                // 4️⃣ Final parking path
                                .strafeTo(new Vector2d(-40, 12))
                                .strafeTo(new Vector2d(-50, 12))
                                .turn(Math.toRadians(105))

                                .build()
                );

        meepMeep.addEntity(myBot)
                .setDarkMode(true)
                .setBackground(MeepMeep.Background.FIELD_CENTERSTAGE_JUICE_DARK)
                .start();
    }
}
