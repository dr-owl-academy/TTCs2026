
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathBuilder;
import com.pedropathing.paths.PathChain;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous(name = "Auton Blue Lower")
public class Auton_Blue_Lower extends OpMode {

    private Follower follower;
    private Limelight3A limelight;

    private static final int TILT_TOWARD_BLUE = 0;
    private static final int TILT_AWAY_BLUE = 1;

    private int state = 0;

    private int secondAI = -1;
    private int thirdAI = -1;
    private int fourthAI = -1;

    private boolean pathStarted = false;
    private boolean turnStarted = false;

    private PathChain currentPath;

    @Override
    public void init() {

        follower = Constants.createFollower(hardwareMap);

        limelight = hardwareMap.get(
                Limelight3A.class, "limelight"
        );

        limelight.pipelineSwitch(0);
        limelight.start();

        follower.setStartingPose(
                new Pose(117, 8, Math.toRadians(90))
        );
    }

    @Override
    public void start() {
        state = 0;
    }

    @Override
    public void loop() {

        follower.update();

        switch (state) {

            // INITIAL AI READ
            case 0:

                int initialAI = readClassifier();

                if (initialAI == TILT_AWAY_BLUE) {
                    state = 1;
                }

                break;

            // INITIAL PATH
            case 1:

                runPath(
                        new Pose(117, 8, Math.toRadians(90)),
                        new Pose(94, 13, Math.toRadians(270)),
                        new Pose(85, 29, Math.toRadians(270))
                );

                if (pathFinished()) {
                    state = 2;
                }

                break;

            // SECOND AI READ POSITION
            case 2:

                runPath(
                        new Pose(85, 29, Math.toRadians(270)),
                        new Pose(94, 19, Math.toRadians(90))
                );

                if (pathFinished()) {
                    state = 3;
                }

                break;

            // SECOND AI READ
            case 3:

                secondAI = readClassifier();

                if (secondAI == TILT_AWAY_BLUE) {
                    state = 4;
                }
                else if (secondAI == TILT_TOWARD_BLUE) {
                    state = 10;
                }

                break;

            // BRANCH 1
            case 4:

                runPath(
                        new Pose(94, 19, Math.toRadians(90)),
                        new Pose(137, 34, Math.toRadians(0))
                );

                if (pathFinished()) {
                    state = 5;
                }

                break;

            // TURN IN PLACE TO 125 DEGREES
            case 5:

                if (!turnStarted) {

                    follower.holdPoint(
                            new Pose(137, 34, Math.toRadians(125))
                    );

                    turnStarted = true;
                }

                double headingError = Math.atan2(
                        Math.sin(Math.toRadians(125)
                                - follower.getPose().getHeading()),
                        Math.cos(Math.toRadians(125)
                                - follower.getPose().getHeading())
                );

                if (Math.abs(headingError) < Math.toRadians(2)) {

                    turnStarted = false;
                    state = 6;
                }

                break;

            // THIRD AI READ
            case 6:

                thirdAI = readClassifier();

                if (thirdAI == TILT_AWAY_BLUE) {
                    state = 7;
                }
                else if (thirdAI == TILT_TOWARD_BLUE) {
                    state = 8;
                }

                break;

            // BRANCH 1-1
            case 7:

                runPath(
                        new Pose(137, 34, Math.toRadians(125)),
                        new Pose(84, 19, Math.toRadians(90))
                );

                if (pathFinished()) {
                    state = 10;
                }

                break;

            // BRANCH 1-2
            case 8:

                state = 10;

                break;

            // SHARED PATH
            case 10:

                runPath(
                        follower.getPose(),
                        new Pose(107, 41, Math.toRadians(0)),
                        new Pose(107, 101, Math.toRadians(0)),
                        new Pose(85, 114, Math.toRadians(-90)),
                        new Pose(128, 94, Math.toRadians(0)),
                        new Pose(120, 94, Math.toRadians(170))
                );

                if (pathFinished()) {
                    state = 11;
                }

                break;

            // FOURTH AI READ
            case 11:

                fourthAI = readClassifier();

                if (fourthAI == TILT_AWAY_BLUE) {
                    state = 12;
                }
                else if (fourthAI == TILT_TOWARD_BLUE) {
                    state = 13;
                }

                break;

            // TILT AWAY: BLUE UPPER SHOOTING POSITION
            case 12:

                runPath(
                        follower.getPose(),
                        new Pose(85, 114, Math.toRadians(-90))
                );

                if (pathFinished()) {
                    state = 14;
                }

                break;

            // TILT TOWARD: REVERSE PATH 1
            case 13:

                runPath(
                        follower.getPose(),
                        new Pose(107, 101, Math.toRadians(0)),
                        new Pose(107, 41, Math.toRadians(0)),
                        new Pose(85, 29, Math.toRadians(90))
                );

                if (pathFinished()) {
                    state = 14;
                }

                break;

            // FINISHED
            case 14:

                telemetry.addLine("Autonomous finished");

                break;
        }

        telemetry.addData("State", state);
        telemetry.addData("Second AI", secondAI);
        telemetry.addData("Third AI", thirdAI);
        telemetry.addData("Fourth AI", fourthAI);

        telemetry.addData(
                "Robot Heading",
                Math.toDegrees(follower.getPose().getHeading())
        );

        telemetry.update();
    }

    // READ LIMELIGHT CLASSIFIER
    private int readClassifier() {

        LLResult result = limelight.getLatestResult();

        if (result == null || !result.isValid()) {
            return -1;
        }

        if (result.getClassifierResults().isEmpty()) {
            return -1;
        }

        LLResultTypes.ClassifierResult best =
                result.getClassifierResults().get(0);

        String className = best.getClassName();

        if (className.equals("1")) {
            return TILT_AWAY_BLUE;
        }

        if (className.equals("0")) {
            return TILT_TOWARD_BLUE;
        }

        return -1;
    }

    // BUILD AND FOLLOW PATH
    private void runPath(Pose... poses) {

        if (!pathStarted) {

            PathBuilder builder = follower.pathBuilder();

            for (int i = 0; i < poses.length - 1; i++) {

                builder.addPath(
                        new BezierLine(poses[i], poses[i + 1])
                );

                builder.setLinearHeadingInterpolation(
                        poses[i].getHeading(),
                        poses[i + 1].getHeading()
                );
            }

            currentPath = builder.build();

            follower.followPath(currentPath);

            pathStarted = true;
        }
    }

    // CHECK IF PATH HAS FINISHED
    private boolean pathFinished() {

        if (pathStarted && !follower.isBusy()) {

            pathStarted = false;

            return true;
        }

        return false;
    }
}
