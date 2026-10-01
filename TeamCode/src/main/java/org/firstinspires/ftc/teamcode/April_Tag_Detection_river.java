package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import java.util.List;

@Autonomous(name = "april_tag_detection_river", group = "Vision")
public class April_Tag_Detection_river extends OpMode {

    private Limelight3A limelight;

    private static final int PIPELINE_30_TO_33 = 7;
    private static final int PIPELINE_34_TO_37 = 8;

    private int currentPipeline = PIPELINE_30_TO_33;

    private boolean cell30to33Detected = false;
    private boolean cell34to37Detected = false;

    private long lastPipelineSwitchTime = 0;

    private static final long PIPELINE_SWITCH_TIME = 500;

    @Override
    public void init() {

        limelight = hardwareMap.get(Limelight3A.class, "limelight");

        limelight.pipelineSwitch(PIPELINE_30_TO_33);

        limelight.start();

        telemetry.addLine("Red HIVE Detection");
        telemetry.update();
    }

    @Override
    public void loop() {

        LLResult result = limelight.getLatestResult();

        if (result != null && result.isValid()) {

            List<LLResultTypes.FiducialResult> tags =
                    result.getFiducialResults();

            boolean redCell30to33 = false;
            boolean redCell34to37 = false;

            for (LLResultTypes.FiducialResult tag : tags) {

                int id = tag.getFiducialId();

                if (id >= 30 && id <= 33) {
                    redCell30to33 = true;
                }

                if (id >= 34 && id <= 37) {
                    redCell34to37 = true;
                }
            }

            if (redCell30to33 && !redCell34to37) {

                cell30to33Detected = true;
                cell34to37Detected = false;
            }

            else if (redCell34to37 && !redCell30to33) {

                cell30to33Detected = false;
                cell34to37Detected = true;
            }

            telemetry.addData("Total Tags", tags.size());
        }

        else {

            telemetry.addLine("No Valid Result");
        }

        if (cell30to33Detected && !cell34to37Detected) {

            telemetry.addLine("34-37 CELL: UP");
            telemetry.addLine("30-33 CELL: DOWN");
        }

        else if (cell34to37Detected && !cell30to33Detected) {

            telemetry.addLine("34-37 CELL: DOWN");
            telemetry.addLine("30-33 CELL: UP");
        }

        else {

            telemetry.addLine("No Red CELL Detected");
        }

        long currentTime = System.currentTimeMillis();

        if (currentTime - lastPipelineSwitchTime >= PIPELINE_SWITCH_TIME) {

            if (currentPipeline == PIPELINE_30_TO_33) {

                currentPipeline = PIPELINE_34_TO_37;

                limelight.pipelineSwitch(PIPELINE_34_TO_37);
            }

            else {

                currentPipeline = PIPELINE_30_TO_33;

                limelight.pipelineSwitch(PIPELINE_30_TO_33);
            }

            lastPipelineSwitchTime = currentTime;
        }

        telemetry.update();
    }

    @Override
    public void stop() {

        limelight.stop();
    }
}