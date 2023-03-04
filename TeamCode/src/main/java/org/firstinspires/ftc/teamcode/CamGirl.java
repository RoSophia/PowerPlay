package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;
import static org.firstinspires.ftc.teamcode.RobotFuncs.batteryVoltageSensor;
import static org.firstinspires.ftc.teamcode.RobotFuncs.dashboard;
import static org.firstinspires.ftc.teamcode.RobotFuncs.hardwareMap;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.ArrayList;

public class CamGirl implements Runnable{
    public int LAST_ID = 0;
    public boolean shouldClose = false;
    OpenCvCamera webcam;

    public CamGirl(OpenCvCamera cm) {
        this.webcam = cm;
    }

    boolean OPENED = false;
    @Override
    public void run() {
        AprilTagDetectionPipeline pipeline;
        double TAGSIZE = 4.5 / 100;
        double FX = 878.272;
        double FY = 878.272;
        double CX = 320;
        double CY = 240;

        TelemetryPacket p;

        p = new TelemetryPacket();
        p.addLine("MKP");
        dashboard.sendTelemetryPacket(p);

        pipeline = new AprilTagDetectionPipeline(TAGSIZE, FX, FY, CX, CY);
        p = new TelemetryPacket();
        p.addLine("SETP");
        dashboard.sendTelemetryPacket(p);
        webcam.setPipeline(pipeline);

        p = new TelemetryPacket();
        p.addLine("OPEN");
        dashboard.sendTelemetryPacket(p);
        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                webcam.startStreaming(640, 480, OpenCvCameraRotation.SIDEWAYS_LEFT);
                dashboard.startCameraStream(webcam, 15);
                OPENED = true;
                TelemetryPacket pack = new TelemetryPacket();
                pack.addLine("OPENED");
                dashboard.sendTelemetryPacket(pack);
            }

            @Override
            public void onError(int errorCode) {
                TelemetryPacket pack = new TelemetryPacket();
                pack.put("Erro", errorCode);
                dashboard.sendTelemetryPacket(pack);
            }
        });

        while (shouldClose) {
            if (OPENED) {
                if (LAST_ID == 0) {
                    TelemetryPacket packet = new TelemetryPacket();
                    packet.put("Waiting on cam open", 0);
                    dashboard.sendTelemetryPacket(packet);
                }
                ArrayList<AprilTagDetection> cd = pipeline.getLatestDetections();
                if (cd.size() > 0) {
                    LAST_ID = cd.get(0).id;
                    TelemetryPacket packet = new TelemetryPacket();
                    packet.put("LID", LAST_ID);
                    dashboard.sendTelemetryPacket(packet);
                }
            }
            try {
                Thread.sleep(5);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
        }

        p = new TelemetryPacket();
        p.addLine("Close");
        dashboard.sendTelemetryPacket(p);

        if (shouldClose && OPENED) {
            webcam.closeCameraDeviceAsync(() -> {
            });
        }
    }

}
