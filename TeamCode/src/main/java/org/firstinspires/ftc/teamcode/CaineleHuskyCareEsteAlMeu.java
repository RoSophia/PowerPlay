package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.HuskyLensLib.EElement;
import org.firstinspires.ftc.teamcode.HuskyLensLib.HuskyLensLib;

import java.util.Vector;

class AlgoByteId {
    public static byte ALGORITHM_OBJECT_TRACKING = 0x01;
    public static byte ALGORITHM_FACE_RECOGNITION = 0x00;
    public static byte ALGORITHM_OBJECT_RECOGNITION = 0x02;
    public static byte ALGORITHM_LINE_TRACKING = 0x03;
    public static byte ALGORITHM_COLOR_RECOGNITION = 0x04;
    public static byte ALGORITHM_TAG_RECOGNITION = 0x05;
    public static byte ALGORITHM_OBJECT_CLASSIFICATION = 0x06;
    public static byte ALGORITHM_QR_CODE_RECOGNTITION = 0x07;
    public static byte ALGORITHM_BARCODE_RECOGNTITION = 0x08;
}

@Config
@TeleOp
public class CaineleHuskyCareEsteAlMeu extends LinearOpMode {
    HuskyLensLib husk;

    public void runOpMode() {
        husk = hardwareMap.get(HuskyLensLib.class, "Husky");
        husk.AddTelemetry(telemetry);
        husk.algorithm(AlgoByteId.ALGORITHM_COLOR_RECOGNITION);

        waitForStart();

        while (opModeIsActive()) {
            Vector<EElement> e = husk.blocks();
            telemetry.addData("Len:", e.size());
            /*for (EElement x : e) {
                telemetry.addData(" \\- ID: ", x.ID);
                telemetry.addData("  - TY: ", x.type);
                telemetry.addData("  - X0: ", x.x0);
                telemetry.addData("  - Y0: ", x.y0);
                telemetry.addData("  - X1: ", x.x1);
                telemetry.addData("  - Y1: ", x.y1);
            }
            telemetry.update();
            sleep(1000);*/
        }
    }
}
