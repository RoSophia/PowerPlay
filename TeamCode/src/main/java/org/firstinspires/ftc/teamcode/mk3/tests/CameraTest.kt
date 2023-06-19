package org.firstinspires.ftc.teamcode.mk3.tests

import com.acmerobotics.dashboard.config.Config
import com.acmerobotics.dashboard.telemetry.TelemetryPacket
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.FocusControl
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.PtzControl.PanTiltHolder
import org.firstinspires.ftc.teamcode.mk3.camera.CamGirl
import org.firstinspires.ftc.teamcode.mk3.camera.ConePipeline
import org.openftc.easyopencv.OpenCvCameraRotation
import java.util.concurrent.TimeUnit

@Config
object CamTest {
    @JvmField
    var GAIN: Int = 100

    @JvmField
    var EXPOSURE: Int = 10
}

@TeleOp
class CameraTest : LinearOpMode() {
    override fun runOpMode() {
        val conePipeline = ConePipeline()
        val coneGirl = CamGirl(this, "coneGirl", OpenCvCameraRotation.UPSIDE_DOWN, 640, 480, conePipeline, streaming = true, waitForOpen = true)

        var LG = CamTest.GAIN
        var LE = CamTest.EXPOSURE

        coneGirl.camera.gainControl.gain = CamTest.GAIN

        waitForStart();

        while (opModeIsActive()) {
            if (LG != CamTest.GAIN) {
                coneGirl.camera.gainControl.gain = CamTest.GAIN
                LG = CamTest.GAIN
            }

            if (LE != CamTest.EXPOSURE) {
                coneGirl.camera.exposureControl.setExposure(CamTest.EXPOSURE.toLong(), TimeUnit.MILLISECONDS)
                LE = CamTest.EXPOSURE
            }

            sleep(2)
        }

        coneGirl.stop()
    }
}