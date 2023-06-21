package org.firstinspires.ftc.teamcode.mk3.tests

import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.config.Config
import com.acmerobotics.dashboard.telemetry.TelemetryPacket
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.AnalogInput
import com.qualcomm.robotcore.hardware.DigitalChannel
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
    var GAIN: Int = 150

    @JvmField
    var EXPOSURE: Int = 10

    @JvmField
    var UPDATE_EXP: Boolean = false
}

@TeleOp
class CameraTest : LinearOpMode() {
    override fun runOpMode() {
        val conePipeline = ConePipeline(480, 640)
        val coneGirl = CamGirl(this, "coneGirl", OpenCvCameraRotation.UPSIDE_DOWN, 640, 480, conePipeline, streaming = true, waitForOpen = true)
        val lamprey1 = hardwareMap.get(AnalogInput::class.java, "lamprey");
        val lamprey2 = hardwareMap.get(AnalogInput::class.java, "lamprey2");
        lamprey1.voltage

        var LG = CamTest.GAIN
        var LE = CamTest.EXPOSURE
        var LU = CamTest.UPDATE_EXP

        waitForStart();

        while (opModeIsActive()) {
            val ttp = TelemetryPacket()
            ttp.put("L1_MAX", lamprey1.maxVoltage)
            ttp.put("L1_CUR", lamprey1.voltage)
            ttp.put("L2_MAX", lamprey2.maxVoltage)
            ttp.put("L2_CUR", lamprey2.voltage)
            FtcDashboard.getInstance().sendTelemetryPacket(ttp)
            if (LG != CamTest.GAIN) {
                coneGirl.camera.gainControl.gain = CamTest.GAIN
                LG = CamTest.GAIN
            }

            if (LE != CamTest.EXPOSURE) {
                coneGirl.camera.exposureControl.setExposure(CamTest.EXPOSURE.toLong(), TimeUnit.MILLISECONDS)
                LE = CamTest.EXPOSURE
            }

            if (LU != CamTest.UPDATE_EXP) {
                val tp = TelemetryPacket()
                tp.put("oldexp", coneGirl.expos)
                tp.put("newexp", coneGirl.expos + conePipeline.getRecommendedExposureDifference())
                FtcDashboard.getInstance().sendTelemetryPacket(tp)
                coneGirl.updateExposure(coneGirl.expos + conePipeline.getRecommendedExposureDifference())
                LU = CamTest.UPDATE_EXP
            }

            sleep(2)
        }

        coneGirl.stop()
    }
}