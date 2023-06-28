package org.firstinspires.ftc.teamcode.mk3.camera

import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.telemetry.TelemetryPacket
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.OpMode
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl
import org.openftc.easyopencv.*
import java.lang.Thread.sleep
import java.util.concurrent.TimeUnit

/// Thanks to Brainstormz for generously "gifting" this code to us

class CamGirl(lom: LinearOpMode,
              name: String,
              orientation: OpenCvCameraRotation,
              resX: Int,
              resY: Int,
              pipeline: OpenCvPipeline,
              streaming: Boolean,
              waitForOpen: Boolean,
              gain: Int,
              exposure: Int) {

    constructor(lom: LinearOpMode,
                name: String,
                orientation: OpenCvCameraRotation,
                resX: Int,
                resY: Int,
                pipeline: OpenCvPipeline
    ) : this(lom, name, orientation, resX, resY, pipeline, false, false)

    constructor(lom: LinearOpMode,
                name: String,
                orientation: OpenCvCameraRotation,
                resX: Int,
                resY: Int,
                pipeline: OpenCvPipeline,
                streaming: Boolean,
                waitForOpen: Boolean,
    ) : this(lom, name, orientation, resX, resY, pipeline, streaming, waitForOpen, 100, 0)

    var camera: OpenCvWebcam
    var ecode: Int = 0
    var opened: Boolean = false
    var expos: Int = 10
    private var dashboardStreaming = false

    init {
        val cameraMonitorViewId: Int = lom.hardwareMap.appContext.resources.getIdentifier("cameraMonitorViewId", "id", lom.hardwareMap.appContext.packageName)
        val webcamName: WebcamName = lom.hardwareMap.get(WebcamName::class.java, name)
        camera = OpenCvCameraFactory.getInstance().createWebcam(webcamName, cameraMonitorViewId)
        camera.setPipeline(pipeline)

        expos = exposure

        dashboardStreaming = streaming
        val cameraListener = object : OpenCvCamera.AsyncCameraOpenListener {
            override fun onOpened() {
                if (exposure != 0) {
                    camera.exposureControl.mode = ExposureControl.Mode.Manual
                    camera.exposureControl.setExposure(expos.toLong(), TimeUnit.MILLISECONDS)
                }
                camera.gainControl.gain = gain

                camera.startStreaming(resX, resY, orientation)
                if (streaming) {
                    FtcDashboard.getInstance().startCameraStream(camera, 30.0)
                }
                opened = true
            }

            override fun onError(errorCode: Int) {
                ecode = errorCode
            }
        }

        camera.openCameraDeviceAsync(cameraListener)
        while (waitForOpen && !opened && !lom.isStopRequested) {
            val tp = TelemetryPacket()
            tp.addLine("Currently waiting on cam open")
            FtcDashboard.getInstance().sendTelemetryPacket(tp)
            lom.telemetry.addLine("Waiting on cam open")
            lom.telemetry.update()
            sleep(5)
        }
        lom.telemetry.addLine("Cam opened")
        lom.telemetry.update()

    }

    fun stop() {
        camera.stopStreaming()
        if (dashboardStreaming) {
            FtcDashboard.getInstance().stopCameraStream();
        }
    }
}