package org.firstinspires.ftc.teamcode.mk3.camera

import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.config.Config
import com.qualcomm.robotcore.eventloop.opmode.OpMode
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl
import org.firstinspires.ftc.teamcode.mk3.tests.CamTest
import org.opencv.videoio.VideoCapture
import org.opencv.videoio.Videoio.CAP_PROP_AUTO_EXPOSURE
import org.openftc.easyopencv.*
import java.lang.Thread.sleep
import java.util.concurrent.TimeUnit

/// Thanks to Brainstormz for generously "gifting" this code to us

class CamGirl(opm: OpMode,
              name: String,
              orientation: OpenCvCameraRotation,
              resX: Int,
              resY: Int,
              pipeline: OpenCvPipeline,
              streaming: Boolean,
              waitForOpen: Boolean,
              gain: Int,
              exposure: Int) {

    constructor(opm: OpMode,
                name: String,
                orientation: OpenCvCameraRotation,
                resX: Int,
                resY: Int,
                pipeline: OpenCvPipeline
    ) : this(opm, name, orientation, resX, resY, pipeline, false, false)

    constructor(opm: OpMode,
                name: String,
                orientation: OpenCvCameraRotation,
                resX: Int,
                resY: Int,
                pipeline: OpenCvPipeline,
                streaming: Boolean,
                waitForOpen: Boolean,
    ) : this(opm, name, orientation, resX, resY, pipeline, streaming, waitForOpen, 100, 0)

    var camera: OpenCvWebcam
    var ecode: Int = 0
    var opened: Boolean = false
    private var dashboardStreaming = false

    init {
        val cameraMonitorViewId: Int = opm.hardwareMap.appContext.resources.getIdentifier("cameraMonitorViewId", "id", opm.hardwareMap.appContext.packageName)
        val webcamName: WebcamName = opm.hardwareMap.get(WebcamName::class.java, name)
        camera = OpenCvCameraFactory.getInstance().createWebcam(webcamName, cameraMonitorViewId)
        camera.setPipeline(pipeline)

        if (exposure != 0) {
            camera.exposureControl.mode = ExposureControl.Mode.Manual
            camera.exposureControl.setExposure(10, TimeUnit.MILLISECONDS)
        }

        camera.gainControl.gain = gain

        dashboardStreaming = streaming
        val cameraListener = object : OpenCvCamera.AsyncCameraOpenListener {
            override fun onOpened() {
                camera.startStreaming(resX, resY, orientation)
                if (streaming) {
                    FtcDashboard.getInstance().startCameraStream(camera, 20.0);
                }
                opened = true
            }

            override fun onError(errorCode: Int) {
                ecode = errorCode
            }
        }

        camera.openCameraDeviceAsync(cameraListener)
        while (waitForOpen && !opened) {
            sleep(5)
        }
    }

    fun setExposure(exp: Long) {
        camera.exposureControl.mode = ExposureControl.Mode.Manual
        camera.exposureControl.setExposure(exp, TimeUnit.MILLISECONDS)
    }

    fun stop() {
        camera.stopStreaming()
        if (dashboardStreaming) {
            FtcDashboard.getInstance().stopCameraStream();
        }
    }
}