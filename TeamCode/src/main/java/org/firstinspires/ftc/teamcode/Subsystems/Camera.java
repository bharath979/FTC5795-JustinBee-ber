package org.firstinspires.ftc.teamcode.Subsystems;

import android.annotation.SuppressLint;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagClusterDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.vision.apriltag.AprilTagSingleDetection;

import java.util.List;


public class Camera {
    private AprilTagProcessor aprilTag;
    private VisionPortal visionPortal;

    public void initiate(HardwareMap hardwareMap) {
        initiate(hardwareMap, "Webcam 1");
    }

    public void initiate(HardwareMap hardwareMap, String webcamName) {
        aprilTag = new AprilTagProcessor.Builder()
                .build();

        visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, webcamName))
                .addProcessor(aprilTag)
                .build();
    }


    public List<AprilTagDetection> getDetections() {
        return aprilTag.getDetections();
    }


    public AprilTagSingleDetection getDetection(int tagId) {
        for (AprilTagDetection detection : getDetections()) {
            if (detection instanceof AprilTagSingleDetection) {
                AprilTagSingleDetection singleDetection = (AprilTagSingleDetection) detection;
                if (singleDetection.id == tagId) {
                    return singleDetection;
                }
            }
        }
        return null;
    }

    @SuppressLint("DefaultLocale")
    public void update(Telemetry telemetry) {
        List<AprilTagDetection> detections = getDetections();
        telemetry.addData("Hive tags visible", detections.size());
        for (AprilTagDetection detection : detections) {
            if (detection instanceof AprilTagSingleDetection) {
                AprilTagSingleDetection singleDetection = (AprilTagSingleDetection) detection;
                if (singleDetection.metadata != null) {
                    telemetry.addLine(String.format("ID %d (%s): range=%.1fin bearing=%.1fdeg yaw=%.1fdeg",
                            singleDetection.id, singleDetection.metadata.name,
                            detection.ftcPose.range, detection.ftcPose.bearing, detection.ftcPose.yaw));
                } else {
                    telemetry.addLine(String.format("ID %d: untracked tag, center=(%.0f, %.0f)",
                            singleDetection.id, singleDetection.center.x, singleDetection.center.y));
                }
            } else {
                AprilTagClusterDetection clusterDetection = (AprilTagClusterDetection) detection;
                telemetry.addLine(String.format("Tag cluster (%s): range=%.1fin bearing=%.1fdeg yaw=%.1fdeg",
                        clusterDetection.metadata.name,
                        detection.ftcPose.range, detection.ftcPose.bearing, detection.ftcPose.yaw));
            }
        }
    }


    public void close() {
        if (visionPortal != null) {
            visionPortal.close();
        }
    }
}
