package org.firstinspires.ftc.teamcode.Subsystems;

import android.annotation.SuppressLint;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagClusterDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase;
import org.firstinspires.ftc.vision.apriltag.AprilTagPoseFtc;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.vision.apriltag.AprilTagSingleDetection;

import java.util.HashMap;
import java.util.List;
import java.util.Map;


public class Camera {
    // Tuned for AprilTag pose noise: low process noise (tag isn't expected to
    // "teleport" between frames) and moderate measurement noise (raw pose jitter).
    private static final double POSITION_PROCESS_NOISE = 0.02;
    private static final double POSITION_MEASUREMENT_NOISE = 2.0;
    private static final double ANGLE_PROCESS_NOISE = 0.05;
    private static final double ANGLE_MEASUREMENT_NOISE = 4.0;

    private AprilTagProcessor aprilTag;
    private VisionPortal visionPortal;
    private final Map<Integer, TagPoseFilter> poseFilters = new HashMap<>();

    public void initiate(HardwareMap hardwareMap) {
        initiate(hardwareMap, "Webcam 1");
    }

    public void initiate(HardwareMap hardwareMap, String webcamName) {
        aprilTag = new AprilTagProcessor.Builder()
                .setTagLibrary(AprilTagGameDatabase.getCurrentGameTagLibrary())
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
    public static class FilteredPose {
        public final double x, y, z, yaw, range, bearing;

        FilteredPose(double x, double y, double z, double yaw, double range, double bearing) {
            this.x = x;
            this.y = y;
            this.z = z;
            this.yaw = yaw;
            this.range = range;
            this.bearing = bearing;
        }
    }

    private static class TagPoseFilter {
        final KalmanFilter1D x = new KalmanFilter1D(POSITION_PROCESS_NOISE, POSITION_MEASUREMENT_NOISE);
        final KalmanFilter1D y = new KalmanFilter1D(POSITION_PROCESS_NOISE, POSITION_MEASUREMENT_NOISE);
        final KalmanFilter1D z = new KalmanFilter1D(POSITION_PROCESS_NOISE, POSITION_MEASUREMENT_NOISE);
        final KalmanFilter1D yaw = new KalmanFilter1D(ANGLE_PROCESS_NOISE, ANGLE_MEASUREMENT_NOISE);
        final KalmanFilter1D range = new KalmanFilter1D(POSITION_PROCESS_NOISE, POSITION_MEASUREMENT_NOISE);
        final KalmanFilter1D bearing = new KalmanFilter1D(ANGLE_PROCESS_NOISE, ANGLE_MEASUREMENT_NOISE);

        FilteredPose update(AprilTagPoseFtc pose) {
            return new FilteredPose(
                    x.update(pose.x),
                    y.update(pose.y),
                    z.update(pose.z),
                    yaw.update(pose.yaw),
                    range.update(pose.range),
                    bearing.update(pose.bearing));
        }
    }
    public FilteredPose getFilteredPose(int tagId) {
        AprilTagSingleDetection detection = getDetection(tagId);
        if (detection == null || detection.ftcPose == null) {
            return null;
        }
        TagPoseFilter filter = poseFilters.computeIfAbsent(tagId, id -> new TagPoseFilter());
        return filter.update(detection.ftcPose);
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
                    FilteredPose filtered = getFilteredPose(singleDetection.id);
                    if (filtered != null) {
                        telemetry.addLine(String.format("  filtered: range=%.1fin bearing=%.1fdeg yaw=%.1fdeg",
                                filtered.range, filtered.bearing, filtered.yaw));
                    }
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
