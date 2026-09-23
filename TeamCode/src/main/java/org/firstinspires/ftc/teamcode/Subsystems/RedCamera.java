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

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;


public class RedCamera {

    private static final double POSITION_PROCESS_NOISE = 0.02;
    private static final double POSITION_MEASUREMENT_NOISE = 2.0;
    private static final double ANGLE_PROCESS_NOISE = 0.05;
    private static final double ANGLE_MEASUREMENT_NOISE = 4.0;

    // Red CELL AprilTag IDs
    public static final int[] AUDIENCE_SIDE_TAG_IDS = {34, 35, 36, 37};
    public static final int[] FAR_SIDE_TAG_IDS = {30, 31, 32, 33};

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

    public static boolean isAudienceSideTag(int tagId) {
        return containsId(AUDIENCE_SIDE_TAG_IDS, tagId);
    }

    public static boolean isFarSideTag(int tagId) {
        return containsId(FAR_SIDE_TAG_IDS, tagId);
    }

    public static boolean isAllianceTag(int tagId) {
        return isAudienceSideTag(tagId) || isFarSideTag(tagId);
    }

    private static boolean containsId(int[] ids, int tagId) {
        for (int id : ids) {
            if (id == tagId) {
                return true;
            }
        }
        return false;
    }

    public List<AprilTagSingleDetection> getAllianceDetections() {
        List<AprilTagSingleDetection> allianceDetections = new ArrayList<>();
        for (AprilTagDetection detection : getDetections()) {
            if (detection instanceof AprilTagSingleDetection) {
                AprilTagSingleDetection singleDetection = (AprilTagSingleDetection) detection;
                if (isAllianceTag(singleDetection.id)) {
                    allianceDetections.add(singleDetection);
                }
            }
        }
        return allianceDetections;
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
        final KalmanFilter x = new KalmanFilter(POSITION_PROCESS_NOISE, POSITION_MEASUREMENT_NOISE);
        final KalmanFilter y = new KalmanFilter(POSITION_PROCESS_NOISE, POSITION_MEASUREMENT_NOISE);
        final KalmanFilter z = new KalmanFilter(POSITION_PROCESS_NOISE, POSITION_MEASUREMENT_NOISE);
        final KalmanFilter yaw = new KalmanFilter(ANGLE_PROCESS_NOISE, ANGLE_MEASUREMENT_NOISE);
        final KalmanFilter range = new KalmanFilter(POSITION_PROCESS_NOISE, POSITION_MEASUREMENT_NOISE);
        final KalmanFilter bearing = new KalmanFilter(ANGLE_PROCESS_NOISE, ANGLE_MEASUREMENT_NOISE);

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
        telemetry.addData("Red tags visible", getAllianceDetections().size());
        for (AprilTagDetection detection : detections) {
            if (detection instanceof AprilTagSingleDetection) {
                AprilTagSingleDetection singleDetection = (AprilTagSingleDetection) detection;
                if (!isAllianceTag(singleDetection.id)) {
                    continue;
                }
                if (singleDetection.metadata != null) {
                    String side = isAudienceSideTag(singleDetection.id) ? "audience" : "far";
                    telemetry.addLine(String.format("ID %d (%s, red %s side): range=%.1fin bearing=%.1fdeg yaw=%.1fdeg",
                            singleDetection.id, singleDetection.metadata.name, side,
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
