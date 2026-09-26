package org.firstinspires.ftc.teamcode.Subsystems;

import android.annotation.SuppressLint;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.Position;

import java.util.ArrayList;
import java.util.Collections;
import java.util.HashMap;
import java.util.List;
import java.util.Map;


public class RedCamera {

    private static final double POSITION_PROCESS_NOISE = 0.02;
    private static final double POSITION_MEASUREMENT_NOISE = 2.0;
    private static final double ANGLE_PROCESS_NOISE = 0.05;
    private static final double ANGLE_MEASUREMENT_NOISE = 4.0;

    // Limelight pipeline index set up for AprilTags (36h11) in the Limelight web UI
    private static final int APRILTAG_PIPELINE = 0;

    // Red CELL AprilTag IDs
    public static final int[] AUDIENCE_SIDE_TAG_IDS = {34, 35, 36, 37};
    public static final int[] FAR_SIDE_TAG_IDS = {30, 31, 32, 33};

    private Limelight3A limelight;
    private final Map<Integer, TagPoseFilter> poseFilters = new HashMap<>();

    public void initiate(HardwareMap hardwareMap) {
        initiate(hardwareMap, "limelight");
    }

    public void initiate(HardwareMap hardwareMap, String limelightName) {
        limelight = hardwareMap.get(Limelight3A.class, limelightName);
        limelight.pipelineSwitch(APRILTAG_PIPELINE);
        limelight.start();
    }


    public List<LLResultTypes.FiducialResult> getDetections() {
        LLResult result = limelight.getLatestResult();
        if (result == null || !result.isValid()) {
            return Collections.emptyList();
        }
        return result.getFiducialResults();
    }


    public LLResultTypes.FiducialResult getDetection(int tagId) {
        for (LLResultTypes.FiducialResult detection : getDetections()) {
            if (detection.getFiducialId() == tagId) {
                return detection;
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

    public List<LLResultTypes.FiducialResult> getAllianceDetections() {
        List<LLResultTypes.FiducialResult> allianceDetections = new ArrayList<>();
        for (LLResultTypes.FiducialResult detection : getDetections()) {
            if (isAllianceTag(detection.getFiducialId())) {
                allianceDetections.add(detection);
            }
        }
        return allianceDetections;
    }

    /** Tag pose relative to the camera, in inches and degrees (x right, y forward, z up). */
    public static class TagPose {
        public final double x, y, z, yaw, range, bearing;

        TagPose(double x, double y, double z, double yaw, double range, double bearing) {
            this.x = x;
            this.y = y;
            this.z = z;
            this.yaw = yaw;
            this.range = range;
            this.bearing = bearing;
        }
    }

    public static TagPose getTagPose(LLResultTypes.FiducialResult detection) {
        Pose3D pose = detection.getTargetPoseCameraSpace();
        if (pose == null) {
            return null;
        }
        // Limelight camera space: +x right, +y down, +z out of the lens
        Position position = pose.getPosition().toUnit(DistanceUnit.INCH);
        double x = position.x;
        double y = position.z;
        double z = -position.y;
        double yaw = pose.getOrientation().getYaw(AngleUnit.DEGREES);
        double range = Math.hypot(x, y);
        // Limelight tx is positive to the right; bearing is positive to the left
        double bearing = -detection.getTargetXDegrees();
        return new TagPose(x, y, z, yaw, range, bearing);
    }

    private static class TagPoseFilter {
        final KalmanFilter x = new KalmanFilter(POSITION_PROCESS_NOISE, POSITION_MEASUREMENT_NOISE);
        final KalmanFilter y = new KalmanFilter(POSITION_PROCESS_NOISE, POSITION_MEASUREMENT_NOISE);
        final KalmanFilter z = new KalmanFilter(POSITION_PROCESS_NOISE, POSITION_MEASUREMENT_NOISE);
        final KalmanFilter yaw = new KalmanFilter(ANGLE_PROCESS_NOISE, ANGLE_MEASUREMENT_NOISE);
        final KalmanFilter range = new KalmanFilter(POSITION_PROCESS_NOISE, POSITION_MEASUREMENT_NOISE);
        final KalmanFilter bearing = new KalmanFilter(ANGLE_PROCESS_NOISE, ANGLE_MEASUREMENT_NOISE);

        TagPose update(TagPose pose) {
            return new TagPose(
                    x.update(pose.x),
                    y.update(pose.y),
                    z.update(pose.z),
                    yaw.update(pose.yaw),
                    range.update(pose.range),
                    bearing.update(pose.bearing));
        }
    }
    public TagPose getFilteredPose(int tagId) {
        LLResultTypes.FiducialResult detection = getDetection(tagId);
        if (detection == null) {
            return null;
        }
        TagPose pose = getTagPose(detection);
        if (pose == null) {
            return null;
        }
        TagPoseFilter filter = poseFilters.computeIfAbsent(tagId, id -> new TagPoseFilter());
        return filter.update(pose);
    }

    @SuppressLint("DefaultLocale")
    public void update(Telemetry telemetry) {
        if (!limelight.isConnected()) {
            telemetry.addLine("Limelight not connected");
            return;
        }
        List<LLResultTypes.FiducialResult> detections = getDetections();
        telemetry.addData("Hive tags visible", detections.size());
        telemetry.addData("Red tags visible", getAllianceDetections().size());
        for (LLResultTypes.FiducialResult detection : detections) {
            int id = detection.getFiducialId();
            if (!isAllianceTag(id)) {
                continue;
            }
            String side = isAudienceSideTag(id) ? "audience" : "far";
            TagPose pose = getTagPose(detection);
            if (pose == null) {
                telemetry.addLine(String.format("ID %d (red %s side): tx=%.1fdeg ty=%.1fdeg",
                        id, side, detection.getTargetXDegrees(), detection.getTargetYDegrees()));
                continue;
            }
            telemetry.addLine(String.format("ID %d (red %s side): range=%.1fin bearing=%.1fdeg yaw=%.1fdeg",
                    id, side, pose.range, pose.bearing, pose.yaw));
            TagPose filtered = getFilteredPose(id);
            if (filtered != null) {
                telemetry.addLine(String.format("  filtered: range=%.1fin bearing=%.1fdeg yaw=%.1fdeg",
                        filtered.range, filtered.bearing, filtered.yaw));
            }
        }
    }


    public void close() {
        if (limelight != null) {
            limelight.stop();
        }
    }
}
