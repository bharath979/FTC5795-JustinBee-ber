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


public class BlueCamera {

    private static final double POSITION_PROCESS_NOISE = 0.02;
    private static final double POSITION_MEASUREMENT_NOISE = 2.0;
    private static final double ANGLE_PROCESS_NOISE = 0.05;
    private static final double ANGLE_MEASUREMENT_NOISE = 4.0;

    // Limelight pipeline index set up for AprilTags (36h11) in the Limelight web UI
    private static final int APRILTAG_PIPELINE = 0;

    // How far the Limelight is tilted up from level to see the hive.
    // Placeholder: measure the mount angle on the robot.
    private static final double CAMERA_PITCH_DEGREES = 20.0;

    // Blue CELL AprilTag IDs
    public static final int[] AUDIENCE_SIDE_TAG_IDS = {38, 39, 40, 41};
    public static final int[] FAR_SIDE_TAG_IDS = {42, 43, 44, 45};

    private Limelight3A limelight;
    private final Map<Integer, TagPoseFilter> poseFilters = new HashMap<>();
    private boolean aimSideChosen;
    private boolean aimingFarSide;

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

    /** True if this one specific tag is in view right now. */
    public boolean isTagVisible(int tagId) {
        return getDetection(tagId) != null;
    }

    /** IDs of every tag in view right now, alliance or not. */
    public List<Integer> getVisibleTagIds() {
        List<Integer> ids = new ArrayList<>();
        for (LLResultTypes.FiducialResult detection : getDetections()) {
            ids.add(detection.getFiducialId());
        }
        return ids;
    }

    /**
     * Limelight tx (degrees, positive = right) of one specific tag, or null
     * if it isn't visible. Pass to the turret to aim at that tag only.
     */
    public Double getTx(int tagId) {
        LLResultTypes.FiducialResult detection = getDetection(tagId);
        return detection == null ? null : detection.getTargetXDegrees();
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

    /**
     * The tags to aim with: only the audience-side group (38-41) or only the
     * far-side group (42-45), never a mix. The current side is kept as long as
     * any of its tags are visible, so the aim doesn't jump between clusters;
     * it only switches when the current side is gone and the other side is in
     * view (the bigger, closer one if both are seen at first). Empty if no
     * blue hive tags are visible.
     */
    public List<LLResultTypes.FiducialResult> getAimDetections() {
        List<LLResultTypes.FiducialResult> audience = new ArrayList<>();
        List<LLResultTypes.FiducialResult> far = new ArrayList<>();
        double audienceArea = 0;
        double farArea = 0;
        for (LLResultTypes.FiducialResult detection : getDetections()) {
            if (isAudienceSideTag(detection.getFiducialId())) {
                audience.add(detection);
                audienceArea += detection.getTargetArea();
            } else if (isFarSideTag(detection.getFiducialId())) {
                far.add(detection);
                farArea += detection.getTargetArea();
            }
        }
        if (!aimSideChosen) {
            if (audience.isEmpty() && far.isEmpty()) {
                return audience;
            }
            aimingFarSide = farArea > audienceArea;
            aimSideChosen = true;
        } else if (aimingFarSide && far.isEmpty() && !audience.isEmpty()) {
            aimingFarSide = false;
        } else if (!aimingFarSide && audience.isEmpty() && !far.isEmpty()) {
            aimingFarSide = true;
        }
        return aimingFarSide ? far : audience;
    }

    /** Forgets the locked side so the next blue tags seen pick it again. */
    public void resetAimSide() {
        aimSideChosen = false;
    }

    /**
     * Limelight tx (degrees, positive = right) to aim the turret at, or null
     * if no blue tags are visible. Averages the visible tags of one side so
     * the turret points at the middle of that group.
     * Raw tx, not filtered, so the turret loop doesn't lag behind the filter.
     */
    public Double getAimTx() {
        List<LLResultTypes.FiducialResult> group = getAimDetections();
        if (group.isEmpty()) {
            return null;
        }
        double sum = 0;
        for (LLResultTypes.FiducialResult detection : group) {
            sum += detection.getTargetXDegrees();
        }
        return sum / group.size();
    }

    /** The largest (closest) tag of the side being aimed at, or null if none are visible. */
    public LLResultTypes.FiducialResult getAimDetection() {
        LLResultTypes.FiducialResult best = null;
        for (LLResultTypes.FiducialResult detection : getAimDetections()) {
            if (best == null || detection.getTargetArea() > best.getTargetArea()) {
                best = detection;
            }
        }
        return best;
    }

    /**
     * Limelight's timestamp for the latest camera frame. Only changes when a
     * new frame arrives, so it tells the turret whether tx is fresh.
     */
    public double getFrameTimestamp() {
        LLResult result = limelight.getLatestResult();
        return result == null ? 0 : result.getTimestamp();
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
        // Undo the upward tilt so y is distance along the floor and z is height
        double pitch = Math.toRadians(CAMERA_PITCH_DEGREES);
        double x = position.x;
        double y = position.z * Math.cos(pitch) + position.y * Math.sin(pitch);
        double z = position.z * Math.sin(pitch) - position.y * Math.cos(pitch);
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
        telemetry.addData("Visible tag IDs", getVisibleTagIds());
        telemetry.addData("Blue tags visible", getAllianceDetections().size());
        getAimDetections();
        telemetry.addData("Locked onto", !aimSideChosen ? "nothing yet"
                : aimingFarSide ? "far side (42-45)" : "audience side (38-41)");
        for (LLResultTypes.FiducialResult detection : detections) {
            int id = detection.getFiducialId();
            if (!isAllianceTag(id)) {
                continue;
            }
            String side = isAudienceSideTag(id) ? "audience" : "far";
            TagPose pose = getTagPose(detection);
            if (pose == null) {
                telemetry.addLine(String.format("ID %d (blue %s side): tx=%.1fdeg ty=%.1fdeg",
                        id, side, detection.getTargetXDegrees(), detection.getTargetYDegrees()));
                continue;
            }
            telemetry.addLine(String.format("ID %d (blue %s side): range=%.1fin bearing=%.1fdeg yaw=%.1fdeg",
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
