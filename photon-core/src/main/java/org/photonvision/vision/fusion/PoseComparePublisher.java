/*
 * Copyright (C) Photon Vision.
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <https://www.gnu.org/licenses/>.
 */

package org.photonvision.vision.fusion;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.IntegerArrayPublisher;
import edu.wpi.first.networktables.IntegerPublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.StringArrayPublisher;
import edu.wpi.first.networktables.StructPublisher;
import java.io.BufferedWriter;
import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Path;
import java.nio.file.StandardOpenOption;
import java.time.LocalDateTime;
import java.time.format.DateTimeFormatter;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import org.photonvision.common.configuration.PathManager;
import org.photonvision.common.dataflow.DataChangeService;
import org.photonvision.common.dataflow.events.OutgoingUIEvent;
import org.photonvision.common.dataflow.networktables.NetworkTablesManager;
import org.photonvision.common.logging.LogGroup;
import org.photonvision.common.logging.Logger;
import org.photonvision.common.util.math.MathUtils;

/** Publishes comparison stream poses to NT, CSV, and the UI. */
public class PoseComparePublisher {
    private static final Logger logger = new Logger(PoseComparePublisher.class, LogGroup.General);
    private static final DateTimeFormatter CSV_TIME =
            DateTimeFormatter.ofPattern("yyyy-MM-dd_HH-mm-ss");

    private NetworkTable root;
    private final Map<String, StreamTopics> topics = new HashMap<>();
    private BufferedWriter csvWriter;
    private Path csvPath;
    private boolean csvEnabled;

    public PoseComparePublisher() {}

    private NetworkTable root() {
        if (root == null) {
            root = NetworkTablesManager.getInstance().kRootTable.getSubTable("poseCompare");
        }
        return root;
    }

    public synchronized void setCsvEnabled(boolean enabled) {
        if (enabled && !csvEnabled) {
            openCsv();
            csvEnabled = csvWriter != null;
            return;
        } else if (!enabled && csvEnabled) {
            closeCsv();
        }
        csvEnabled = enabled;
    }

    public synchronized void publish(List<PoseEstimate> estimates) {
        try {
            var offset = NetworkTablesManager.getInstance().getOffset();
            var uiEstimates = new java.util.ArrayList<HashMap<String, Object>>();
            for (var est : estimates) {
                publishNt(est, offset);
                writeCsv(est, offset);
                uiEstimates.add(toUi(est, offset));
            }
            var payload = new HashMap<String, Object>();
            payload.put("estimates", uiEstimates);
            DataChangeService.getInstance()
                    .publishEvent(OutgoingUIEvent.wrappedOf("updatePoseCompare", payload));
            if (csvWriter != null) {
                csvWriter.flush();
            }
        } catch (Throwable e) {
            logger.error("Failed to publish pose comparison results", e);
        }
    }

    private void publishNt(PoseEstimate est, long offsetMicros) {
        var table = tableFor(est);
        var t = topics.computeIfAbsent(keyFor(est), k -> new StreamTopics(table));
        t.pose.set(est.fieldToRobot);
        t.timestamp.set(MathUtils.nanosToMicros(est.timestampNanos) + offsetMicros);
        t.hasPose.set(true);
        t.camerasUsed.set(est.camerasUsed.toArray(String[]::new));
        long[] ids = new long[est.fiducialIdsUsed.size()];
        for (int i = 0; i < ids.length; i++) {
            ids[i] = est.fiducialIdsUsed.get(i);
        }
        t.fiducialIDsUsed.set(ids);
        t.reprojErr.set(est.reprojErr);
        t.computeMs.set(est.computeMs);
    }

    private String keyFor(PoseEstimate est) {
        return est.method + "/" + (est.camera != null ? est.camera : "_");
    }

    private NetworkTable tableFor(PoseEstimate est) {
        if (BaselinePoseStream.NAME.equals(est.method) && est.camera != null) {
            return root().getSubTable(est.method).getSubTable(est.camera);
        }
        return root().getSubTable(est.method);
    }

    private HashMap<String, Object> toUi(PoseEstimate est, long offsetMicros) {
        var map = new HashMap<String, Object>();
        map.put("method", est.method);
        map.put("camera", est.camera);
        map.put("x", est.fieldToRobot.getX());
        map.put("y", est.fieldToRobot.getY());
        map.put("z", est.fieldToRobot.getZ());
        map.put("roll", est.fieldToRobot.getRotation().getX());
        map.put("pitch", est.fieldToRobot.getRotation().getY());
        map.put("yaw", est.fieldToRobot.getRotation().getZ());
        map.put("timestamp", MathUtils.nanosToMicros(est.timestampNanos) + offsetMicros);
        map.put("camerasUsed", est.camerasUsed);
        map.put("fiducialIdsUsed", est.fiducialIdsUsed);
        map.put("reprojErr", est.reprojErr);
        map.put("computeMs", est.computeMs);
        return map;
    }

    private void openCsv() {
        try {
            var dir = PathManager.getInstance().getLogsDir();
            Files.createDirectories(dir);
            csvPath = dir.resolve("pose-compare-" + CSV_TIME.format(LocalDateTime.now()) + ".csv");
            csvWriter =
                    Files.newBufferedWriter(
                            csvPath, StandardOpenOption.CREATE, StandardOpenOption.TRUNCATE_EXISTING);
            csvWriter.write(
                    "timestamp_us,method,camera,x,y,z,roll,pitch,yaw,n_cameras,n_tags,reproj_err,compute_ms\n");
            csvWriter.flush();
            logger.info("Pose comparison CSV: " + csvPath);
        } catch (IOException e) {
            logger.error("Failed to open pose comparison CSV", e);
            csvWriter = null;
        }
    }

    private void writeCsv(PoseEstimate est, long offsetMicros) {
        if (csvWriter == null) {
            return;
        }
        var r = est.fieldToRobot.getRotation();
        try {
            csvWriter.write(
                    String.format(
                            "%d,%s,%s,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%d,%d,%.6f,%.3f%n",
                            MathUtils.nanosToMicros(est.timestampNanos) + offsetMicros,
                            est.method,
                            est.camera == null ? "" : est.camera,
                            est.fieldToRobot.getX(),
                            est.fieldToRobot.getY(),
                            est.fieldToRobot.getZ(),
                            r.getX(),
                            r.getY(),
                            r.getZ(),
                            est.camerasUsed.size(),
                            est.fiducialIdsUsed.size(),
                            est.reprojErr,
                            est.computeMs));
        } catch (IOException e) {
            logger.error("Failed to write pose comparison CSV row", e);
        }
    }

    private void closeCsv() {
        if (csvWriter != null) {
            try {
                csvWriter.close();
            } catch (IOException e) {
                logger.error("Failed to close pose comparison CSV", e);
            }
            csvWriter = null;
        }
    }

    private static class StreamTopics {
        final StructPublisher<Pose3d> pose;
        final IntegerPublisher timestamp;
        final BooleanPublisher hasPose;
        final StringArrayPublisher camerasUsed;
        final IntegerArrayPublisher fiducialIDsUsed;
        final DoublePublisher reprojErr;
        final DoublePublisher computeMs;

        StreamTopics(NetworkTable table) {
            pose = table.getStructTopic("pose", Pose3d.struct).publish();
            timestamp = table.getIntegerTopic("timestamp").publish();
            hasPose = table.getBooleanTopic("hasPose").publish();
            camerasUsed = table.getStringArrayTopic("camerasUsed").publish();
            fiducialIDsUsed = table.getIntegerArrayTopic("fiducialIDsUsed").publish();
            reprojErr = table.getDoubleTopic("reprojErr").publish();
            computeMs = table.getDoubleTopic("computeMs").publish();
        }
    }
}
