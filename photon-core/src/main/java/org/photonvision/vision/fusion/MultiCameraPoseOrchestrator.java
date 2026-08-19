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
 * MERCHANTABILITY OR FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <https://www.gnu.org/licenses/>.
 */

package org.photonvision.vision.fusion;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.concurrent.TimeUnit;
import org.photonvision.common.configuration.ConfigManager;
import org.photonvision.common.configuration.MultiCameraFusionConfig;
import org.photonvision.common.logging.LogGroup;
import org.photonvision.common.logging.Logger;
import org.photonvision.vision.pipeline.result.CVPipelineResult;
import org.photonvision.vision.processes.VisionModule;

/**
 * Collects per-camera 3D observations and runs enabled pose comparison streams on a shared time
 * window.
 */
public class MultiCameraPoseOrchestrator {
    private static final Logger logger =
            new Logger(MultiCameraPoseOrchestrator.class, LogGroup.VisionModule);

    private static class Holder {
        static final MultiCameraPoseOrchestrator INSTANCE = new MultiCameraPoseOrchestrator();
    }

    public static MultiCameraPoseOrchestrator getInstance() {
        return Holder.INSTANCE;
    }

    private final Map<String, CameraObservation> latest = new HashMap<>();
    private final PoseComparePublisher publisher = new PoseComparePublisher();

    MultiCameraPoseOrchestrator() {}

    public void register(VisionModule module) {
        module.addResultConsumer(result -> accept(module, result));
    }

    public void unregister(String uniqueName) {
        synchronized (this) {
            latest.remove(uniqueName);
        }
    }

    public void accept(VisionModule module, CVPipelineResult result) {
        var fusion = config();
        if (!fusion.enabled) {
            publisher.setCsvEnabled(false);
            return;
        }
        publisher.setCsvEnabled(fusion.writeCsv);

        var config = module.getStateAsCameraConfig();
        var layout = ConfigManager.getInstance().getConfig().getApriltagFieldLayout();
        var maybeObs = CameraObservationFactory.fromResult(config, result, layout);
        synchronized (this) {
            if (maybeObs.isEmpty()) {
                latest.remove(config.uniqueName);
                return;
            }
            var obs = maybeObs.get();
            latest.put(obs.uniqueName, obs);
            var window = window(obs, fusion);
            var all = runStreams(window, fusion);
            if (!all.isEmpty()) {
                publisher.publish(all);
            }
        }
    }

    /** Test hook: inject a copied observation and run streams without NT/CSV publish. */
    public synchronized List<PoseEstimate> acceptObservation(
            CameraObservation obs, MultiCameraFusionConfig fusion) {
        if (obs == null || !obs.hasPose() && obs.tags.isEmpty()) {
            if (obs != null) {
                latest.remove(obs.uniqueName);
            }
            return List.of();
        }
        latest.put(obs.uniqueName, obs);
        return runStreams(window(obs, fusion), fusion);
    }

    ObservationWindow window(CameraObservation trigger, MultiCameraFusionConfig fusion) {
        long maxDtNanos = TimeUnit.MILLISECONDS.toNanos((long) Math.max(0, fusion.maxDtMs));
        var cams = new ArrayList<CameraObservation>();
        for (var obs : latest.values()) {
            if (Math.abs(obs.captureTimestampNanos - trigger.captureTimestampNanos) <= maxDtNanos) {
                cams.add(obs);
            }
        }
        return new ObservationWindow(trigger.captureTimestampNanos, cams);
    }

    private List<PoseEstimate> runStreams(ObservationWindow window, MultiCameraFusionConfig fusion) {
        var streams = new ArrayList<PoseEstimatorStream>();
        if (fusion.baseline) {
            streams.add(new BaselinePoseStream());
        }
        if (fusion.weightedAverage) {
            streams.add(new WeightedAveragePoseStream(fusion.minCameras));
        }
        if (fusion.jointPnp) {
            streams.add(
                    new JointPnpPoseStream(
                            fusion.minCameras,
                            () -> ConfigManager.getInstance().getConfig().getApriltagFieldLayout()));
        }
        return runAll(streams, window);
    }

    /** Visible for tests: run streams independently so one failure cannot silence the others. */
    static List<PoseEstimate> runAll(List<PoseEstimatorStream> streams, ObservationWindow window) {
        var all = new ArrayList<PoseEstimate>();
        for (var stream : streams) {
            try {
                all.addAll(stream.estimate(window));
            } catch (Exception e) {
                logger.error("Pose comparison stream " + stream.name() + " failed", e);
            }
        }
        return all;
    }

    private static MultiCameraFusionConfig config() {
        return ConfigManager.getInstance().getConfig().getFusionConfig();
    }
}
