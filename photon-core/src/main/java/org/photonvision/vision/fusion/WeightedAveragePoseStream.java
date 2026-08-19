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

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Quaternion;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import java.util.ArrayList;
import java.util.LinkedHashSet;
import java.util.List;

/** Weighted average of per-camera field-to-robot poses in a time window. */
public class WeightedAveragePoseStream implements PoseEstimatorStream {
    public static final String NAME = "weightedAverage";

    private final int minCameras;

    public WeightedAveragePoseStream(int minCameras) {
        this.minCameras = Math.max(1, minCameras);
    }

    @Override
    public String name() {
        return NAME;
    }

    @Override
    public List<PoseEstimate> estimate(ObservationWindow window) {
        long start = System.nanoTime();
        var withPoses = window.withPoses();
        if (withPoses.size() < minCameras) {
            return List.of();
        }

        double totalWeight = 0;
        double x = 0;
        double y = 0;
        double z = 0;
        double qw = 0;
        double qx = 0;
        double qy = 0;
        double qz = 0;
        double timestamp = 0;
        double reproj = 0;
        var cameras = new ArrayList<String>();
        var tagIds = new LinkedHashSet<Integer>();

        for (var cam : withPoses) {
            double weight = weight(cam);
            if (weight <= 0) {
                continue;
            }
            var pose = cam.fieldToRobot.get();
            totalWeight += weight;
            x += weight * pose.getX();
            y += weight * pose.getY();
            z += weight * pose.getZ();
            var q = pose.getRotation().getQuaternion();
            // Keep quaternion hemisphere consistent with the running average
            double dot = qw * q.getW() + qx * q.getX() + qy * q.getY() + qz * q.getZ();
            if (totalWeight > weight && dot < 0) {
                qw -= weight * q.getW();
                qx -= weight * q.getX();
                qy -= weight * q.getY();
                qz -= weight * q.getZ();
            } else {
                qw += weight * q.getW();
                qx += weight * q.getX();
                qy += weight * q.getY();
                qz += weight * q.getZ();
            }
            timestamp += weight * cam.captureTimestampNanos;
            reproj += weight * cam.reprojErr;
            cameras.add(cam.nickname);
            for (var tag : cam.tags) {
                tagIds.add(tag.id);
            }
        }

        if (totalWeight <= 0) {
            return List.of();
        }

        x /= totalWeight;
        y /= totalWeight;
        z /= totalWeight;
        timestamp /= totalWeight;
        reproj /= totalWeight;
        double qNorm = Math.sqrt(qw * qw + qx * qx + qy * qy + qz * qz);
        if (qNorm < 1e-9) {
            return List.of();
        }
        qw /= qNorm;
        qx /= qNorm;
        qy /= qNorm;
        qz /= qNorm;

        var fused =
                new Pose3d(new Translation3d(x, y, z), new Rotation3d(new Quaternion(qw, qx, qy, qz)));
        return List.of(
                new PoseEstimate(
                        NAME,
                        null,
                        fused,
                        Math.round(timestamp),
                        cameras,
                        new ArrayList<>(tagIds),
                        reproj,
                        (System.nanoTime() - start) / 1e6));
    }

    static double weight(CameraObservation cam) {
        if (cam.multiTag) {
            return cam.tagCount() / (1.0 + Math.max(0, cam.reprojErr));
        }
        double ambiguity = 1;
        double distance = 0;
        for (var tag : cam.tags) {
            if (tag.ambiguity <= ambiguity) {
                ambiguity = tag.ambiguity;
                distance = tag.distanceMeters;
            }
        }
        return (1.0 - clamp(ambiguity, 0, 1)) / (1.0 + Math.max(0, distance));
    }

    private static double clamp(double v, double lo, double hi) {
        return Math.max(lo, Math.min(hi, v));
    }
}
