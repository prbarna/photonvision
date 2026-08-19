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

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.MatBuilder;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import java.util.ArrayList;
import java.util.LinkedHashSet;
import java.util.List;
import java.util.Optional;
import java.util.function.Supplier;
import org.photonvision.common.logging.LogGroup;
import org.photonvision.common.logging.Logger;
import org.photonvision.estimation.TargetModel;
import org.photonvision.estimation.VisionEstimation;
import org.photonvision.estimation.VisionEstimation.ConstrainedCameraObservations;
import org.photonvision.targeting.PhotonTrackedTarget;
import org.photonvision.targeting.PnpResult;
import org.photonvision.targeting.TargetCorner;

/** Joint constrained PnP over all cameras in the window. */
public class JointPnpPoseStream implements PoseEstimatorStream {
    public static final String NAME = "jointPnp";
    private static final Logger logger = new Logger(JointPnpPoseStream.class, LogGroup.VisionModule);

    private final int minCameras;
    private final Supplier<AprilTagFieldLayout> layoutSupplier;
    private final WeightedAveragePoseStream seedStream;

    public JointPnpPoseStream(int minCameras, Supplier<AprilTagFieldLayout> layoutSupplier) {
        this.minCameras = Math.max(1, minCameras);
        this.layoutSupplier = layoutSupplier;
        this.seedStream = new WeightedAveragePoseStream(1);
    }

    @Override
    public String name() {
        return NAME;
    }

    @Override
    public List<PoseEstimate> estimate(ObservationWindow window) {
        var withTags = new ArrayList<CameraObservation>();
        for (var cam : window.cameras) {
            if (!cam.tags.isEmpty() && cam.fx > 0) {
                withTags.add(cam);
            }
        }
        if (withTags.size() < minCameras) {
            return List.of();
        }

        var inputs = new ArrayList<ConstrainedCameraObservations>();
        var camerasUsed = new ArrayList<String>();
        var tagIds = new LinkedHashSet<Integer>();
        long timestampSum = 0;
        for (var cam : withTags) {
            var targets = new ArrayList<PhotonTrackedTarget>();
            int nTags = cam.tags.size();
            if (nTags > 10) {
                logger.warn("Camera " + cam.nickname + " has " + nTags + " tags; joint PnP using first 10");
                nTags = 10;
            }
            for (int ti = 0; ti < nTags; ti++) {
                var tag = cam.tags.get(ti);
                if (tag.cornerX.length < 4) {
                    continue;
                }
                var corners = new ArrayList<TargetCorner>();
                for (int i = 0; i < 4; i++) {
                    corners.add(new TargetCorner(tag.cornerX[i], tag.cornerY[i]));
                }
                targets.add(
                        new PhotonTrackedTarget(
                                0,
                                0,
                                0,
                                0,
                                tag.id,
                                -1,
                                -1,
                                tag.cameraToTarget.orElse(Transform3d.kZero),
                                Transform3d.kZero,
                                tag.ambiguity,
                                corners,
                                corners));
                tagIds.add(tag.id);
            }
            if (targets.isEmpty()) {
                continue;
            }
            double[] dist = cam.distCoeffs != null ? cam.distCoeffs : new double[5];
            double[] padded = new double[8];
            System.arraycopy(dist, 0, padded, 0, Math.min(dist.length, 8));
            var cameraMatrix =
                    MatBuilder.fill(Nat.N3(), Nat.N3(), cam.fx, 0, cam.cx, 0, cam.fy, cam.cy, 0, 0, 1);
            var distCoeffs =
                    MatBuilder.fill(
                            Nat.N8(), Nat.N1(), padded[0], padded[1], padded[2], padded[3], padded[4], padded[5],
                            padded[6], padded[7]);
            inputs.add(
                    new ConstrainedCameraObservations(cameraMatrix, distCoeffs, targets, cam.robotToCamera));
            camerasUsed.add(cam.nickname);
            timestampSum += cam.captureTimestampNanos;
        }
        if (inputs.size() < minCameras) {
            return List.of();
        }

        Pose3d seed = Pose3d.kZero;
        var seedEst = seedStream.estimate(window);
        if (!seedEst.isEmpty()) {
            seed = seedEst.get(0).fieldToRobot;
        } else {
            CameraObservation best = withTags.get(0);
            for (var cam : withTags) {
                if (cam.fieldToRobot.isPresent() && cam.tagCount() > best.tagCount()) {
                    best = cam;
                }
            }
            if (best.fieldToRobot.isPresent()) {
                seed = best.fieldToRobot.get();
            }
        }

        long start = System.nanoTime();
        Optional<PnpResult> solved;
        try {
            solved =
                    VisionEstimation.estimateRobotPoseMultiCameraConstrainedSolvepnp(
                            inputs, seed, layoutSupplier.get(), TargetModel.kAprilTag36h11);
        } catch (Throwable e) {
            return List.of();
        }
        double computeMs = (System.nanoTime() - start) / 1e6;
        if (solved.isEmpty()) {
            return List.of();
        }
        var pose = Pose3d.kZero.plus(solved.get().best);
        return List.of(
                new PoseEstimate(
                        NAME,
                        null,
                        pose,
                        timestampSum / withTags.size(),
                        camerasUsed,
                        new ArrayList<>(tagIds),
                        solved.get().bestReprojErr,
                        computeMs));
    }
}
