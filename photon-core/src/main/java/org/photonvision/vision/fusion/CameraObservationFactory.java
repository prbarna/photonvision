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

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import java.util.ArrayList;
import java.util.Optional;
import org.photonvision.common.configuration.CameraConfiguration;
import org.photonvision.common.configuration.RobotToCameraTransform;
import org.photonvision.estimation.TargetModel;
import org.photonvision.targeting.MultiTargetPNPResult;
import org.photonvision.vision.calibration.CameraCalibrationCoefficients;
import org.photonvision.vision.pipeline.result.CVPipelineResult;
import org.photonvision.vision.target.TrackedTarget;

final class CameraObservationFactory {
    private CameraObservationFactory() {}

    static Optional<CameraObservation> fromResult(
            CameraConfiguration config, CVPipelineResult result, AprilTagFieldLayout layout) {
        if (config == null || result == null || !config.includeInFusion) {
            return Optional.empty();
        }

        var robotToCamera =
                (config.robotToCamera != null ? config.robotToCamera : new RobotToCameraTransform())
                        .toTransform3d();

        double fx = 0;
        double fy = 0;
        double cx = 0;
        double cy = 0;
        double[] dist = new double[5];
        CameraCalibrationCoefficients cal = null;
        if (result.inputAndOutputFrame != null
                && result.inputAndOutputFrame.frameStaticProperties != null) {
            cal = result.inputAndOutputFrame.frameStaticProperties.cameraCalibration;
        }
        if (cal != null && cal.getIntrinsicsArr() != null && cal.getIntrinsicsArr().length >= 6) {
            var k = cal.getIntrinsicsArr();
            fx = k[0];
            cx = k[2];
            fy = k[4];
            cy = k[5];
            if (cal.getDistCoeffsArr() != null) {
                dist = cal.getDistCoeffsArr();
            }
        }

        var tags = new ArrayList<TagObservation>();
        TrackedTarget bestSingle = null;
        for (var target : result.targets) {
            int id = target.getFiducialId();
            if (id < 0) {
                continue;
            }
            var tagPose = layout != null ? layout.getTagPose(id) : Optional.<Pose3d>empty();
            var corners = target.getTargetCorners();
            if (corners == null || corners.size() < 4 || tagPose.isEmpty()) {
                continue;
            }
            double[] cxArr = new double[4];
            double[] cyArr = new double[4];
            for (int i = 0; i < 4; i++) {
                cxArr[i] = corners.get(i).x;
                cyArr[i] = corners.get(i).y;
            }
            Optional<Transform3d> camToTgt = Optional.empty();
            var best = target.getBestCameraToTarget3d();
            if (best != null && !best.equals(Transform3d.kZero)) {
                camToTgt = Optional.of(best);
            }
            double distMeters = camToTgt.map(t -> t.getTranslation().getNorm()).orElse(0.0);
            tags.add(
                    new TagObservation(
                            id,
                            TargetModel.kAprilTag36h11.getFieldVertices(tagPose.get()),
                            cxArr,
                            cyArr,
                            camToTgt,
                            target.getPoseAmbiguity(),
                            distMeters));
            if (camToTgt.isPresent()
                    && (bestSingle == null || target.getPoseAmbiguity() < bestSingle.getPoseAmbiguity())) {
                bestSingle = target;
            }
        }

        Optional<Pose3d> fieldToRobot = Optional.empty();
        boolean multiTag = false;
        double reproj = 0;
        if (result.multiTagResult != null && result.multiTagResult.isPresent()) {
            MultiTargetPNPResult mt = result.multiTagResult.get();
            fieldToRobot =
                    Optional.of(Pose3d.kZero.plus(mt.estimatedPose.best).plus(robotToCamera.inverse()));
            multiTag = true;
            reproj = mt.estimatedPose.bestReprojErr;
        } else if (bestSingle != null && layout != null) {
            var tagPose = layout.getTagPose(bestSingle.getFiducialId());
            if (tagPose.isPresent()) {
                fieldToRobot =
                        Optional.of(
                                tagPose
                                        .get()
                                        .plus(bestSingle.getBestCameraToTarget3d().inverse())
                                        .plus(robotToCamera.inverse()));
                reproj = 0;
            }
        }

        if (tags.isEmpty() && fieldToRobot.isEmpty()) {
            return Optional.empty();
        }

        return Optional.of(
                new CameraObservation(
                        config.uniqueName,
                        config.nickname,
                        result.getImageCaptureTimestampNanos(),
                        robotToCamera,
                        fx,
                        fy,
                        cx,
                        cy,
                        dist,
                        fieldToRobot,
                        multiTag,
                        reproj,
                        tags));
    }
}
