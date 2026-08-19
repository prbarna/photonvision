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
import java.util.List;

/** Per-camera field-to-robot poses (today's PhotonPoseEstimator behavior). */
public class BaselinePoseStream implements PoseEstimatorStream {
    public static final String NAME = "baseline";

    @Override
    public String name() {
        return NAME;
    }

    @Override
    public List<PoseEstimate> estimate(ObservationWindow window) {
        var out = new ArrayList<PoseEstimate>();
        for (var cam : window.cameras) {
            if (cam.fieldToRobot.isEmpty()) {
                continue;
            }
            var tagIds = new ArrayList<Integer>();
            for (var tag : cam.tags) {
                tagIds.add(tag.id);
            }
            out.add(
                    new PoseEstimate(
                            NAME,
                            cam.nickname,
                            cam.fieldToRobot.get(),
                            cam.captureTimestampNanos,
                            List.of(cam.nickname),
                            tagIds,
                            cam.reprojErr,
                            0));
        }
        return out;
    }
}
