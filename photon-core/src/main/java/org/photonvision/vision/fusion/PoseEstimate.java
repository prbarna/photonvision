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
import java.util.List;

/** One pose estimate from a comparison stream. */
public class PoseEstimate {
    public final String method;
    public final String camera;
    public final Pose3d fieldToRobot;
    public final long timestampNanos;
    public final List<String> camerasUsed;
    public final List<Integer> fiducialIdsUsed;
    public final double reprojErr;
    public final double computeMs;

    public PoseEstimate(
            String method,
            String camera,
            Pose3d fieldToRobot,
            long timestampNanos,
            List<String> camerasUsed,
            List<Integer> fiducialIdsUsed,
            double reprojErr,
            double computeMs) {
        this.method = method;
        this.camera = camera;
        this.fieldToRobot = fieldToRobot;
        this.timestampNanos = timestampNanos;
        this.camerasUsed = camerasUsed;
        this.fiducialIdsUsed = fiducialIdsUsed;
        this.reprojErr = reprojErr;
        this.computeMs = computeMs;
    }
}
