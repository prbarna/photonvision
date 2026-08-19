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
import edu.wpi.first.math.geometry.Transform3d;
import java.util.List;
import java.util.Optional;

/** One camera's 3D observations at a capture timestamp. */
public class CameraObservation {
    public final String uniqueName;
    public final String nickname;
    public final long captureTimestampNanos;
    public final Transform3d robotToCamera;
    public final double fx;
    public final double fy;
    public final double cx;
    public final double cy;
    public final double[] distCoeffs;
    public final Optional<Pose3d> fieldToRobot;
    public final boolean multiTag;
    public final double reprojErr;
    public final List<TagObservation> tags;

    public CameraObservation(
            String uniqueName,
            String nickname,
            long captureTimestampNanos,
            Transform3d robotToCamera,
            double fx,
            double fy,
            double cx,
            double cy,
            double[] distCoeffs,
            Optional<Pose3d> fieldToRobot,
            boolean multiTag,
            double reprojErr,
            List<TagObservation> tags) {
        this.uniqueName = uniqueName;
        this.nickname = nickname;
        this.captureTimestampNanos = captureTimestampNanos;
        this.robotToCamera = robotToCamera;
        this.fx = fx;
        this.fy = fy;
        this.cx = cx;
        this.cy = cy;
        this.distCoeffs = distCoeffs;
        this.fieldToRobot = fieldToRobot;
        this.multiTag = multiTag;
        this.reprojErr = reprojErr;
        this.tags = tags;
    }

    public boolean hasPose() {
        return fieldToRobot.isPresent();
    }

    public int tagCount() {
        return tags.size();
    }
}
