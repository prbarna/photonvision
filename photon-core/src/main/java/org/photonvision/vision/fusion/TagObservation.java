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

import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import java.util.List;
import java.util.Optional;

/** Copied tag observation used by pose comparison streams. */
public class TagObservation {
    public final int id;
    public final List<Translation3d> fieldVertices;
    public final double[] cornerX;
    public final double[] cornerY;
    public final Optional<Transform3d> cameraToTarget;
    public final double ambiguity;
    public final double distanceMeters;

    public TagObservation(
            int id,
            List<Translation3d> fieldVertices,
            double[] cornerX,
            double[] cornerY,
            Optional<Transform3d> cameraToTarget,
            double ambiguity,
            double distanceMeters) {
        this.id = id;
        this.fieldVertices = fieldVertices;
        this.cornerX = cornerX;
        this.cornerY = cornerY;
        this.cameraToTarget = cameraToTarget;
        this.ambiguity = ambiguity;
        this.distanceMeters = distanceMeters;
    }
}
