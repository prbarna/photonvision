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

import java.util.ArrayList;
import java.util.List;

/** Cameras whose capture timestamps fall in the fusion window. */
public class ObservationWindow {
    public final long triggerTimestampNanos;
    public final List<CameraObservation> cameras;

    public ObservationWindow(long triggerTimestampNanos, List<CameraObservation> cameras) {
        this.triggerTimestampNanos = triggerTimestampNanos;
        this.cameras = List.copyOf(cameras);
    }

    public List<CameraObservation> withPoses() {
        var out = new ArrayList<CameraObservation>();
        for (var cam : cameras) {
            if (cam.hasPose()) {
                out.add(cam);
            }
        }
        return out;
    }
}
