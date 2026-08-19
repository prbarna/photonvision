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

package org.photonvision.common.configuration;

import com.fasterxml.jackson.annotation.JsonCreator;
import com.fasterxml.jackson.annotation.JsonProperty;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;

/** Robot origin to camera optical frame, stored as XYZ meters and RPY radians. */
public class RobotToCameraTransform {
    public double x;
    public double y;
    public double z;
    public double roll;
    public double pitch;
    public double yaw;

    public RobotToCameraTransform() {}

    @JsonCreator
    public RobotToCameraTransform(
            @JsonProperty("x") double x,
            @JsonProperty("y") double y,
            @JsonProperty("z") double z,
            @JsonProperty("roll") double roll,
            @JsonProperty("pitch") double pitch,
            @JsonProperty("yaw") double yaw) {
        this.x = x;
        this.y = y;
        this.z = z;
        this.roll = roll;
        this.pitch = pitch;
        this.yaw = yaw;
    }

    public Transform3d toTransform3d() {
        return new Transform3d(new Translation3d(x, y, z), new Rotation3d(roll, pitch, yaw));
    }

    public static RobotToCameraTransform fromTransform3d(Transform3d transform) {
        var t = transform.getTranslation();
        var r = transform.getRotation();
        return new RobotToCameraTransform(t.getX(), t.getY(), t.getZ(), r.getX(), r.getY(), r.getZ());
    }
}
