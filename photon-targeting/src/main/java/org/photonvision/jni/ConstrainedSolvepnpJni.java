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

package org.photonvision.jni;

public class ConstrainedSolvepnpJni {
    public static native double[] do_optimization(
            boolean heading_free,
            int nTags,
            double[] cameraCal,
            double[] robot2camera,
            double[] x_guess,
            double[] field2points,
            double[] point_observations,
            double gyro_θ,
            double gyro_error_scale_fac);

    /**
     * Joint constrained solvePNP over multiple cameras that share a robot pose [x, y, theta].
     *
     * <p>{@code cameraCals} is length {@code 4 * nCameras} (fx, fy, cx, cy). {@code robot2cameras} is
     * length {@code 16 * nCameras} (row-major 4x4). {@code field2points} / {@code pointObservations}
     * are concatenations of each camera's 4xn and 2xn matrices (row-major), with column counts {@code
     * 4 * nTagsPerCamera[i]}.
     */
    public static native double[] do_optimization_multi(
            boolean heading_free,
            int[] nTagsPerCamera,
            double[] cameraCals,
            double[] robot2cameras,
            double[] x_guess,
            double[] field2points,
            double[] pointObservations,
            double gyro_θ,
            double gyro_error_scale_fac);
}
