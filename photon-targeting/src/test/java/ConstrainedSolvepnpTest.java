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

import static org.junit.jupiter.api.Assertions.assertNotNull;
import static org.junit.jupiter.api.Assertions.fail;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.MatBuilder;
import edu.wpi.first.math.Nat;
import edu.wpi.first.util.RuntimeLoader;
import java.io.IOException;
import org.junit.jupiter.api.AfterAll;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.Test;
import org.photonvision.jni.ConstrainedSolvepnpJni;
import org.photonvision.jni.LibraryLoader;

public class ConstrainedSolvepnpTest {
    @BeforeAll
    public static void load() throws IOException {
        if (!LibraryLoader.loadWpiLibraries()) {
            fail();
        }
        RuntimeLoader.loadLibrary("photontargetingJNI");

        HAL.initialize(1000, 0);
    }

    @AfterAll
    public static void teardown() {
        HAL.shutdown();
    }

    @Test
    public void smoketest() {
        double[] cameraCal = {
            600, 600, 300, 150,
        };

        var field2points =
                MatBuilder.fill(
                                Nat.N4(),
                                Nat.N4(),
                                2.5,
                                0 - 0.08255,
                                0.5 - 0.08255,
                                1,
                                2.5,
                                0 - 0.08255,
                                0.5 + 0.08255,
                                1,
                                2.5,
                                0 + 0.08255,
                                0.5 + 0.08255,
                                1,
                                2.5,
                                0 + 0.08255,
                                0.5 - 0.08255,
                                1)
                        .transpose();

        var point_observations =
                MatBuilder.fill(Nat.N4(), Nat.N2(), 333, -17, 333, -83, 267, -83, 267, -17).transpose();

        // Camera with +x in world -y, +y in world -z, and +z in world +x
        // (IE, camera pointing straight along the +X axis facing forwards)
        var robot2camera =
                MatBuilder.fill(Nat.N4(), Nat.N4(), 0, 0, 1, 0, -1, 0, 0, 0, 0, -1, 0, 0, 0, 0, 0, 1);

        // Initial guess for optimization
        double[] x_guess = {0.2, 0.1, -.05};

        var ret =
                ConstrainedSolvepnpJni.do_optimization(
                        true,
                        1,
                        cameraCal,
                        robot2camera.getData(),
                        x_guess,
                        field2points.getData(),
                        point_observations.getData(),
                        0,
                        0);
        var retMulti =
                ConstrainedSolvepnpJni.do_optimization_multi(
                        true,
                        new int[] {1},
                        cameraCal,
                        robot2camera.getData(),
                        x_guess,
                        field2points.getData(),
                        point_observations.getData(),
                        0,
                        0);
        assertNotNull(retMulti);
        org.junit.jupiter.api.Assertions.assertArrayEquals(ret, retMulti, 1e-6);

        // Two cameras seeing the same tag: summed cost has the same minimizer.
        double[] fieldTwice = new double[field2points.getData().length * 2];
        System.arraycopy(field2points.getData(), 0, fieldTwice, 0, field2points.getData().length);
        System.arraycopy(
                field2points.getData(),
                0,
                fieldTwice,
                field2points.getData().length,
                field2points.getData().length);
        double[] obsTwice = new double[point_observations.getData().length * 2];
        System.arraycopy(
                point_observations.getData(), 0, obsTwice, 0, point_observations.getData().length);
        System.arraycopy(
                point_observations.getData(),
                0,
                obsTwice,
                point_observations.getData().length,
                point_observations.getData().length);
        double[] r2cTwice = new double[32];
        System.arraycopy(robot2camera.getData(), 0, r2cTwice, 0, 16);
        System.arraycopy(robot2camera.getData(), 0, r2cTwice, 16, 16);
        double[] calTwice = new double[8];
        System.arraycopy(cameraCal, 0, calTwice, 0, 4);
        System.arraycopy(cameraCal, 0, calTwice, 4, 4);

        var retTwoCam =
                ConstrainedSolvepnpJni.do_optimization_multi(
                        true, new int[] {1, 1}, calTwice, r2cTwice, x_guess, fieldTwice, obsTwice, 0, 0);
        assertNotNull(retTwoCam);
        org.junit.jupiter.api.Assertions.assertArrayEquals(ret, retTwoCam, 1e-4);
    }
}
