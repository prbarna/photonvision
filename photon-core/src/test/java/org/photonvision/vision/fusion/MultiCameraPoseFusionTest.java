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

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import java.util.List;
import java.util.Optional;
import java.util.concurrent.TimeUnit;
import org.junit.jupiter.api.Test;
import org.photonvision.common.configuration.MultiCameraFusionConfig;

public class MultiCameraPoseFusionTest {
    private static CameraObservation cam(
            String name,
            long tNanos,
            Pose3d fieldToRobot,
            boolean multiTag,
            int nTags,
            double reproj,
            Transform3d robotToCamera) {
        var tags = new java.util.ArrayList<TagObservation>();
        for (int i = 0; i < nTags; i++) {
            tags.add(
                    new TagObservation(
                            i + 1,
                            List.of(),
                            new double[] {0, 1, 1, 0},
                            new double[] {0, 0, 1, 1},
                            Optional.empty(),
                            0.1,
                            1.0));
        }
        return new CameraObservation(
                name,
                name,
                tNanos,
                robotToCamera,
                600,
                600,
                320,
                240,
                new double[5],
                Optional.of(fieldToRobot),
                multiTag,
                reproj,
                tags);
    }

    private static CameraObservation withTagCount(
            String name, Pose3d pose, int nTags, double reproj) {
        var tags = new java.util.ArrayList<TagObservation>();
        for (int i = 0; i < nTags; i++) {
            tags.add(
                    new TagObservation(
                            i + 1,
                            List.of(),
                            new double[] {0, 1, 1, 0},
                            new double[] {0, 0, 1, 1},
                            Optional.empty(),
                            0.1,
                            1.0));
        }
        return new CameraObservation(
                name,
                name,
                0,
                Transform3d.kZero,
                600,
                600,
                320,
                240,
                new double[5],
                Optional.of(pose),
                nTags >= 2,
                reproj,
                tags);
    }

    @Test
    public void identicalPosesAverageToSame() {
        var pose = new Pose3d(1, 2, 0, new Rotation3d());
        var a = cam("a", 0, pose, true, 2, 0.1, Transform3d.kZero);
        var b = cam("b", 1_000_000, pose, true, 2, 0.1, Transform3d.kZero);
        var window = new ObservationWindow(0, List.of(a, b));
        var out = new WeightedAveragePoseStream(1).estimate(window);
        assertEquals(1, out.size());
        assertEquals(1.0, out.get(0).fieldToRobot.getX(), 1e-9);
        assertEquals(2.0, out.get(0).fieldToRobot.getY(), 1e-9);
    }

    @Test
    public void extrinsicsAreAppliedByCaller() {
        var robotToCamera = new Transform3d(new Translation3d(0.5, 0, 0), new Rotation3d());
        // field-to-camera is identity; field-to-robot = cameraToRobot = inverse(robotToCamera)
        var fieldToRobot = Pose3d.kZero.plus(robotToCamera.inverse());
        var a = cam("a", 0, fieldToRobot, true, 2, 0.1, robotToCamera);
        var window = new ObservationWindow(0, List.of(a));
        var out = new BaselinePoseStream().estimate(window);
        assertEquals(1, out.size());
        assertEquals(-0.5, out.get(0).fieldToRobot.getX(), 1e-9);
    }

    @Test
    public void farApartPosesAreNotInTheSameWindow() {
        var pose = new Pose3d(1, 0, 0, new Rotation3d());
        var a = cam("a", 0, pose, true, 2, 0.1, Transform3d.kZero);
        var b =
                cam(
                        "b",
                        TimeUnit.MILLISECONDS.toNanos(200),
                        new Pose3d(9, 0, 0, new Rotation3d()),
                        true,
                        2,
                        0.1,
                        Transform3d.kZero);
        var orch = new MultiCameraPoseOrchestrator();
        var fusion = new MultiCameraFusionConfig();
        fusion.maxDtMs = 50;
        fusion.jointPnp = false;
        fusion.writeCsv = false;
        fusion.enabled = true;
        orch.acceptObservation(a, fusion);
        orch.acceptObservation(b, fusion);
        var window = orch.window(b, fusion);
        assertEquals(1, window.cameras.size());
        assertEquals("b", window.cameras.get(0).uniqueName);
    }

    @Test
    public void emptyObservationRemovesCamera() {
        var pose = new Pose3d(1, 0, 0, new Rotation3d());
        var a = cam("a", 0, pose, true, 2, 0.1, Transform3d.kZero);
        var orch = new MultiCameraPoseOrchestrator();
        var fusion = new MultiCameraFusionConfig();
        fusion.jointPnp = false;
        fusion.writeCsv = false;
        fusion.enabled = true;
        orch.acceptObservation(a, fusion);
        var empty =
                new CameraObservation(
                        "a",
                        "a",
                        1,
                        Transform3d.kZero,
                        0,
                        0,
                        0,
                        0,
                        new double[0],
                        Optional.empty(),
                        false,
                        0,
                        List.of());
        orch.acceptObservation(empty, fusion);
        var other = cam("b", 1, pose, true, 2, 0.1, Transform3d.kZero);
        var window = orch.window(other, fusion);
        assertTrue(window.cameras.stream().noneMatch(c -> c.uniqueName.equals("a")));
    }

    @Test
    public void moreTagsDominateAverage() {
        var low = withTagCount("few", new Pose3d(0, 0, 0, new Rotation3d()), 2, 0.1);
        var high = withTagCount("many", new Pose3d(10, 0, 0, new Rotation3d()), 8, 0.1);
        var window = new ObservationWindow(0, List.of(low, high));
        var out = new WeightedAveragePoseStream(1).estimate(window);
        assertEquals(1, out.size());
        assertTrue(out.get(0).fieldToRobot.getX() > 5.0);
    }

    @Test
    public void orchestratorRunsBaselineAndWeightedWithoutJoint() {
        var pose = new Pose3d(1, 2, 0, new Rotation3d());
        var a = cam("a", 0, pose, true, 2, 0.1, Transform3d.kZero);
        var b = cam("b", 1_000_000, pose, true, 2, 0.1, Transform3d.kZero);
        var orch = new MultiCameraPoseOrchestrator();
        var fusion = new MultiCameraFusionConfig();
        fusion.enabled = true;
        fusion.jointPnp = false;
        fusion.writeCsv = false;
        fusion.baseline = true;
        fusion.weightedAverage = true;
        orch.acceptObservation(a, fusion);
        var results = orch.acceptObservation(b, fusion);
        assertTrue(results.stream().anyMatch(r -> r.method.equals("baseline")));
        assertTrue(results.stream().anyMatch(r -> r.method.equals("weightedAverage")));
        assertTrue(results.stream().noneMatch(r -> r.method.equals("jointPnp")));
        assertEquals(2, results.stream().filter(r -> r.method.equals("baseline")).count());
    }

    @Test
    public void closeTimestampsAreMerged() {
        var pose = new Pose3d(1, 0, 0, new Rotation3d());
        var a = cam("a", 0, pose, true, 2, 0.1, Transform3d.kZero);
        var b = cam("b", TimeUnit.MILLISECONDS.toNanos(10), pose, true, 2, 0.1, Transform3d.kZero);
        var orch = new MultiCameraPoseOrchestrator();
        var fusion = new MultiCameraFusionConfig();
        fusion.maxDtMs = 50;
        fusion.jointPnp = false;
        fusion.writeCsv = false;
        fusion.enabled = true;
        orch.acceptObservation(a, fusion);
        orch.acceptObservation(b, fusion);
        assertEquals(2, orch.window(b, fusion).cameras.size());
    }

    @Test
    public void streamExceptionDoesNotSilenceOthers() {
        var pose = new Pose3d(1, 2, 0, new Rotation3d());
        var a = cam("a", 0, pose, true, 2, 0.1, Transform3d.kZero);
        var window = new ObservationWindow(0, List.of(a));
        PoseEstimatorStream throwing =
                new PoseEstimatorStream() {
                    @Override
                    public String name() {
                        return "boom";
                    }

                    @Override
                    public List<PoseEstimate> estimate(ObservationWindow w) {
                        throw new RuntimeException("boom");
                    }
                };
        var results =
                MultiCameraPoseOrchestrator.runAll(
                        List.of(throwing, new BaselinePoseStream(), new WeightedAveragePoseStream(1)), window);
        assertTrue(results.stream().anyMatch(r -> r.method.equals("baseline")));
        assertTrue(results.stream().anyMatch(r -> r.method.equals("weightedAverage")));
        assertTrue(results.stream().noneMatch(r -> r.method.equals("boom")));
    }
}
