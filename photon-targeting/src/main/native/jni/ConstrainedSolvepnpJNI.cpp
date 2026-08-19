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

#include <iostream>
#include <span>
#include <vector>

#include <Eigen/Core>
#include <fmt/core.h>
#include <fmt/ranges.h>

#include "org_photonvision_jni_ConstrainedSolvepnpJni.h"
#include "photon/constrained_solvepnp/wrap/casadi_wrapper.h"

std::vector<double> convertJDoubleArray(JNIEnv* env, jdoubleArray array) {
  jsize length = env->GetArrayLength(array);
  std::vector<double> result(length);
  env->GetDoubleArrayRegion(array, 0, length, result.data());
  return result;
}

jdoubleArray createJDoubleArray(JNIEnv* env, const std::span<double> vec) {
  jdoubleArray array = env->NewDoubleArray(vec.size());
  env->SetDoubleArrayRegion(array, 0, vec.size(), vec.data());
  return array;
}

extern "C" {
/*
 * Class:     org_photonvision_jni_ConstrainedSolvepnpJni_do
 * Method:    1optimization
 * Signature: (ZI[D[D[D[D[DDD)[D
 */
JNIEXPORT jdoubleArray JNICALL
Java_org_photonvision_jni_ConstrainedSolvepnpJni_do_1optimization
  (JNIEnv* env, jclass, jboolean headingFree, jint nTags,
   jdoubleArray cameraCal, jdoubleArray robot2camera, jdoubleArray xGuess,
   jdoubleArray field2points, jdoubleArray pointObservations, jdouble gyro_θ,
   jdouble gyro_error_scale_fac)
{
  auto cameraCalVec = convertJDoubleArray(env, cameraCal);
  auto robot2cameraVec = convertJDoubleArray(env, robot2camera);
  auto xGuessVec = convertJDoubleArray(env, xGuess);
  auto field2pointsVec = convertJDoubleArray(env, field2points);
  auto pointObservationsVec = convertJDoubleArray(env, pointObservations);

  constrained_solvepnp::CameraCalibration cameraCal_{
      cameraCalVec[0],
      cameraCalVec[1],
      cameraCalVec[2],
      cameraCalVec[3],
  };
  Eigen::Map<Eigen::Matrix<double, 4, 4, Eigen::RowMajor>> robot2cameraMat(
      robot2cameraVec.data());
  Eigen::Map<Eigen::Matrix<double, 3, 1>> xGuessMat(xGuessVec.data());
  Eigen::Map<Eigen::Matrix<double, 4, Eigen::Dynamic, Eigen::RowMajor>>
      field2pointsMat(field2pointsVec.data(), 4, field2pointsVec.size() / 4);
  Eigen::Map<Eigen::Matrix<double, 2, Eigen::Dynamic, Eigen::RowMajor>>
      pointObservationsMat(pointObservationsVec.data(), 2,
                           pointObservationsVec.size() / 2);

#if 0
  fmt::println("======================================================");
  fmt::println("Got robot2camera raw {}", robot2cameraVec);
  fmt::println("Camera cal {} {} {} {}", cameraCal_.fx, cameraCal_.fy,
               cameraCal_.cx, cameraCal_.cy);
  fmt::println("{} tags", nTags);
  std::cout << "robot2camera:\n" << robot2cameraMat << std::endl;
  std::cout << "x guess:\n" << xGuessMat << std::endl;
  std::cout << "field2pt:\n" << field2pointsMat << std::endl;
  std::cout << "observations:\n" << pointObservationsMat << std::endl;
#endif

  wpi::expected<constrained_solvepnp::RobotStateMat, slp::ExitStatus> result =
      constrained_solvepnp::do_optimization(
          headingFree, nTags, cameraCal_, robot2cameraMat, xGuessMat,
          field2pointsMat, pointObservationsMat, gyro_θ, gyro_error_scale_fac);

  if (result) {
    std::vector<double> resultVec{result->data(),
                                  result->data() + result->size()};
    return createJDoubleArray(env, resultVec);
  } else {
    return nullptr;
  }
}

/*
 * Class:     org_photonvision_jni_ConstrainedSolvepnpJni_do_1optimization
 * Method:    1multi
 * Signature: (Z[I[D[D[D[D[DDD)[D
 */
JNIEXPORT jdoubleArray JNICALL
Java_org_photonvision_jni_ConstrainedSolvepnpJni_do_1optimization_1multi
  (JNIEnv* env, jclass, jboolean headingFree, jintArray nTagsPerCamera,
   jdoubleArray cameraCals, jdoubleArray robot2cameras, jdoubleArray xGuess,
   jdoubleArray field2points, jdoubleArray pointObservations, jdouble gyro_θ,
   jdouble gyro_error_scale_fac)
{
  jsize nCameras = env->GetArrayLength(nTagsPerCamera);
  if (nCameras <= 0) {
    return nullptr;
  }

  std::vector<int> nTags(nCameras);
  {
    std::vector<jint> tmp(nCameras);
    env->GetIntArrayRegion(nTagsPerCamera, 0, nCameras, tmp.data());
    for (jsize i = 0; i < nCameras; i++) {
      nTags[i] = tmp[i];
    }
  }

  auto cameraCalVec = convertJDoubleArray(env, cameraCals);
  auto robot2cameraVec = convertJDoubleArray(env, robot2cameras);
  auto xGuessVec = convertJDoubleArray(env, xGuess);
  auto field2pointsVec = convertJDoubleArray(env, field2points);
  auto pointObservationsVec = convertJDoubleArray(env, pointObservations);

  if (cameraCalVec.size() < static_cast<size_t>(nCameras) * 4 ||
      robot2cameraVec.size() < static_cast<size_t>(nCameras) * 16 ||
      xGuessVec.size() < 3) {
    return nullptr;
  }

  std::vector<constrained_solvepnp::CameraCalibration> cals(nCameras);
  std::vector<Eigen::Matrix<double, 4, 4, Eigen::ColMajor>> r2c(nCameras);
  std::vector<Eigen::Matrix<double, 4, Eigen::Dynamic, Eigen::ColMajor>> field(
      nCameras);
  std::vector<Eigen::Matrix<double, 2, Eigen::Dynamic, Eigen::ColMajor>> obs(
      nCameras);

  size_t fieldOffset = 0;
  size_t obsOffset = 0;
  for (jsize i = 0; i < nCameras; i++) {
    int tags = nTags[i];
    if (tags < 0) {
      return nullptr;
    }
    int nPts = tags * 4;
    if (fieldOffset + static_cast<size_t>(4 * nPts) > field2pointsVec.size() ||
        obsOffset + static_cast<size_t>(2 * nPts) >
            pointObservationsVec.size()) {
      return nullptr;
    }
    cals[i] = constrained_solvepnp::CameraCalibration{
        cameraCalVec[i * 4 + 0],
        cameraCalVec[i * 4 + 1],
        cameraCalVec[i * 4 + 2],
        cameraCalVec[i * 4 + 3],
    };
    Eigen::Map<Eigen::Matrix<double, 4, 4, Eigen::RowMajor>> r2cRow(
        robot2cameraVec.data() + i * 16);
    r2c[i] = r2cRow;

    Eigen::Map<Eigen::Matrix<double, 4, Eigen::Dynamic, Eigen::RowMajor>>
        fieldRow(field2pointsVec.data() + fieldOffset, 4, nPts);
    field[i] = fieldRow;
    fieldOffset += static_cast<size_t>(4 * nPts);

    Eigen::Map<Eigen::Matrix<double, 2, Eigen::Dynamic, Eigen::RowMajor>>
        obsRow(pointObservationsVec.data() + obsOffset, 2, nPts);
    obs[i] = obsRow;
    obsOffset += static_cast<size_t>(2 * nPts);
  }

  Eigen::Map<Eigen::Matrix<double, 3, 1>> xGuessMat(xGuessVec.data());

  wpi::expected<constrained_solvepnp::RobotStateMat, slp::ExitStatus> result =
      constrained_solvepnp::do_optimization_multi(
          headingFree, static_cast<int>(nCameras), nTags.data(), cals.data(),
          r2c.data(), xGuessMat, field.data(), obs.data(), gyro_θ,
          gyro_error_scale_fac);

  if (result) {
    std::vector<double> resultVec{result->data(),
                                  result->data() + result->size()};
    return createJDoubleArray(env, resultVec);
  } else {
    return nullptr;
  }
}
}  // extern "C"
