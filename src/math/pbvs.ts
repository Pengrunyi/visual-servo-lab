/**
 * Position-based visual servoing (PBVS).
 *
 * We regulate the pose error between the current camera frame {c} and the
 * desired camera frame {c*}. Let
 *
 *   T_{c* c} = T_{c* w} T_{w c} = T_desired⁻¹ T_current
 *
 * with translation t = ^{c*}t_c and rotation ^{c*}R_c whose angle-axis
 * vector is θu = Log(^{c*}R_c). The camera twist expressed in the *current*
 * camera frame is (Chaumette & Hutchinson, IEEE RAM 2006, PBVS form):
 *
 *   v_c = −λ  ^{c}R_{c*}  t     = −λ (^{c*}R_c)ᵀ t
 *   ω_c = −λ  L_{θu}⁻¹ θu
 *
 * L_{θu} is the rotation interaction matrix given in the paper; at small
 * angles L_{θu} ≈ I and the law reduces to ω_c = −λ θu. Because the object
 * is static, this is equivalent to regulating the object's pose to its
 * desired pose in the camera frame.
 */

import { type Vec, matVec, scale } from "./linalg";
import {
  type Pose,
  invertRotationInteraction,
  logSO3,
  relativePose,
} from "./se3";

export interface PbvsInput {
  current: Pose;
  desired: Pose;
  /** Gain λ > 0. */
  lambda: number;
}

export interface PbvsResult {
  /** Translation error ^{c*}t_c (current camera origin in the desired frame). */
  translationError: Vec;
  /** Rotation error θu = Log(^{c*}R_c). */
  rotationError: Vec;
  /** Camera twist v_c = (v, ω) in the current camera frame. */
  twist: Vec;
}

/** One PBVS control step: compute the camera twist, do not integrate. */
export function pbvsStep(input: PbvsInput): PbvsResult {
  const { current, desired, lambda } = input;
  const rel = relativePose(desired, current); // T_{c* c}
  const translationError = rel.t;
  const rotationError = logSO3(rel.R);

  // v_c = −λ Rᵀ t, with R = ^{c*}R_c, so Rᵀ = ^{c}R_{c*}.
  const Rt = [
    [rel.R[0][0], rel.R[1][0], rel.R[2][0]],
    [rel.R[0][1], rel.R[1][1], rel.R[2][1]],
    [rel.R[0][2], rel.R[1][2], rel.R[2][2]],
  ];
  const v = scale(matVec(Rt, translationError), -lambda);

  // ω_c = −λ L_{θu}⁻¹ θu
  const Linv = invertRotationInteraction(rotationError);
  const omega = scale(matVec(Linv, rotationError), -lambda);

  return {
    translationError,
    rotationError,
    twist: [v[0], v[1], v[2], omega[0], omega[1], omega[2]],
  };
}

export function poseTranslationErrorNorm(err: Vec): number {
  return Math.sqrt(err[0] * err[0] + err[1] * err[1] + err[2] * err[2]);
}

export function poseRotationErrorNorm(err: Vec): number {
  return Math.sqrt(err[0] * err[0] + err[1] * err[1] + err[2] * err[2]);
}
