/**
 * Image-based visual servoing (IBVS) for four point features.
 *
 * Control law (Chaumette & Hutchinson, IEEE RAM 2006, Part I):
 *
 *   ė = L_s v_c
 *   v_c = −λ L_s⁺ e,    e = s − s*
 *
 * For a point with normalized coordinates (x, y) = (X/Z, Y/Z) and depth Z,
 * the 2×6 interaction matrix (image Jacobian) is
 *
 *   L(s, Z) = [ −1/Z    0    x/Z    x y    −(1+x²)    y ]
 *             [    0 −1/Z    y/Z  1+y²        −x y   −x ]
 *
 * Four points are stacked into an 8×6 matrix. The camera twist v_c is
 * expressed in the current camera frame. Depth Z may be the current (exact,
 * in this simulation) or the desired (constant approximation) value.
 */

import { type Mat, type Vec, matVec, pseudoInverse, scale, zerosMat } from "./linalg";

export type DepthMode = "current" | "desired";

/**
 * 2×6 interaction matrix of a single point in normalized coordinates.
 */
export function pointInteractionMatrix(x: number, y: number, z: number): Mat {
  if (z <= 1e-9) {
    throw new Error("Depth Z must be positive");
  }
  const invZ = 1 / z;
  return [
    [-invZ, 0, x * invZ, x * y, -(1 + x * x), y],
    [0, -invZ, y * invZ, 1 + y * y, -x * y, -x],
  ];
}

/**
 * Stacked 8×6 interaction matrix for four (or N) points.
 * `features` is [x1, y1, x2, y2, ...]; `depths` is [Z1, Z2, ...].
 */
export function stackedInteractionMatrix(features: Vec, depths: Vec): Mat {
  const n = depths.length;
  if (features.length !== 2 * n) {
    throw new Error("Feature vector length must be 2 × number of depths");
  }
  const L = zerosMat(2 * n, 6);
  for (let i = 0; i < n; i++) {
    const Li = pointInteractionMatrix(features[2 * i], features[2 * i + 1], depths[i]);
    L[2 * i] = Li[0];
    L[2 * i + 1] = Li[1];
  }
  return L;
}

export interface IbvsInput {
  /** Current image features s (8-vector for four points). */
  s: Vec;
  /** Desired image features s*. */
  sStar: Vec;
  /** Depths used to build L: current Z, or desired Z*, depending on mode. */
  depths: Vec;
  /** Gain λ > 0. */
  lambda: number;
}

export interface IbvsResult {
  /** Feature error e = s − s*. */
  error: Vec;
  /** 8×6 (for four points) interaction matrix. */
  L: Mat;
  /** Camera twist v_c = (v, ω) in the current camera frame. */
  twist: Vec;
}

/** One IBVS control step: compute the camera twist, do not integrate. */
export function ibvsStep(input: IbvsInput): IbvsResult {
  const { s, sStar, depths, lambda } = input;
  if (s.length !== sStar.length) {
    throw new Error("s and s* must have the same length");
  }
  const error = s.map((v, i) => v - sStar[i]);
  const L = stackedInteractionMatrix(s, depths);
  const pinv = pseudoInverse(L);
  const twist = scale(matVec(pinv, error), -lambda);
  return { error, L, twist };
}

/**
 * Pick the depth vector according to the approximation used in L.
 * - `current`: exact current depths (available in simulation).
 * - `desired`: constant Z* measured at the desired pose.
 */
export function depthsForMode(
  mode: DepthMode,
  current: Vec,
  desired: Vec,
): Vec {
  return mode === "desired" ? desired.slice() : current.slice();
}
