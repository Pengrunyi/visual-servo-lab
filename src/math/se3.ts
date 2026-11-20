/**
 * SE(3) / SO(3) helpers: rigid poses, Rodrigues' formula, and twist integration.
 *
 * A pose T = (R, t) maps coordinates from the body (camera) frame to the world
 * frame:  p_world = R p_camera + t.
 * Camera twists are body-frame quantities, so a step integrates as
 * T ← T · Exp(ξ Δt)  (right / body-fixed product).
 */

import {
  type Mat,
  type Vec,
  add,
  cloneMat,
  cloneVec,
  eye,
  invert,
  matMul,
  matVec,
  norm,
  scale,
  skew,
  zeros,
} from "./linalg";

export interface Pose {
  /** 3×3 rotation, camera → world. */
  R: Mat;
  /** Camera origin in world coordinates. */
  t: Vec;
}

export function identityPose(): Pose {
  return { R: eye(3), t: [0, 0, 0] };
}

export function pose(R: Mat, t: Vec): Pose {
  return { R: cloneMat(R), t: cloneVec(t) };
}

export function copyPose(p: Pose): Pose {
  return pose(p.R, p.t);
}

/** p_world = R p_cam + t */
export function transformPoint(p: Pose, q: Vec): Vec {
  return add(matVec(p.R, q), p.t);
}

/** p_cam = Rᵀ (p_world − t) */
export function inverseTransformPoint(p: Pose, qWorld: Vec): Vec {
  const d = [qWorld[0] - p.t[0], qWorld[1] - p.t[1], qWorld[2] - p.t[2]];
  const Rt = [
    [p.R[0][0], p.R[1][0], p.R[2][0]],
    [p.R[0][1], p.R[1][1], p.R[2][1]],
    [p.R[0][2], p.R[1][2], p.R[2][2]],
  ];
  return matVec(Rt, d);
}

export function invertPose(p: Pose): Pose {
  const Rt = [
    [p.R[0][0], p.R[1][0], p.R[2][0]],
    [p.R[0][1], p.R[1][1], p.R[2][1]],
    [p.R[0][2], p.R[1][2], p.R[2][2]],
  ];
  return { R: Rt, t: scale(matVec(Rt, p.t), -1) };
}

/** T_ac = T_ab · T_bc */
export function composePose(a: Pose, b: Pose): Pose {
  return {
    R: matMul(a.R, b.R),
    t: add(matVec(a.R, b.t), a.t),
  };
}

/**
 * Relative pose of `current` expressed in the `desired` frame:
 * T_{c* c} = T_{c* w} · T_{w c} = T_desired⁻¹ · T_current.
 */
export function relativePose(desired: Pose, current: Pose): Pose {
  return composePose(invertPose(desired), current);
}

/** Rotation about a coordinate axis by `angle` radians. */
export function rotX(angle: number): Mat {
  const c = Math.cos(angle);
  const s = Math.sin(angle);
  return [
    [1, 0, 0],
    [0, c, -s],
    [0, s, c],
  ];
}

export function rotY(angle: number): Mat {
  const c = Math.cos(angle);
  const s = Math.sin(angle);
  return [
    [c, 0, s],
    [0, 1, 0],
    [-s, 0, c],
  ];
}

export function rotZ(angle: number): Mat {
  const c = Math.cos(angle);
  const s = Math.sin(angle);
  return [
    [c, -s, 0],
    [s, c, 0],
    [0, 0, 1],
  ];
}

/** Compose R = Rz · Ry · Rx (intrinsic XYZ, or extrinsic ZYX). */
export function rotationFromEulerXyz(rx: number, ry: number, rz: number): Mat {
  return matMul(rotZ(rz), matMul(rotY(ry), rotX(rx)));
}

export function poseFromEuler(t: Vec, rx: number, ry: number, rz: number): Pose {
  return { R: rotationFromEulerXyz(rx, ry, rz), t: cloneVec(t) };
}

/**
 * Rodrigues' formula: Exp: so(3) → SO(3).
 * R = I + (sin θ / θ) [ω]_× + ((1 − cos θ) / θ²) [ω]_×²,  θ = ‖ω‖.
 */
export function expSO3(omega: Vec): Mat {
  const theta = norm(omega);
  const I = eye(3);
  const K = skew(omega);
  if (theta < 1e-12) {
    // I + [ω]_× + ½ [ω]_×²
    const K2 = matMul(K, K);
    return [
      [1 + K[0][0] + 0.5 * K2[0][0], K[0][1] + 0.5 * K2[0][1], K[0][2] + 0.5 * K2[0][2]],
      [K[1][0] + 0.5 * K2[1][0], 1 + K[1][1] + 0.5 * K2[1][1], K[1][2] + 0.5 * K2[1][2]],
      [K[2][0] + 0.5 * K2[2][0], K[2][1] + 0.5 * K2[2][1], 1 + K[2][2] + 0.5 * K2[2][2]],
    ];
  }
  const a = Math.sin(theta) / theta;
  const b = (1 - Math.cos(theta)) / (theta * theta);
  const K2 = matMul(K, K);
  const R = eye(3);
  for (let i = 0; i < 3; i++) {
    for (let j = 0; j < 3; j++) {
      R[i][j] = I[i][j] + a * K[i][j] + b * K2[i][j];
    }
  }
  return R;
}

/**
 * Log: SO(3) → so(3). Returns the angle-axis vector θu.
 * Stable near 0 and near π.
 */
export function logSO3(R: Mat): Vec {
  const tr = R[0][0] + R[1][1] + R[2][2];
  const cosTheta = Math.min(1, Math.max(-1, (tr - 1) / 2));
  const theta = Math.acos(cosTheta);

  if (theta < 1e-12) {
    return [0.5 * (R[2][1] - R[1][2]), 0.5 * (R[0][2] - R[2][0]), 0.5 * (R[1][0] - R[0][1])];
  }

  if (Math.abs(theta - Math.PI) < 1e-6) {
    // Near π, (R − Rᵀ) vanishes; use the diagonal of R to recover the axis.
    const axis = zeros(3);
    axis[0] = Math.sqrt(Math.max(0, (R[0][0] + 1) / 2));
    axis[1] = Math.sqrt(Math.max(0, (R[1][1] + 1) / 2));
    axis[2] = Math.sqrt(Math.max(0, (R[2][2] + 1) / 2));
    if (R[1][0] < 0) axis[1] = -axis[1];
    if (R[2][0] < 0) axis[2] = -axis[2];
    if (axis[0] < 1e-6 && R[2][1] < 0) axis[2] = -axis[2];
    const n = norm(axis);
    if (n < 1e-12) return [theta, 0, 0];
    return scale(axis, theta / n);
  }

  const factor = theta / (2 * Math.sin(theta));
  return [
    factor * (R[2][1] - R[1][2]),
    factor * (R[0][2] - R[2][0]),
    factor * (R[1][0] - R[0][1]),
  ];
}

/**
 * Left Jacobian of SO(3), also the V matrix in the SE(3) exponential:
 * V = I + ((1−cos θ)/θ²) [ω]_× + ((θ−sin θ)/θ³) [ω]_×².
 */
export function so3LeftJacobian(omega: Vec): Mat {
  const theta = norm(omega);
  const K = skew(omega);
  const I = eye(3);
  if (theta < 1e-12) {
    const K2 = matMul(K, K);
    const V = eye(3);
    for (let i = 0; i < 3; i++) {
      for (let j = 0; j < 3; j++) {
        V[i][j] = I[i][j] + 0.5 * K[i][j] + (1 / 6) * K2[i][j];
      }
    }
    return V;
  }
  const theta2 = theta * theta;
  const a = (1 - Math.cos(theta)) / theta2;
  const b = (theta - Math.sin(theta)) / (theta2 * theta);
  const K2 = matMul(K, K);
  const V = eye(3);
  for (let i = 0; i < 3; i++) {
    for (let j = 0; j < 3; j++) {
      V[i][j] = I[i][j] + a * K[i][j] + b * K2[i][j];
    }
  }
  return V;
}

/**
 * SE(3) exponential. Twist ξ = (ν, ω) ∈ ℝ⁶ is integrated for a unit time,
 * i.e. call with (ν Δt, ω Δt) to take a step of duration Δt.
 *
 *   R = Exp(ω)
 *   t = V ν
 */
export function expSE3(nu: Vec, omega: Vec): Pose {
  return { R: expSO3(omega), t: matVec(so3LeftJacobian(omega), nu) };
}

/**
 * Integrate a body-frame camera twist ξ = (v, ω) over dt:
 * T ← T · Exp(ξ dt).
 */
export function integratePose(p: Pose, twist: Vec, dt: number): Pose {
  if (twist.length !== 6) {
    throw new Error("twist must be a 6-vector (v, ω)");
  }
  const nu = scale(twist.slice(0, 3), dt);
  const omega = scale(twist.slice(3, 6), dt);
  return composePose(p, expSE3(nu, omega));
}

/**
 * Rotation interaction matrix L_{θu} used in classical PBVS
 * (Chaumette & Hutchinson, IEEE RAM 2006, eq. for L_{θu}):
 *
 *   L_{θu} = I − (θ/2) [u]_× + (1 − sinc(θ) / sinc²(θ/2)) [u]_×²
 *
 * with sinc(x) = sin(x)/x and the convention sinc(0) = 1.
 * Returns the 3×3 matrix; at θ = 0 it is I.
 */
export function rotationInteractionMatrix(thetaU: Vec): Mat {
  const theta = norm(thetaU);
  const I = eye(3);
  if (theta < 1e-12) return I;

  const u = scale(thetaU, 1 / theta);
  const Ku = skew(u);
  const Ku2 = matMul(Ku, Ku);
  const sinc = (x: number) => (Math.abs(x) < 1e-12 ? 1 : Math.sin(x) / x);
  const coeff = 1 - sinc(theta) / (sinc(theta / 2) * sinc(theta / 2));

  const L = eye(3);
  for (let i = 0; i < 3; i++) {
    for (let j = 0; j < 3; j++) {
      L[i][j] = I[i][j] - (theta / 2) * Ku[i][j] + coeff * Ku2[i][j];
    }
  }
  return L;
}

export function invertRotationInteraction(thetaU: Vec): Mat {
  return invert(rotationInteractionMatrix(thetaU));
}

/** Frobenius-ish rotation error: ‖log(R)‖ = |θ|. */
export function rotationAngle(R: Mat): number {
  return norm(logSO3(R));
}
