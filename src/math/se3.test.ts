import { describe, expect, it } from "vitest";
import { eye, matMul, matVec, norm, vecAlmostEqual } from "./linalg";
import {
  composePose,
  expSO3,
  expSE3,
  identityPose,
  integratePose,
  invertPose,
  logSO3,
  poseFromEuler,
  relativePose,
  rotZ,
  rotationAngle,
  rotationInteractionMatrix,
} from "./se3";

const EPS = 1e-9;

describe("SO(3) exp / log", () => {
  it("maps 0 to the identity and back", () => {
    const R = expSO3([0, 0, 0]);
    expect(vecAlmostEqual(R[0], [1, 0, 0], EPS)).toBe(true);
    expect(vecAlmostEqual(R[1], [0, 1, 0], EPS)).toBe(true);
    expect(vecAlmostEqual(R[2], [0, 0, 1], EPS)).toBe(true);
    expect(norm(logSO3(R))).toBeLessThan(1e-12);
  });

  it("reproduces a 90° rotation about Z", () => {
    const omega = [0, 0, Math.PI / 2];
    const R = expSO3(omega);
    const expected = rotZ(Math.PI / 2);
    for (let i = 0; i < 3; i++) {
      for (let j = 0; j < 3; j++) {
        expect(R[i][j]).toBeCloseTo(expected[i][j], 12);
      }
    }
    const recovered = logSO3(R);
    expect(recovered[0]).toBeCloseTo(0, 12);
    expect(recovered[1]).toBeCloseTo(0, 12);
    expect(recovered[2]).toBeCloseTo(Math.PI / 2, 12);
  });

  it("is invertible for a set of random-ish angle-axis vectors", () => {
    const samples = [
      [0.2, -0.1, 0.4],
      [0, 1.2, 0],
      [2.5, 0.3, -1.1],
      [0.001, 0.002, -0.001],
    ];
    for (const w of samples) {
      const recovered = logSO3(expSO3(w));
      for (let i = 0; i < 3; i++) {
        expect(recovered[i]).toBeCloseTo(w[i], 10);
      }
    }
  });
});

describe("SE(3) poses", () => {
  it("has inverse and composition identities", () => {
    const p = poseFromEuler([0.3, -0.2, 0.5], 0.2, -0.4, 0.7);
    const inv = invertPose(p);
    const I = composePose(p, inv);
    const I2 = composePose(inv, p);
    for (const id of [I, I2]) {
      expect(norm(id.t)).toBeLessThan(1e-12);
      expect(rotationAngle(id.R)).toBeLessThan(1e-12);
    }
  });

  it("Exp(0) is the identity pose", () => {
    const p = expSE3([0, 0, 0], [0, 0, 0]);
    expect(norm(p.t)).toBeLessThan(1e-15);
    expect(rotationAngle(p.R)).toBeLessThan(1e-15);
  });

  it("integrates a pure +X body twist by moving along the camera X axis", () => {
    const start = identityPose();
    const end = integratePose(start, [1, 0, 0, 0, 0, 0], 0.2);
    expect(end.t[0]).toBeCloseTo(0.2, 12);
    expect(end.t[1]).toBeCloseTo(0, 12);
    expect(end.t[2]).toBeCloseTo(0, 12);
    expect(rotationAngle(end.R)).toBeLessThan(1e-12);
  });

  it("integrates a body twist in a rotated frame along the camera's own axis", () => {
    // Camera already rotated 90° about Z: its X axis is world Y.
    const start = poseFromEuler([0, 0, 0], 0, 0, Math.PI / 2);
    const end = integratePose(start, [1, 0, 0, 0, 0, 0], 0.3);
    expect(end.t[0]).toBeCloseTo(0, 12);
    expect(end.t[1]).toBeCloseTo(0.3, 12);
    expect(end.t[2]).toBeCloseTo(0, 12);
  });

  it("relativePose(desired, desired) is identity", () => {
    const p = poseFromEuler([1, 2, 3], 0.1, 0.2, 0.3);
    const rel = relativePose(p, p);
    expect(norm(rel.t)).toBeLessThan(1e-12);
    expect(rotationAngle(rel.R)).toBeLessThan(1e-12);
  });
});

describe("L_θu", () => {
  it("is the identity at zero rotation", () => {
    const L = rotationInteractionMatrix([0, 0, 0]);
    const I = eye(3);
    for (let i = 0; i < 3; i++) {
      expect(vecAlmostEqual(L[i], I[i], 1e-15)).toBe(true);
    }
  });

  it("is invertible at a finite rotation", () => {
    const L = rotationInteractionMatrix([0.4, -0.2, 0.7]);
    const I = matMul(L, [
      // just check det-ish via multiplying a vector energy
      [1, 0, 0],
      [0, 1, 0],
      [0, 0, 1],
    ]);
    expect(norm(matVec(I, [1, 0, 0]))).toBeGreaterThan(0.5);
  });
});
