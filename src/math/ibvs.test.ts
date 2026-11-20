import { describe, expect, it } from "vitest";
import {
  DEFAULT_DESIRED_POSE,
  DEFAULT_TARGET,
  desiredFeatures,
  projectPoints,
  featuresFromProjected,
  depthsFromProjected,
} from "./camera";
import { ibvsStep, pointInteractionMatrix, stackedInteractionMatrix } from "./ibvs";
import { integratePose, poseFromEuler } from "./se3";
import { norm } from "./linalg";

describe("interaction matrix", () => {
  it("is 2×6 for one point and 8×6 for four points", () => {
    const L1 = pointInteractionMatrix(0.1, -0.2, 0.5);
    expect(L1.length).toBe(2);
    expect(L1[0].length).toBe(6);

    const features = [0.2, 0.2, -0.2, 0.2, -0.2, -0.2, 0.2, -0.2];
    const depths = [0.6, 0.6, 0.6, 0.6];
    const L = stackedInteractionMatrix(features, depths);
    expect(L.length).toBe(8);
    expect(L[0].length).toBe(6);
  });

  it("matches the closed form at the principal point", () => {
    const Z = 0.8;
    const L = pointInteractionMatrix(0, 0, Z);
    // First row: [−1/Z, 0, 0, 0, −1, 0]
    expect(L[0][0]).toBeCloseTo(-1 / Z, 12);
    expect(L[0][1]).toBeCloseTo(0, 12);
    expect(L[0][2]).toBeCloseTo(0, 12);
    expect(L[0][3]).toBeCloseTo(0, 12);
    expect(L[0][4]).toBeCloseTo(-1, 12);
    expect(L[0][5]).toBeCloseTo(0, 12);
    // Second row: [0, −1/Z, 0, 1, 0, 0]
    expect(L[1][0]).toBeCloseTo(0, 12);
    expect(L[1][1]).toBeCloseTo(-1 / Z, 12);
    expect(L[1][2]).toBeCloseTo(0, 12);
    expect(L[1][3]).toBeCloseTo(1, 12);
    expect(L[1][4]).toBeCloseTo(0, 12);
    expect(L[1][5]).toBeCloseTo(0, 12);
  });
});

describe("IBVS control law", () => {
  const { sStar, zStar } = desiredFeatures(DEFAULT_TARGET, DEFAULT_DESIRED_POSE);

  it("gives ~0 error and ~0 twist at the desired pose", () => {
    const proj = projectPoints(DEFAULT_DESIRED_POSE, DEFAULT_TARGET);
    expect(proj).not.toBeNull();
    const result = ibvsStep({
      s: featuresFromProjected(proj!),
      sStar,
      depths: depthsFromProjected(proj!),
      lambda: 2,
    });
    expect(norm(result.error)).toBeLessThan(1e-12);
    expect(norm(result.twist)).toBeLessThan(1e-12);
    expect(result.L.length).toBe(8);
    expect(result.L[0].length).toBe(6);
  });

  it("responds to a small +X camera translation with a negative v_x", () => {
    // Camera moved to the right: points slide left, law should pull back −X.
    const current = poseFromEuler([0.04, 0, 0], 0, 0, 0);
    const proj = projectPoints(current, DEFAULT_TARGET);
    expect(proj).not.toBeNull();
    const result = ibvsStep({
      s: featuresFromProjected(proj!),
      sStar,
      depths: depthsFromProjected(proj!),
      lambda: 1,
    });
    expect(result.twist[0]).toBeLessThan(0);
    expect(Math.abs(result.twist[0])).toBeGreaterThan(Math.abs(result.twist[1]));
    expect(Math.abs(result.twist[0])).toBeGreaterThan(Math.abs(result.twist[2]));
  });

  it("responds to a small +Z retreat (camera closer to the target? farther behind) correctly", () => {
    // Camera at z = −0.05 is farther from a target at z = 0.6, so points
    // shrink. The law should move the camera forward (+Z) toward the origin.
    const farther = poseFromEuler([0, 0, -0.05], 0, 0, 0);
    const proj = projectPoints(farther, DEFAULT_TARGET);
    expect(proj).not.toBeNull();
    const result = ibvsStep({
      s: featuresFromProjected(proj!),
      sStar,
      depths: depthsFromProjected(proj!),
      lambda: 1,
    });
    expect(result.twist[2]).toBeGreaterThan(0);
  });

  it("reduces the feature error after a short integration", () => {
    const start = poseFromEuler([0.08, -0.05, -0.04], 0.08, -0.1, 0.05);
    const proj0 = projectPoints(start, DEFAULT_TARGET)!;
    const step = ibvsStep({
      s: featuresFromProjected(proj0),
      sStar,
      depths: zStar, // desired-depth approximation also works for a small offset
      lambda: 2,
    });
    const e0 = norm(step.error);
    let pose = start;
    for (let i = 0; i < 90; i++) {
      const p = projectPoints(pose, DEFAULT_TARGET)!;
      const r = ibvsStep({
        s: featuresFromProjected(p),
        sStar,
        depths: depthsFromProjected(p),
        lambda: 2,
      });
      pose = integratePose(pose, r.twist, 0.02);
    }
    const projN = projectPoints(pose, DEFAULT_TARGET)!;
    const eN = norm(
      ibvsStep({
        s: featuresFromProjected(projN),
        sStar,
        depths: depthsFromProjected(projN),
        lambda: 2,
      }).error,
    );
    expect(eN).toBeLessThan(e0 * 0.15);
  });
});
