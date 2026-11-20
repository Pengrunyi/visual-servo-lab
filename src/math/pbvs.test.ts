import { describe, expect, it } from "vitest";
import { DEFAULT_DESIRED_POSE } from "./camera";
import { norm } from "./linalg";
import { pbvsStep } from "./pbvs";
import { integratePose, poseFromEuler, relativePose } from "./se3";

describe("PBVS control law", () => {
  it("gives ~0 error and ~0 twist at the desired pose", () => {
    const result = pbvsStep({
      current: DEFAULT_DESIRED_POSE,
      desired: DEFAULT_DESIRED_POSE,
      lambda: 2,
    });
    expect(norm(result.translationError)).toBeLessThan(1e-15);
    expect(norm(result.rotationError)).toBeLessThan(1e-15);
    expect(norm(result.twist)).toBeLessThan(1e-15);
  });

  it("responds to a small +X translation with a negative v_x", () => {
    const current = poseFromEuler([0.05, 0, 0], 0, 0, 0);
    const result = pbvsStep({
      current,
      desired: DEFAULT_DESIRED_POSE,
      lambda: 1,
    });
    expect(result.translationError[0]).toBeCloseTo(0.05, 12);
    expect(result.twist[0]).toBeCloseTo(-0.05, 12);
    expect(result.twist[1]).toBeCloseTo(0, 12);
    expect(result.twist[2]).toBeCloseTo(0, 12);
    expect(Math.abs(result.twist[3])).toBeLessThan(1e-12);
    expect(Math.abs(result.twist[4])).toBeLessThan(1e-12);
    expect(Math.abs(result.twist[5])).toBeLessThan(1e-12);
  });

  it("responds to a pure +Z rotation with a corrective −ω_z and almost no translation", () => {
    const current = poseFromEuler([0, 0, 0], 0, 0, 0.4);
    const result = pbvsStep({
      current,
      desired: DEFAULT_DESIRED_POSE,
      lambda: 1,
    });
    expect(Math.abs(result.twist[0])).toBeLessThan(1e-12);
    expect(Math.abs(result.twist[1])).toBeLessThan(1e-12);
    expect(Math.abs(result.twist[2])).toBeLessThan(1e-12);
    expect(result.twist[5]).toBeLessThan(0);
    expect(Math.abs(result.twist[5])).toBeGreaterThan(Math.abs(result.twist[3]));
    expect(Math.abs(result.twist[5])).toBeGreaterThan(Math.abs(result.twist[4]));
  });

  it("reduces the pose error after a short integration", () => {
    const start = poseFromEuler([0.12, -0.08, -0.07], 0.25, -0.2, 0.35);
    const e0 = pbvsStep({
      current: start,
      desired: DEFAULT_DESIRED_POSE,
      lambda: 1,
    });
    const err0 =
      norm(e0.translationError) + norm(e0.rotationError);
    let pose = start;
    for (let i = 0; i < 120; i++) {
      const r = pbvsStep({
        current: pose,
        desired: DEFAULT_DESIRED_POSE,
        lambda: 1.5,
      });
      pose = integratePose(pose, r.twist, 0.02);
    }
    const rel = relativePose(DEFAULT_DESIRED_POSE, pose);
    expect(norm(rel.t)).toBeLessThan(0.15 * err0);
    const eN = pbvsStep({
      current: pose,
      desired: DEFAULT_DESIRED_POSE,
      lambda: 1,
    });
    expect(norm(eN.translationError) + norm(eN.rotationError)).toBeLessThan(0.15 * err0);
  });
});
