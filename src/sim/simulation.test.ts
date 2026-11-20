import { describe, expect, it } from "vitest";
import { VisualServoSim } from "./simulation";

function runUntil(sim: VisualServoSim, seconds: number): void {
  const steps = Math.ceil(seconds / sim.config.dt);
  for (let i = 0; i < steps; i++) {
    const ok = sim.step();
    if (!ok) break;
    if (sim.snapshot().converged) break;
  }
}

describe("default scene convergence", () => {
  it("IBVS converges from the default initial pose in a few seconds", () => {
    const sim = new VisualServoSim("ibvs", "translate-rotate", { lambda: 1.5 });
    const e0 = sim.snapshot().error;
    runUntil(sim, 6);
    const snap = sim.snapshot();
    expect(snap.valid).toBe(true);
    expect(snap.error).toBeLessThan(0.15 * e0);
    expect(snap.translationError).toBeLessThan(0.03);
    expect(snap.rotationError).toBeLessThan(0.08);
  });

  it("PBVS converges from the same initial pose in a few seconds", () => {
    const sim = new VisualServoSim("pbvs", "translate-rotate", { lambda: 1.5 });
    const e0 = sim.snapshot().translationError + sim.snapshot().rotationError;
    runUntil(sim, 6);
    const snap = sim.snapshot();
    expect(snap.valid).toBe(true);
    expect(snap.translationError + snap.rotationError).toBeLessThan(0.15 * e0);
    expect(snap.translationError).toBeLessThan(0.03);
    expect(snap.rotationError).toBeLessThan(0.05);
  });

  it("IBVS distance first grows on the large-rotation (retreat) preset", () => {
    const sim = new VisualServoSim("ibvs", "large-rotation", { lambda: 1.5 });
    const d0 = sim.snapshot().distance;
    let maxD = d0;
    for (let i = 0; i < 90; i++) {
      sim.step();
      maxD = Math.max(maxD, sim.snapshot().distance);
    }
    expect(maxD).toBeGreaterThan(d0 + 0.02);
  });
});
