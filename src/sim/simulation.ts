/**
 * Discrete-time visual-servoing simulation.
 * Integrates the camera pose with the SE(3) exponential of the body twist.
 * A soft speed clamp keeps large-gain / large-error steps numerically safe
 * without changing the direction of the commanded screw.
 */

import {
  type Point2,
  type Point3,
  DEFAULT_DESIRED_POSE,
  DEFAULT_TARGET,
  cameraToTargetDistance,
  depthsFromProjected,
  desiredFeatures,
  featureErrorNorm,
  featuresFromProjected,
  projectPoints,
} from "../math/camera";
import { depthsForMode, ibvsStep, type DepthMode } from "../math/ibvs";
import { type Vec, norm } from "../math/linalg";
import { pbvsStep } from "../math/pbvs";
import { type Pose, copyPose, integratePose, relativePose, logSO3 } from "../math/se3";
import { DEFAULT_PRESET_ID, presetById } from "./presets";

export type Method = "ibvs" | "pbvs";

export interface Sample {
  t: number;
  featureError: number;
  translationError: number;
  rotationError: number;
  distance: number;
}

export interface SimSnapshot {
  pose: Pose;
  method: Method;
  features: Point2[];
  desiredFeatures: Point2[];
  featureTrajectory: Point2[][];
  error: number;
  translationError: number;
  rotationError: number;
  distance: number;
  twist: Vec;
  history: Sample[];
  time: number;
  converged: boolean;
  valid: boolean;
}

export interface SimConfig {
  lambda: number;
  depthMode: DepthMode;
  dt: number;
  /** Maximum translational speed (m/s) after clamping. */
  maxLinear: number;
  /** Maximum angular speed (rad/s) after clamping. */
  maxAngular: number;
}

export const DEFAULT_CONFIG: SimConfig = {
  lambda: 1.5,
  depthMode: "current",
  dt: 1 / 60,
  maxLinear: 0.6,
  maxAngular: 2.2,
};

const CONVERGE_FEATURE = 2e-3;
const CONVERGE_T = 2e-3;
const CONVERGE_R = 2e-3;

function clampTwist(twist: Vec, cfg: SimConfig): Vec {
  const v = twist.slice(0, 3);
  const w = twist.slice(3, 6);
  const vn = norm(v);
  const wn = norm(w);
  let s = 1;
  if (vn > cfg.maxLinear) s = Math.min(s, cfg.maxLinear / vn);
  if (wn > cfg.maxAngular) s = Math.min(s, cfg.maxAngular / wn);
  if (s === 1) return twist.slice();
  return twist.map((x) => x * s);
}

function poseErrors(current: Pose, desired: Pose): { tErr: number; rErr: number } {
  const rel = relativePose(desired, current);
  return { tErr: norm(rel.t), rErr: norm(logSO3(rel.R)) };
}

export class VisualServoSim {
  readonly target: Point3[] = DEFAULT_TARGET;
  readonly desiredPose: Pose = copyPose(DEFAULT_DESIRED_POSE);
  readonly sStar: Vec;
  readonly zStar: Vec;

  config: SimConfig;
  method: Method;
  pose: Pose;
  initialPose: Pose;
  time = 0;
  history: Sample[] = [];
  featureTrajectory: Point2[][] = [[], [], [], []];
  lastTwist: Vec = [0, 0, 0, 0, 0, 0];
  valid = true;

  constructor(method: Method = "ibvs", presetId = DEFAULT_PRESET_ID, config: Partial<SimConfig> = {}) {
    const feat = desiredFeatures(this.target, this.desiredPose);
    this.sStar = feat.sStar;
    this.zStar = feat.zStar;
    this.config = { ...DEFAULT_CONFIG, ...config };
    this.method = method;
    this.initialPose = copyPose(presetById(presetId).pose);
    this.pose = copyPose(this.initialPose);
    this.record();
  }

  reset(presetId?: string, method?: Method): void {
    if (presetId) this.initialPose = copyPose(presetById(presetId).pose);
    if (method) this.method = method;
    this.pose = copyPose(this.initialPose);
    this.time = 0;
    this.history = [];
    this.featureTrajectory = [[], [], [], []];
    this.lastTwist = [0, 0, 0, 0, 0, 0];
    this.valid = true;
    this.record();
  }

  setInitialPose(pose: Pose): void {
    this.initialPose = copyPose(pose);
  }

  private computeTwist(): Vec | null {
    const projected = projectPoints(this.pose, this.target);
    if (!projected) return null;

    if (this.method === "ibvs") {
      const s = featuresFromProjected(projected);
      const depths = depthsForMode(
        this.config.depthMode,
        depthsFromProjected(projected),
        this.zStar,
      );
      return ibvsStep({
        s,
        sStar: this.sStar,
        depths,
        lambda: this.config.lambda,
      }).twist;
    }

    return pbvsStep({
      current: this.pose,
      desired: this.desiredPose,
      lambda: this.config.lambda,
    }).twist;
  }

  /**
   * Advance the simulation by `dt` (defaults to the configured step).
   * Returns false if the target left the front half-space of the camera.
   */
  step(dt?: number): boolean {
    const h = dt ?? this.config.dt;
    const raw = this.computeTwist();
    if (!raw) {
      this.valid = false;
      return false;
    }
    const twist = clampTwist(raw, this.config);
    const next = integratePose(this.pose, twist, h);
    if (!projectPoints(next, this.target)) {
      this.valid = false;
      return false;
    }
    this.pose = next;
    this.lastTwist = twist;
    this.time += h;
    this.valid = true;
    this.record();
    return true;
  }

  private record(): void {
    const projected = projectPoints(this.pose, this.target);
    if (!projected) return;
    const features = featuresFromProjected(projected);
    const { tErr, rErr } = poseErrors(this.pose, this.desiredPose);
    this.history.push({
      t: this.time,
      featureError: featureErrorNorm(features, this.sStar),
      translationError: tErr,
      rotationError: rErr,
      distance: cameraToTargetDistance(this.pose, this.target),
    });
    projected.forEach((p, i) => {
      this.featureTrajectory[i].push({ x: p.image.x, y: p.image.y });
    });
  }

  snapshot(): SimSnapshot {
    const projected = projectPoints(this.pose, this.target);
    const desired = projectPoints(this.desiredPose, this.target);
    const last = this.history[this.history.length - 1];
    const features = projected ? projected.map((p) => p.image) : [];
    const error = last?.featureError ?? Infinity;
    const tErr = last?.translationError ?? Infinity;
    const rErr = last?.rotationError ?? Infinity;
    return {
      pose: copyPose(this.pose),
      method: this.method,
      features,
      desiredFeatures: desired ? desired.map((p) => p.image) : [],
      featureTrajectory: this.featureTrajectory.map((tr) => tr.slice()),
      error,
      translationError: tErr,
      rotationError: rErr,
      distance: last?.distance ?? cameraToTargetDistance(this.pose, this.target),
      twist: this.lastTwist.slice(),
      history: this.history.slice(),
      time: this.time,
      converged: error < CONVERGE_FEATURE && tErr < CONVERGE_T && rErr < CONVERGE_R,
      valid: this.valid && projected !== null,
    };
  }
}
