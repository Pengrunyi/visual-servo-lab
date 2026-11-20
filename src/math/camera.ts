/**
 * Pinhole camera model and a planar four-point target.
 *
 * Computer-vision convention: camera looks along +Z, X to the right, Y down.
 * Image features used by the control laws are *normalized* coordinates
 * (x, y) = (X/Z, Y/Z). Pixel coordinates are only used for display.
 */

import { type Vec } from "./linalg";
import {
  type Pose,
  identityPose,
  inverseTransformPoint,
} from "./se3";

export interface Point2 {
  x: number;
  y: number;
}

export interface Point3 {
  x: number;
  y: number;
  z: number;
}

export function vec3(p: Point3): Vec {
  return [p.x, p.y, p.z];
}

export function point3(v: Vec): Point3 {
  return { x: v[0], y: v[1], z: v[2] };
}

/** A square (or rectangle) of four coplanar points in the world frame. */
export function makeSquareTarget(
  center: Point3,
  halfX: number,
  halfY: number,
): Point3[] {
  const { x, y, z } = center;
  // Order: top-left, top-right, bottom-right, bottom-left in the image
  // (Y down), i.e. increasing x then increasing y.
  return [
    { x: x - halfX, y: y - halfY, z },
    { x: x + halfX, y: y - halfY, z },
    { x: x + halfX, y: y + halfY, z },
    { x: x - halfX, y: y + halfY, z },
  ];
}

/**
 * Default teaching scene.
 * Desired camera sits at the world origin looking along +Z. The target is a
 * 0.24 m square, 0.60 m in front of the camera, centred on the optical axis.
 * Desired normalized features are then (±0.2, ±0.2).
 */
export const DEFAULT_TARGET: Point3[] = makeSquareTarget(
  { x: 0, y: 0, z: 0.6 },
  0.12,
  0.12,
);

export const DEFAULT_DESIRED_POSE: Pose = identityPose();

/** Transform a world point into the camera frame. */
export function worldToCamera(camera: Pose, pWorld: Point3): Point3 {
  return point3(inverseTransformPoint(camera, vec3(pWorld)));
}

/**
 * Normalized projection. Returns null if the point is at or behind the camera
 * (Z ≤ zMin), so the caller can reject an invalid step.
 */
export function projectNormalized(pCam: Point3, zMin = 1e-4): Point2 | null {
  if (pCam.z <= zMin) return null;
  return { x: pCam.x / pCam.z, y: pCam.y / pCam.z };
}

export interface ProjectedPoint {
  image: Point2;
  /** Depth Z in the camera frame (metres). */
  z: number;
  camera: Point3;
}

export function projectPoints(
  camera: Pose,
  worldPoints: Point3[],
  zMin = 1e-4,
): ProjectedPoint[] | null {
  const out: ProjectedPoint[] = [];
  for (const p of worldPoints) {
    const cam = worldToCamera(camera, p);
    const image = projectNormalized(cam, zMin);
    if (!image) return null;
    out.push({ image, z: cam.z, camera: cam });
  }
  return out;
}

/** Stack four (x, y) pairs into an 8-vector, the feature s. */
export function featuresFromProjected(pts: ProjectedPoint[]): Vec {
  const s: number[] = [];
  for (const p of pts) {
    s.push(p.image.x, p.image.y);
  }
  return s;
}

export function depthsFromProjected(pts: ProjectedPoint[]): Vec {
  return pts.map((p) => p.z);
}

export function desiredFeatures(
  worldPoints: Point3[],
  desiredPose: Pose,
): { sStar: Vec; zStar: Vec } {
  const projected = projectPoints(desiredPose, worldPoints);
  if (!projected) {
    throw new Error("Desired pose does not see all target points");
  }
  return {
    sStar: featuresFromProjected(projected),
    zStar: depthsFromProjected(projected),
  };
}

/** Euclidean feature-error norm ‖s − s*‖. */
export function featureErrorNorm(s: Vec, sStar: Vec): number {
  let acc = 0;
  for (let i = 0; i < s.length; i++) {
    const d = s[i] - sStar[i];
    acc += d * d;
  }
  return Math.sqrt(acc);
}

/** Mid-point of the four target corners, used as a distance reference. */
export function targetCenter(points: Point3[]): Point3 {
  const acc = [0, 0, 0];
  for (const p of points) {
    acc[0] += p.x;
    acc[1] += p.y;
    acc[2] += p.z;
  }
  const n = points.length || 1;
  return { x: acc[0] / n, y: acc[1] / n, z: acc[2] / n };
}

export function cameraToTargetDistance(camera: Pose, points: Point3[]): number {
  const c = targetCenter(points);
  const dx = camera.t[0] - c.x;
  const dy = camera.t[1] - c.y;
  const dz = camera.t[2] - c.z;
  return Math.sqrt(dx * dx + dy * dy + dz * dz);
}

/** Convert normalized (x, y) to pixel coordinates for display. */
export function normalizedToPixel(
  p: Point2,
  width: number,
  height: number,
  fx: number,
  fy: number,
  cx?: number,
  cy?: number,
): Point2 {
  const cX = cx ?? width / 2;
  const cY = cy ?? height / 2;
  return { x: fx * p.x + cX, y: fy * p.y + cY };
}
