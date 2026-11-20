import { type Pose, poseFromEuler } from "../math/se3";

export interface PosePreset {
  id: string;
  /** i18n key, resolved in the UI. */
  labelKey: string;
  pose: Pose;
  /** Highlighted in the UI as the camera-retreat demonstration. */
  illustratesRetreat?: boolean;
}

const deg = (d: number) => (d * Math.PI) / 180;

export const PRESETS: PosePreset[] = [
  {
    id: "small-translation",
    labelKey: "presetSmallT",
    pose: poseFromEuler([0.12, -0.08, -0.05], 0, 0, 0),
  },
  {
    id: "translate-rotate",
    labelKey: "presetTR",
    pose: poseFromEuler([0.16, -0.1, -0.08], deg(18), deg(-22), deg(15)),
  },
  {
    id: "large-rotation",
    labelKey: "presetRetreat",
    // ~150° about the optical axis: IBVS drives features in straight
    // lines through the image centre, so the camera retreats then returns.
    pose: poseFromEuler([0.02, 0.02, 0.0], deg(4), deg(-4), deg(150)),
    illustratesRetreat: true,
  },
  {
    id: "farther",
    labelKey: "presetFar",
    pose: poseFromEuler([0.1, 0.06, -0.22], deg(8), deg(12), deg(-10)),
  },
];

export const DEFAULT_PRESET_ID = "translate-rotate";

export function presetById(id: string): PosePreset {
  return PRESETS.find((p) => p.id === id) ?? PRESETS[1];
}
