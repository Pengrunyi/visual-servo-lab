import * as THREE from "three";
import { OrbitControls } from "three/examples/jsm/controls/OrbitControls.js";
import type { Point3 } from "../math/camera";
import type { Pose } from "../math/se3";
import { targetCenter } from "../math/camera";

/** Map CV coordinates (Y down, Z forward) into Three.js (Y up, Z forward). */
function cvToThree(x: number, y: number, z: number): THREE.Vector3 {
  return new THREE.Vector3(x, -y, z);
}

function poseToThreeMatrix(pose: Pose): THREE.Matrix4 {
  // Columns of R are camera axes in world (CV). After Y flip, the
  // basis vectors become (Rx, -Ry, Rz) and the origin (tx, -ty, tz).
  const R = pose.R;
  const t = pose.t;
  const m = new THREE.Matrix4();
  // Three.js Matrix4 is column-major.
  const cx = cvToThree(R[0][0], R[1][0], R[2][0]);
  const cy = cvToThree(R[0][1], R[1][1], R[2][1]);
  const cz = cvToThree(R[0][2], R[1][2], R[2][2]);
  const p = cvToThree(t[0], t[1], t[2]);
  m.makeBasis(cx, cy, cz);
  m.setPosition(p);
  return m;
}

function makeFrustum(color: number, opacity: number): THREE.Group {
  const group = new THREE.Group();
  const near = 0.05;
  const far = 0.22;
  const half = 0.5; // matches the drawn image-plane frame of ±0.5
  const n = near * half;
  const f = far * half;

  const pts = [
    new THREE.Vector3(0, 0, 0),
    new THREE.Vector3(-n, -n, near),
    new THREE.Vector3(n, -n, near),
    new THREE.Vector3(n, n, near),
    new THREE.Vector3(-n, n, near),
    new THREE.Vector3(-f, -f, far),
    new THREE.Vector3(f, -f, far),
    new THREE.Vector3(f, f, far),
    new THREE.Vector3(-f, f, far),
  ];

  const indexPairs = [
    [0, 5], [0, 6], [0, 7], [0, 8],
    [5, 6], [6, 7], [7, 8], [8, 5],
    [1, 2], [2, 3], [3, 4], [4, 1],
  ];
  const positions: number[] = [];
  for (const [a, b] of indexPairs) {
    positions.push(pts[a].x, pts[a].y, pts[a].z, pts[b].x, pts[b].y, pts[b].z);
  }
  const geom = new THREE.BufferGeometry();
  geom.setAttribute("position", new THREE.Float32BufferAttribute(positions, 3));
  const lines = new THREE.LineSegments(
    geom,
    new THREE.LineBasicMaterial({
      color,
      transparent: opacity < 1,
      opacity,
      depthWrite: false,
    }),
  );
  group.add(lines);

  // Slightly filled far face so the facing direction is obvious
  const face = new THREE.BufferGeometry();
  face.setAttribute(
    "position",
    new THREE.Float32BufferAttribute(
      [
        -f, -f, far, f, -f, far, f, f, far,
        -f, -f, far, f, f, far, -f, f, far,
      ],
      3,
    ),
  );
  face.computeVertexNormals();
  const mesh = new THREE.Mesh(
    face,
    new THREE.MeshBasicMaterial({
      color,
      transparent: true,
      opacity: opacity * 0.18,
      side: THREE.DoubleSide,
      depthWrite: false,
    }),
  );
  group.add(mesh);

  // Small body
  const body = new THREE.Mesh(
    new THREE.BoxGeometry(0.028, 0.02, 0.036),
    new THREE.MeshStandardMaterial({
      color,
      metalness: 0.15,
      roughness: 0.45,
      transparent: opacity < 1,
      opacity: Math.max(opacity, 0.35),
    }),
  );
  body.position.z = -0.01;
  group.add(body);
  return group;
}

export interface CameraActor {
  pose: Pose;
  color: number;
  opacity?: number;
}

export class Scene3DView {
  private readonly renderer: THREE.WebGLRenderer;
  private readonly scene: THREE.Scene;
  private readonly camera: THREE.PerspectiveCamera;
  private readonly controls: OrbitControls;
  private readonly host: HTMLElement;
  private readonly frustums = new Map<string, THREE.Group>();
  private ro: ResizeObserver;
  private disposed = false;

  constructor(host: HTMLElement, target: Point3[]) {
    this.host = host;
    this.scene = new THREE.Scene();
    this.scene.background = new THREE.Color(0x0e1014);

    const w = Math.max(1, host.clientWidth);
    const h = Math.max(1, host.clientHeight);
    this.camera = new THREE.PerspectiveCamera(42, w / h, 0.02, 20);
    // A slightly elevated, three-quarter view from behind the desired camera.
    this.camera.position.copy(cvToThree(-0.38, -0.28, -0.82));
    this.camera.up.set(0, 1, 0);

    this.renderer = new THREE.WebGLRenderer({ antialias: true, alpha: false });
    this.renderer.setPixelRatio(Math.min(window.devicePixelRatio || 1, 2));
    this.renderer.setSize(w, h);
    this.renderer.domElement.style.display = "block";
    this.renderer.domElement.style.width = "100%";
    this.renderer.domElement.style.height = "100%";
    host.appendChild(this.renderer.domElement);

    this.controls = new OrbitControls(this.camera, this.renderer.domElement);
    this.controls.enableDamping = true;
    this.controls.dampingFactor = 0.08;
    this.controls.target.copy(cvToThree(0, 0, 0.26));
    this.controls.minDistance = 0.25;
    this.controls.maxDistance = 4;
    this.controls.update();

    this.scene.add(new THREE.AmbientLight(0xb8c0cc, 0.55));
    const key = new THREE.DirectionalLight(0xfff4e6, 1.1);
    key.position.copy(cvToThree(-0.6, -1.0, -0.4));
    this.scene.add(key);
    const fill = new THREE.DirectionalLight(0x8ea4c8, 0.4);
    fill.position.copy(cvToThree(0.8, 0.4, 0.6));
    this.scene.add(fill);

    this.addHelpers(target);
    this.addTarget(target);

    this.ro = new ResizeObserver(() => this.resize());
    this.ro.observe(host);
  }

  private addHelpers(target: Point3[]): void {
    const origin = cvToThree(0, 0, 0);
    const axisLen = 0.16;
    const addAxis = (to: THREE.Vector3, color: number) => {
      const g = new THREE.BufferGeometry().setFromPoints([origin, to]);
      this.scene.add(new THREE.Line(g, new THREE.LineBasicMaterial({ color, transparent: true, opacity: 0.85 })));
    };
    addAxis(cvToThree(axisLen, 0, 0), 0xd67a6a);
    addAxis(cvToThree(0, axisLen, 0), 0x7dbe74);
    addAxis(cvToThree(0, 0, axisLen), 0x6aa4d6);

    // Target-plane grid
    const z = target[0]?.z ?? 0.6;
    const grid = new THREE.GridHelper(0.8, 8, 0x2a2f3a, 0x1a1e26);
    grid.position.copy(cvToThree(0, 0, z));
    // GridHelper lies in XZ with Y up; rotate so it lies in the CV XY (three XZ? wait)
    // CV target plane is z = const, i.e. three.js z = const, a plane parallel to three XY.
    // GridHelper is in XZ (y=0). Rotate +90° about X to put it in XY.
    grid.rotation.x = Math.PI / 2;
    this.scene.add(grid);

    const c = targetCenter(target);
    this.controls.target.copy(cvToThree(c.x, c.y, c.z * 0.45));
  }

  private addTarget(points: Point3[]): void {
    const color = 0xe6e4dc;
    const mats = new THREE.MeshStandardMaterial({
      color,
      metalness: 0.1,
      roughness: 0.35,
    });
    for (const p of points) {
      const sph = new THREE.Mesh(new THREE.SphereGeometry(0.016, 20, 16), mats);
      sph.position.copy(cvToThree(p.x, p.y, p.z));
      this.scene.add(sph);
    }
    const loop = points.map((p) => cvToThree(p.x, p.y, p.z));
    loop.push(loop[0].clone());
    const line = new THREE.Line(
      new THREE.BufferGeometry().setFromPoints(loop),
      new THREE.LineBasicMaterial({ color: 0xc4c0b4 }),
    );
    this.scene.add(line);

    if (points.length >= 4) {
      const a = cvToThree(points[0].x, points[0].y, points[0].z);
      const b = cvToThree(points[1].x, points[1].y, points[1].z);
      const c = cvToThree(points[2].x, points[2].y, points[2].z);
      const d = cvToThree(points[3].x, points[3].y, points[3].z);
      const geom = new THREE.BufferGeometry().setFromPoints([a, b, c, a, c, d]);
      geom.computeVertexNormals();
      this.scene.add(
        new THREE.Mesh(
          geom,
          new THREE.MeshStandardMaterial({
            color: 0xd8d2c4,
            transparent: true,
            opacity: 0.14,
            side: THREE.DoubleSide,
            metalness: 0,
            roughness: 0.8,
          }),
        ),
      );
    }
  }

  setCameras(actors: { id: string; pose: Pose; color: number; opacity?: number }[]): void {
    const seen = new Set<string>();
    for (const a of actors) {
      seen.add(a.id);
      let g = this.frustums.get(a.id);
      if (!g) {
        g = makeFrustum(a.color, a.opacity ?? 1);
        this.frustums.set(a.id, g);
        this.scene.add(g);
      }
      g.matrixAutoUpdate = false;
      g.matrix.copy(poseToThreeMatrix(a.pose));
    }
    for (const [id, g] of this.frustums) {
      if (!seen.has(id)) {
        this.scene.remove(g);
        this.frustums.delete(id);
      }
    }
  }

  private resize(): void {
    if (this.disposed) return;
    const w = Math.max(1, this.host.clientWidth);
    const h = Math.max(1, this.host.clientHeight);
    this.camera.aspect = w / h;
    this.camera.updateProjectionMatrix();
    this.renderer.setSize(w, h, false);
  }

  render(): void {
    if (this.disposed) return;
    this.controls.update();
    this.renderer.render(this.scene, this.camera);
  }

  dispose(): void {
    this.disposed = true;
    this.ro.disconnect();
    this.controls.dispose();
    this.renderer.dispose();
    this.renderer.domElement.remove();
  }
}
