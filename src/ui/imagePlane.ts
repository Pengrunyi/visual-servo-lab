import type { Point2 } from "../math/camera";
import type { Lang } from "../i18n";
import { t } from "../i18n";

export interface ImagePlaneSeries {
  features: Point2[];
  desired?: Point2[];
  trajectories: Point2[][];
  color: string;
  label: string;
}

const FRAME = 0.55; // half-size of the drawn "sensor" in normalized units

export class ImagePlaneView {
  private readonly canvas: HTMLCanvasElement;
  private readonly ctx: CanvasRenderingContext2D;
  private ro: ResizeObserver;

  constructor(canvas: HTMLCanvasElement) {
    this.canvas = canvas;
    const ctx = canvas.getContext("2d");
    if (!ctx) throw new Error("2D canvas is not available");
    this.ctx = ctx;
    this.ro = new ResizeObserver(() => this.resize());
    this.ro.observe(canvas);
    this.resize();
  }

  private resize(): void {
    const rect = this.canvas.getBoundingClientRect();
    const dpr = Math.min(window.devicePixelRatio || 1, 2);
    const w = Math.max(1, Math.round(rect.width * dpr));
    const h = Math.max(1, Math.round(rect.height * dpr));
    if (this.canvas.width !== w || this.canvas.height !== h) {
      this.canvas.width = w;
      this.canvas.height = h;
    }
  }

  /** Map normalized (x, y) (Y down) to canvas pixels. */
  private toPx(p: Point2): Point2 {
    const w = this.canvas.width;
    const h = this.canvas.height;
    const s = 0.42 * Math.min(w, h) / FRAME;
    return {
      x: w / 2 + p.x * s,
      y: h / 2 + p.y * s,
    };
  }

  render(series: ImagePlaneSeries[], lang: Lang): void {
    this.resize();
    const ctx = this.ctx;
    const w = this.canvas.width;
    const h = this.canvas.height;
    ctx.clearRect(0, 0, w, h);

    // Background
    ctx.fillStyle = "#0e1014";
    ctx.fillRect(0, 0, w, h);

    const tl = this.toPx({ x: -FRAME, y: -FRAME });
    const br = this.toPx({ x: FRAME, y: FRAME });
    const rw = br.x - tl.x;
    const rh = br.y - tl.y;

    // Sensor well
    ctx.fillStyle = "#161920";
    ctx.strokeStyle = "#2a2f3a";
    ctx.lineWidth = Math.max(1, this.canvas.width / 500);
    ctx.beginPath();
    const r = 8 * (this.canvas.width / 600);
    this.roundRect(tl.x, tl.y, rw, rh, r);
    ctx.fill();
    ctx.stroke();

    // Grid
    ctx.save();
    ctx.beginPath();
    this.roundRect(tl.x, tl.y, rw, rh, r);
    ctx.clip();
    ctx.strokeStyle = "rgba(255,255,255,0.04)";
    ctx.lineWidth = 1;
    for (let g = -0.4; g <= 0.4 + 1e-9; g += 0.2) {
      const a = this.toPx({ x: g, y: -FRAME });
      const b = this.toPx({ x: g, y: FRAME });
      ctx.beginPath();
      ctx.moveTo(a.x, a.y);
      ctx.lineTo(b.x, b.y);
      ctx.stroke();
      const c = this.toPx({ x: -FRAME, y: g });
      const d = this.toPx({ x: FRAME, y: g });
      ctx.beginPath();
      ctx.moveTo(c.x, c.y);
      ctx.lineTo(d.x, d.y);
      ctx.stroke();
    }

    // Principal-point cross
    const o = this.toPx({ x: 0, y: 0 });
    ctx.strokeStyle = "rgba(196,192,180,0.28)";
    ctx.lineWidth = 1;
    const tick = 10 * (this.canvas.width / 600);
    ctx.beginPath();
    ctx.moveTo(o.x - tick, o.y);
    ctx.lineTo(o.x + tick, o.y);
    ctx.moveTo(o.x, o.y - tick);
    ctx.lineTo(o.x, o.y + tick);
    ctx.stroke();
    ctx.restore();

    // Axis labels
    const font = `${Math.round(11 * (this.canvas.width / 600))}px "IBM Plex Mono", ui-monospace, monospace`;
    ctx.font = font;
    ctx.fillStyle = "rgba(154,151,140,0.85)";
    ctx.textAlign = "left";
    ctx.textBaseline = "top";
    const xLab = this.toPx({ x: FRAME, y: 0 });
    ctx.fillText("x", xLab.x + 6, xLab.y - 6);
    const yLab = this.toPx({ x: 0, y: FRAME });
    ctx.fillText("y", yLab.x + 6, yLab.y + 2);

    // Desired features (once, from the first series that has them)
    const desired = series.find((s) => s.desired && s.desired.length)?.desired;
    if (desired) {
      for (const p of desired) {
        this.drawDesired(p);
      }
    }

    // Trajectories then current features
    for (const s of series) {
      this.drawTrajectories(s.trajectories, s.color);
    }
    for (const s of series) {
      for (const p of s.features) this.drawCurrent(p, s.color);
    }

    // Legend
    ctx.font = font;
    ctx.textAlign = "left";
    ctx.textBaseline = "middle";
    const lx = tl.x + 8;
    let ly = tl.y - 16 * (this.canvas.width / 600);
    if (ly < 14) ly = br.y + 16 * (this.canvas.width / 600);
    const items: { color?: string; hollow?: boolean; label: string }[] = [
      { hollow: true, label: t(lang, "desired") },
    ];
    for (const s of series) items.push({ color: s.color, label: s.label });
    let x = lx;
    for (const item of items) {
      if (item.hollow) {
        ctx.strokeStyle = "#c4c0b4";
        ctx.lineWidth = 1.2;
        ctx.strokeRect(x, ly - 4, 8, 8);
      } else if (item.color) {
        ctx.fillStyle = item.color;
        ctx.beginPath();
        ctx.arc(x + 4, ly, 3.2, 0, Math.PI * 2);
        ctx.fill();
      }
      ctx.fillStyle = "rgba(230,228,220,0.75)";
      ctx.fillText(item.label, x + 14, ly);
      x += ctx.measureText(item.label).width + 36;
    }
  }

  private drawDesired(p: Point2): void {
    const q = this.toPx(p);
    const ctx = this.ctx;
    const s = 7 * (this.canvas.width / 700);
    ctx.strokeStyle = "#c4c0b4";
    ctx.lineWidth = 1.4 * (this.canvas.width / 700);
    ctx.strokeRect(q.x - s / 2, q.y - s / 2, s, s);
  }

  private drawCurrent(p: Point2, color: string): void {
    const q = this.toPx(p);
    const ctx = this.ctx;
    const r = 3.6 * (this.canvas.width / 700);
    ctx.fillStyle = color;
    ctx.beginPath();
    ctx.arc(q.x, q.y, r, 0, Math.PI * 2);
    ctx.fill();
    ctx.strokeStyle = "rgba(0,0,0,0.35)";
    ctx.lineWidth = 1;
    ctx.stroke();
  }

  private drawTrajectories(trails: Point2[][], color: string): void {
    const ctx = this.ctx;
    ctx.strokeStyle = color;
    ctx.globalAlpha = 0.7;
    ctx.lineWidth = 1.4 * (this.canvas.width / 700);
    ctx.lineJoin = "round";
    ctx.lineCap = "round";
    for (const trail of trails) {
      if (trail.length < 2) continue;
      ctx.beginPath();
      const p0 = this.toPx(trail[0]);
      ctx.moveTo(p0.x, p0.y);
      for (let i = 1; i < trail.length; i++) {
        const p = this.toPx(trail[i]);
        ctx.lineTo(p.x, p.y);
      }
      ctx.stroke();
    }
    ctx.globalAlpha = 1;
  }

  private roundRect(x: number, y: number, w: number, h: number, r: number): void {
    const ctx = this.ctx;
    const rr = Math.min(r, w / 2, h / 2);
    ctx.moveTo(x + rr, y);
    ctx.arcTo(x + w, y, x + w, y + h, rr);
    ctx.arcTo(x + w, y + h, x, y + h, rr);
    ctx.arcTo(x, y + h, x, y, rr);
    ctx.arcTo(x, y, x + w, y, rr);
    ctx.closePath();
  }
}
