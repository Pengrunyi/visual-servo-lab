import type { Lang } from "../i18n";
import { t } from "../i18n";
import type { Sample } from "../sim/simulation";

export type PlotMetric = "feature" | "translation" | "rotation" | "distance";

export interface PlotSeries {
  history: Sample[];
  color: string;
  label: string;
}

const METRICS: PlotMetric[] = ["feature", "translation", "rotation", "distance"];

function valueOf(s: Sample, m: PlotMetric): number {
  switch (m) {
    case "feature":
      return s.featureError;
    case "translation":
      return s.translationError;
    case "rotation":
      return s.rotationError;
    case "distance":
      return s.distance;
  }
}

function metricLabel(lang: Lang, m: PlotMetric): string {
  switch (m) {
    case "feature":
      return t(lang, "errorFeature");
    case "translation":
      return t(lang, "errorTrans");
    case "rotation":
      return t(lang, "errorRot");
    case "distance":
      return t(lang, "errorDist");
  }
}

export class ErrorPlotView {
  private readonly canvas: HTMLCanvasElement;
  private readonly ctx: CanvasRenderingContext2D;
  private ro: ResizeObserver;
  metric: PlotMetric = "feature";

  constructor(canvas: HTMLCanvasElement) {
    this.canvas = canvas;
    const ctx = canvas.getContext("2d");
    if (!ctx) throw new Error("2D canvas is not available");
    this.ctx = ctx;
    this.ro = new ResizeObserver(() => this.resize());
    this.ro.observe(canvas);
    this.resize();
  }

  cycleMetric(): PlotMetric {
    const i = METRICS.indexOf(this.metric);
    this.metric = METRICS[(i + 1) % METRICS.length];
    return this.metric;
  }

  setMetric(m: PlotMetric): void {
    this.metric = m;
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

  render(series: PlotSeries[], lang: Lang): void {
    this.resize();
    const ctx = this.ctx;
    const w = this.canvas.width;
    const h = this.canvas.height;
    ctx.clearRect(0, 0, w, h);
    ctx.fillStyle = "#0e1014";
    ctx.fillRect(0, 0, w, h);

    const sf = Math.max(0.7, Math.min(w / 900, h / 260));
    const pad = { l: 52 * sf, r: 16 * sf, t: 26 * sf, b: 26 * sf };
    const plotW = Math.max(1, w - pad.l - pad.r);
    const plotH = Math.max(1, h - pad.t - pad.b);

    const all = series.flatMap((s) => s.history);
    const tMax = Math.max(1e-3, ...all.map((s) => s.t), 1);
    let yMax = Math.max(...all.map((s) => valueOf(s, this.metric)), 1e-6);
    if (this.metric === "distance") {
      // keep a little headroom around the nominal 0.6 m
      yMax = Math.max(yMax, 0.8);
    }
    yMax *= 1.12;

    // Grid
    ctx.strokeStyle = "rgba(255,255,255,0.05)";
    ctx.lineWidth = 1;
    const ticks = 4;
    const font = `${Math.round(10 * sf)}px "IBM Plex Mono", ui-monospace, monospace`;
    ctx.font = font;
    ctx.fillStyle = "rgba(154,151,140,0.85)";
    ctx.textAlign = "right";
    ctx.textBaseline = "middle";
    for (let i = 0; i <= ticks; i++) {
      const y = pad.t + (plotH * i) / ticks;
      ctx.beginPath();
      ctx.moveTo(pad.l, y);
      ctx.lineTo(pad.l + plotW, y);
      ctx.stroke();
      const v = yMax * (1 - i / ticks);
      ctx.fillText(formatTick(v), pad.l - 6, y);
    }

    ctx.textAlign = "center";
    ctx.textBaseline = "top";
    for (let i = 0; i <= ticks; i++) {
      const x = pad.l + (plotW * i) / ticks;
      ctx.beginPath();
      ctx.moveTo(x, pad.t);
      ctx.lineTo(x, pad.t + plotH);
      ctx.stroke();
      const tv = (tMax * i) / ticks;
      ctx.fillText(tv.toFixed(1) + " s", x, pad.t + plotH + 6);
    }

    // Series
    for (const s of series) {
      if (s.history.length < 2) continue;
      ctx.beginPath();
      ctx.strokeStyle = s.color;
      ctx.lineWidth = Math.max(1.6, 2.1 * sf);
      ctx.lineJoin = "round";
      ctx.lineCap = "round";
      s.history.forEach((sample, i) => {
        const x = pad.l + (sample.t / tMax) * plotW;
        const y = pad.t + (1 - valueOf(sample, this.metric) / yMax) * plotH;
        if (i === 0) ctx.moveTo(x, y);
        else ctx.lineTo(x, y);
      });
      ctx.stroke();
    }

    // Title / metric
    ctx.fillStyle = "rgba(230,228,220,0.8)";
    ctx.font = `${Math.round(11 * sf)}px "IBM Plex Sans", "Noto Sans SC", sans-serif`;
    ctx.textAlign = "left";
    ctx.textBaseline = "alphabetic";
    ctx.fillText(metricLabel(lang, this.metric), pad.l, pad.t - 8);

    // Legend
    ctx.font = font;
    ctx.textBaseline = "middle";
    let lx = pad.l + plotW;
    ctx.textAlign = "right";
    for (let i = series.length - 1; i >= 0; i--) {
      const s = series[i];
      ctx.fillStyle = "rgba(230,228,220,0.75)";
      ctx.fillText(s.label, lx, pad.t - 10);
      const tw = ctx.measureText(s.label).width;
      ctx.fillStyle = s.color;
      ctx.beginPath();
      ctx.arc(lx - tw - 10, pad.t - 10, 3, 0, Math.PI * 2);
      ctx.fill();
      lx -= tw + 28;
    }
  }
}

function formatTick(v: number): string {
  if (v === 0) return "0";
  if (v >= 1) return v.toFixed(2);
  if (v >= 0.1) return v.toFixed(2);
  return v.toFixed(3);
}
