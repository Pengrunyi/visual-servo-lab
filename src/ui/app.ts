import type { DepthMode } from "../math/ibvs";
import type { Lang, MessageKey } from "../i18n";
import { t } from "../i18n";
import { PRESETS } from "../sim/presets";
import { type Method, VisualServoSim } from "../sim/simulation";
import { ErrorPlotView, type PlotMetric } from "./errorPlot";
import { ImagePlaneView } from "./imagePlane";
import { Scene3DView } from "./scene3d";

export type ViewMode = Method | "compare";

const IBVS_COLOR = "#3ecfbf";
const PBVS_COLOR = "#e0b15a";
const DESIRED_COLOR = 0xc4c0b4;

export class App {
  private lang: Lang = "zh";
  private mode: ViewMode = "ibvs";
  private presetId = "translate-rotate";
  private running = false;
  private raf = 0;
  private lastTs = 0;
  private acc = 0;

  private readonly ibvs: VisualServoSim;
  private readonly pbvs: VisualServoSim;
  private readonly image: ImagePlaneView;
  private readonly plot: ErrorPlotView;
  private readonly scene: Scene3DView;

  constructor() {
    this.ibvs = new VisualServoSim("ibvs", this.presetId);
    this.pbvs = new VisualServoSim("pbvs", this.presetId);

    const imageCanvas = must("#image-plane") as HTMLCanvasElement;
    const plotCanvas = must("#error-plot") as HTMLCanvasElement;
    const sceneHost = must("#scene-3d");
    this.image = new ImagePlaneView(imageCanvas);
    this.plot = new ErrorPlotView(plotCanvas);
    this.scene = new Scene3DView(sceneHost, this.ibvs.target);

    this.bind();
    this.buildPresets();
    this.applyLang();
    this.paint();
    this.tickScene();
  }

  private bind(): void {
    must("#btn-start").addEventListener("click", () => this.start());
    must("#btn-pause").addEventListener("click", () => this.pause());
    must("#btn-reset").addEventListener("click", () => this.reset());
    must("#lang-btn").addEventListener("click", () => {
      this.lang = this.lang === "zh" ? "en" : "zh";
      this.applyLang();
      this.paint();
    });

    document.querySelectorAll<HTMLButtonElement>("[data-method]").forEach((btn) => {
      btn.addEventListener("click", () => {
        const mode = btn.dataset.method as ViewMode;
        this.setMode(mode);
      });
    });

    document.querySelectorAll<HTMLButtonElement>("[data-depth]").forEach((btn) => {
      btn.addEventListener("click", () => {
        const depth = btn.dataset.depth as DepthMode;
        this.setDepth(depth);
      });
    });

    const slider = must("#lambda") as HTMLInputElement;
    slider.addEventListener("input", () => {
      const lambda = Number(slider.value);
      this.ibvs.config.lambda = lambda;
      this.pbvs.config.lambda = lambda;
      must("#lambda-val").textContent = lambda.toFixed(2);
    });

    document.querySelectorAll<HTMLButtonElement>("[data-metric]").forEach((btn) => {
      btn.addEventListener("click", () => {
        const metric = btn.dataset.metric as PlotMetric;
        this.plot.setMetric(metric);
        document.querySelectorAll("[data-metric]").forEach((b) => {
          b.classList.toggle("is-active", b === btn);
        });
        this.paint();
      });
    });

    window.addEventListener("keydown", (ev) => {
      if (ev.target instanceof HTMLInputElement) return;
      if (ev.code === "Space") {
        ev.preventDefault();
        if (this.running) this.pause();
        else this.start();
      } else if (ev.key === "r" || ev.key === "R") {
        this.reset();
      }
    });
  }

  private buildPresets(): void {
    const host = must("#presets");
    host.replaceChildren();
    for (const p of PRESETS) {
      const btn = document.createElement("button");
      btn.type = "button";
      btn.className = "preset-btn" + (p.illustratesRetreat ? " is-retreat" : "");
      btn.dataset.preset = p.id;
      btn.dataset.i18n = p.labelKey;
      if (p.id === this.presetId) btn.classList.add("is-active");
      btn.addEventListener("click", () => this.setPreset(p.id));
      host.appendChild(btn);
    }
  }

  private setMode(mode: ViewMode): void {
    if (this.mode === mode) return;
    this.pause();
    this.mode = mode;
    document.querySelectorAll("[data-method]").forEach((b) => {
      b.classList.toggle("is-active", (b as HTMLElement).dataset.method === mode);
    });
    // Same initial condition, method switch is instant.
    this.ibvs.reset(this.presetId, "ibvs");
    this.pbvs.reset(this.presetId, "pbvs");
    this.applyLang();
    this.paint();
  }

  private setDepth(mode: DepthMode): void {
    this.ibvs.config.depthMode = mode;
    this.pbvs.config.depthMode = mode;
    document.querySelectorAll("[data-depth]").forEach((b) => {
      b.classList.toggle("is-active", (b as HTMLElement).dataset.depth === mode);
    });
  }

  private setPreset(id: string): void {
    this.pause();
    this.presetId = id;
    document.querySelectorAll("[data-preset]").forEach((b) => {
      b.classList.toggle("is-active", (b as HTMLElement).dataset.preset === id);
    });
    this.ibvs.reset(id, "ibvs");
    this.pbvs.reset(id, "pbvs");
    this.paint();
  }

  private start(): void {
    if (this.running) return;
    const snaps = this.activeSims().map((s) => s.snapshot());
    if (snaps.every((s) => s.converged || !s.valid)) return;
    this.running = true;
    this.lastTs = performance.now();
    this.acc = 0;
    this.raf = requestAnimationFrame((t) => this.loop(t));
    this.updateButtons();
  }

  private pause(): void {
    this.running = false;
    if (this.raf) cancelAnimationFrame(this.raf);
    this.raf = 0;
    this.updateButtons();
    this.paint();
  }

  private reset(): void {
    this.pause();
    this.ibvs.reset(this.presetId, "ibvs");
    this.pbvs.reset(this.presetId, "pbvs");
    this.paint();
  }

  private loop(ts: number): void {
    if (!this.running) return;
    const wall = Math.min(0.05, (ts - this.lastTs) / 1000);
    this.lastTs = ts;
    this.acc += wall;
    const dt = this.ibvs.config.dt;
    let steps = 0;
    while (this.acc >= dt && steps < 8) {
      for (const sim of this.activeSims()) {
        if (!sim.snapshot().converged && sim.valid) sim.step(dt);
      }
      this.acc -= dt;
      steps += 1;
    }
    this.paint();
    const done = this.activeSims().every((s) => {
      const snap = s.snapshot();
      return snap.converged || !snap.valid;
    });
    if (done) {
      this.pause();
      return;
    }
    this.raf = requestAnimationFrame((t) => this.loop(t));
  }

  private activeSims(): VisualServoSim[] {
    if (this.mode === "ibvs") return [this.ibvs];
    if (this.mode === "pbvs") return [this.pbvs];
    return [this.ibvs, this.pbvs];
  }

  private applyLang(): void {
    document.documentElement.lang = this.lang === "zh" ? "zh-CN" : "en";
    document.querySelectorAll<HTMLElement>("[data-i18n]").forEach((el) => {
      const key = el.dataset.i18n as MessageKey | undefined;
      if (!key) return;
      el.textContent = t(this.lang, key);
    });
    const title =
      this.lang === "zh"
        ? "视觉伺服实验台 · Visual Servo Lab"
        : "Visual Servo Lab · 视觉伺服实验台";
    document.title = title;
    this.updateButtons();
    this.renderReadouts();
  }

  private updateButtons(): void {
    const start = must("#btn-start");
    start.textContent = t(this.lang, "start");
    start.classList.toggle("is-running", this.running);
    (must("#btn-pause") as HTMLButtonElement).disabled = !this.running;
    (must("#btn-start") as HTMLButtonElement).disabled = this.running;
  }

  private paint(): void {
    this.renderImage();
    this.renderPlot();
    this.renderScene();
    this.renderReadouts();
    this.updateButtons();
  }

  private renderImage(): void {
    const desired = this.ibvs.snapshot().desiredFeatures;
    const series = [];
    if (this.mode !== "pbvs") {
      const s = this.ibvs.snapshot();
      series.push({
        features: s.features,
        desired,
        trajectories: s.featureTrajectory,
        color: IBVS_COLOR,
        label: t(this.lang, "ibvsSeries"),
      });
    }
    if (this.mode !== "ibvs") {
      const s = this.pbvs.snapshot();
      series.push({
        features: s.features,
        desired,
        trajectories: s.featureTrajectory,
        color: PBVS_COLOR,
        label: t(this.lang, "pbvsSeries"),
      });
    }
    this.image.render(series, this.lang);
  }

  private renderPlot(): void {
    const series = [];
    if (this.mode !== "pbvs") {
      series.push({
        history: this.ibvs.snapshot().history,
        color: IBVS_COLOR,
        label: t(this.lang, "ibvsSeries"),
      });
    }
    if (this.mode !== "ibvs") {
      series.push({
        history: this.pbvs.snapshot().history,
        color: PBVS_COLOR,
        label: t(this.lang, "pbvsSeries"),
      });
    }
    this.plot.render(series, this.lang);
  }

  private renderScene(): void {
    const actors = [
      {
        id: "desired",
        pose: this.ibvs.desiredPose,
        color: DESIRED_COLOR,
        opacity: 0.7,
      },
    ];
    if (this.mode !== "pbvs") {
      actors.push({
        id: "ibvs",
        pose: this.ibvs.pose,
        color: 0x3ecfbf,
        opacity: 1,
      });
    }
    if (this.mode !== "ibvs") {
      actors.push({
        id: "pbvs",
        pose: this.pbvs.pose,
        color: 0xe0b15a,
        opacity: 1,
      });
    }
    this.scene.setCameras(actors);
  }

  private tickScene = (): void => {
    this.scene.render();
    requestAnimationFrame(this.tickScene);
  };

  private renderReadouts(): void {
    const host = must("#readouts");
    const compare = this.mode === "compare";
    const rows: { key: MessageKey; ibvs: string; pbvs: string; cls?: string }[] = [];

    const fmt = (v: number, digits = 3) => (Number.isFinite(v) ? v.toFixed(digits) : "—");
    const statusOf = (s: ReturnType<VisualServoSim["snapshot"]>): { text: string; cls: string } => {
      if (!s.valid) return { text: t(this.lang, "statusInvalid"), cls: "status-bad" };
      if (s.converged) return { text: t(this.lang, "statusConverged"), cls: "status-ok" };
      if (this.running) return { text: t(this.lang, "statusRunning"), cls: "" };
      if (s.time > 0) return { text: t(this.lang, "statusPaused"), cls: "" };
      return { text: t(this.lang, "statusIdle"), cls: "" };
    };

    const a = this.ibvs.snapshot();
    const b = this.pbvs.snapshot();
    const sa = statusOf(a);
    const sb = statusOf(b);

    rows.push({
      key: "readoutTime",
      ibvs: `${fmt(a.time, 2)} s`,
      pbvs: `${fmt(b.time, 2)} s`,
    });
    rows.push({
      key: "readoutFeature",
      ibvs: fmt(a.error),
      pbvs: fmt(b.error),
    });
    rows.push({
      key: "readoutTrans",
      ibvs: `${fmt(a.translationError)} m`,
      pbvs: `${fmt(b.translationError)} m`,
    });
    rows.push({
      key: "readoutRot",
      ibvs: `${fmt(a.rotationError)} rad`,
      pbvs: `${fmt(b.rotationError)} rad`,
    });
    rows.push({
      key: "readoutDist",
      ibvs: `${fmt(a.distance)} m`,
      pbvs: `${fmt(b.distance)} m`,
    });

    const table = document.createElement("table");
    table.className = "readout-table";
    const thead = document.createElement("thead");
    const hr = document.createElement("tr");
    hr.innerHTML = compare
      ? `<th></th><th class="col-ibvs">IBVS</th><th class="col-pbvs">PBVS</th>`
      : `<th></th><th class="${this.mode === "ibvs" ? "col-ibvs" : "col-pbvs"}">${this.mode === "ibvs" ? "IBVS" : "PBVS"}</th>`;
    thead.appendChild(hr);
    table.appendChild(thead);

    const tb = document.createElement("tbody");
    const st = document.createElement("tr");
    if (compare) {
      st.innerHTML = `<td>${t(this.lang, "readoutStatus")}</td><td class="col-ibvs ${sa.cls}">${sa.text}</td><td class="col-pbvs ${sb.cls}">${sb.text}</td>`;
    } else {
      const s = this.mode === "ibvs" ? sa : sb;
      st.innerHTML = `<td>${t(this.lang, "readoutStatus")}</td><td class="${s.cls}">${s.text}</td>`;
    }
    tb.appendChild(st);

    for (const row of rows) {
      const tr = document.createElement("tr");
      if (compare) {
        tr.innerHTML = `<td>${t(this.lang, row.key)}</td><td class="col-ibvs">${row.ibvs}</td><td class="col-pbvs">${row.pbvs}</td>`;
      } else {
        const val = this.mode === "ibvs" ? row.ibvs : row.pbvs;
        tr.innerHTML = `<td>${t(this.lang, row.key)}</td><td>${val}</td>`;
      }
      tb.appendChild(tr);
    }
    table.appendChild(tb);
    host.replaceChildren(table);
  }
}

function must(sel: string): HTMLElement {
  const el = document.querySelector(sel);
  if (!(el instanceof HTMLElement)) {
    throw new Error(`Missing element ${sel}`);
  }
  return el;
}
