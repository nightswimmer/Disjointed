/**
 * A recording stand-in for CanvasRenderingContext2D that emits SVG instead of pixels.
 *
 * Used only by the automation hook (src/automation.ts) to capture the canvas as a vector
 * illustration for the in-app help: `render(recorder as any, renderInput())` replays one
 * frame through exactly the drawing code the real canvas gets, so the picture is the
 * renderer's own, then `toSvg()` serialises it. The renderer uses a small, well-defined
 * slice of the 2D API (paths, arcs, dashes, roundRect, fill / stroke, alpha, transforms
 * and four text calls) - that is the whole surface implemented here. Anything else is
 * simply absent, so a renderer change that outgrows the recorder fails loudly in the
 * generator rather than silently drawing nothing.
 *
 * Coordinates: each path is kept in the user space it was built in and emitted with the
 * transform current at fill / stroke time (the renderer never changes the transform
 * mid-path), so stroke widths, dash patterns and fonts scale exactly like on canvas.
 * Theme colours can be mapped to CSS custom properties (`themeVars`) so one SVG follows
 * the help page's light / dark theme live when inlined into the document.
 */

type Matrix = [number, number, number, number, number, number];
const IDENTITY: Matrix = [1, 0, 0, 1, 0, 0];
const TAU = Math.PI * 2;

interface State {
  m: Matrix;
  lineWidth: number;
  strokeStyle: string;
  fillStyle: string;
  globalAlpha: number;
  textAlign: string;
  textBaseline: string;
  font: string;
  lineCap: string;
  lineJoin: string;
  dash: number[];
}

/** Multiply two affine matrices (canvas `transform` semantics: m x n). */
function mul(m: Matrix, n: Matrix): Matrix {
  return [
    m[0] * n[0] + m[2] * n[1],
    m[1] * n[0] + m[3] * n[1],
    m[0] * n[2] + m[2] * n[3],
    m[1] * n[2] + m[3] * n[3],
    m[0] * n[4] + m[2] * n[5] + m[4],
    m[1] * n[4] + m[3] * n[5] + m[5],
  ];
}

function fmt(n: number, digits = 3): string {
  if (!Number.isFinite(n)) return "0";
  const s = n.toFixed(digits);
  return s.includes(".") ? s.replace(/\.?0+$/, "").replace(/^-0$/, "0") : s;
}

function sameMatrix(a: Matrix, b: Matrix): boolean {
  for (let i = 0; i < 6; i++) if (Math.abs(a[i] - b[i]) > 1e-9) return false;
  return true;
}

function escapeXml(s: string): string {
  return s.replace(/&/g, "&amp;").replace(/</g, "&lt;").replace(/>/g, "&gt;").replace(/"/g, "&quot;");
}

/** Split a CSS `font` shorthand into the pieces SVG wants. */
function parseFont(font: string): { style: string; weight: string; size: string; family: string } {
  const tokens = font.trim().split(/\s+/);
  let style = "normal";
  let weight = "normal";
  let i = 0;
  for (; i < tokens.length; i++) {
    const t = tokens[i];
    if (/^\d/.test(t)) break; // the size
    if (t === "italic" || t === "oblique") style = t;
    else if (t === "bold" || t === "bolder" || t === "lighter" || /^\d{3}$/.test(t)) weight = t;
  }
  const size = tokens[i] ?? "10px";
  const family = tokens.slice(i + 1).join(" ") || "sans-serif";
  return { style, weight, size, family };
}

export interface SvgRecorderOptions {
  /** CSS size of the (pretend) canvas. */
  width: number;
  height: number;
  /** Device pixel ratio the renderer will use (it reads window.devicePixelRatio). */
  dpr: number;
  /**
   * Colours to express as CSS custom properties: lower-case `#rrggbb` -> variable name
   * (without the leading dashes). Emitted as `var(--name, #rrggbb)` so the SVG still
   * looks right on its own and follows the page theme once inlined.
   */
  themeVars?: Record<string, string>;
}

export class SvgRecorder {
  /** What the renderer reads for its layout (`ctx.canvas.clientWidth` ...). */
  readonly canvas: { clientWidth: number; clientHeight: number; width: number; height: number };
  private readonly dpr: number;
  private readonly themeVars: Map<string, string>;
  private st: State;
  private stack: State[] = [];
  private d: string[] = []; // the current path, user space
  private cur: { x: number; y: number } | null = null;
  private start: { x: number; y: number } | null = null;
  private out: string[] = [];
  private groupMatrix: Matrix | null = null;
  private measurer: CanvasRenderingContext2D | null;

  constructor(opts: SvgRecorderOptions) {
    this.dpr = opts.dpr;
    this.canvas = {
      clientWidth: opts.width,
      clientHeight: opts.height,
      width: Math.floor(opts.width * opts.dpr),
      height: Math.floor(opts.height * opts.dpr),
    };
    this.themeVars = new Map(Object.entries(opts.themeVars ?? {}).map(([k, v]) => [k.toLowerCase(), v]));
    this.st = {
      m: [...IDENTITY] as Matrix,
      lineWidth: 1,
      strokeStyle: "#000000",
      fillStyle: "#000000",
      globalAlpha: 1,
      textAlign: "start",
      textBaseline: "alphabetic",
      font: "10px sans-serif",
      lineCap: "butt",
      lineJoin: "miter",
      dash: [],
    };
    this.measurer =
      typeof document !== "undefined" ? document.createElement("canvas").getContext("2d") : null;
  }

  // --- state properties (mirroring the canvas API) -----------------------------
  get lineWidth(): number { return this.st.lineWidth; }
  set lineWidth(v: number) { this.st.lineWidth = v; }
  get strokeStyle(): string { return this.st.strokeStyle; }
  set strokeStyle(v: string) { this.st.strokeStyle = String(v); }
  get fillStyle(): string { return this.st.fillStyle; }
  set fillStyle(v: string) { this.st.fillStyle = String(v); }
  get globalAlpha(): number { return this.st.globalAlpha; }
  set globalAlpha(v: number) { this.st.globalAlpha = v; }
  get textAlign(): string { return this.st.textAlign; }
  set textAlign(v: string) { this.st.textAlign = v; }
  get textBaseline(): string { return this.st.textBaseline; }
  set textBaseline(v: string) { this.st.textBaseline = v; }
  get font(): string { return this.st.font; }
  set font(v: string) { this.st.font = v; }
  get lineCap(): string { return this.st.lineCap; }
  set lineCap(v: string) { this.st.lineCap = v; }
  get lineJoin(): string { return this.st.lineJoin; }
  set lineJoin(v: string) { this.st.lineJoin = v; }

  save(): void {
    this.stack.push({ ...this.st, m: [...this.st.m] as Matrix, dash: [...this.st.dash] });
  }
  restore(): void {
    const s = this.stack.pop();
    if (s) this.st = s;
  }

  // --- transforms ------------------------------------------------------------------
  setTransform(a: number, b: number, c: number, d: number, e: number, f: number): void {
    this.st.m = [a, b, c, d, e, f];
  }
  transform(a: number, b: number, c: number, d: number, e: number, f: number): void {
    this.st.m = mul(this.st.m, [a, b, c, d, e, f]);
  }
  translate(x: number, y: number): void {
    this.transform(1, 0, 0, 1, x, y);
  }
  rotate(angle: number): void {
    const c = Math.cos(angle), s = Math.sin(angle);
    this.transform(c, s, -s, c, 0, 0);
  }
  /** Clipping is not recorded (the shape-draft hatching is a transient overlay). */
  clip(): void {}
  setLineDash(segments: number[]): void {
    this.st.dash = [...segments];
  }
  getLineDash(): number[] {
    return [...this.st.dash];
  }

  // --- path building ----------------------------------------------------------------
  beginPath(): void {
    this.d = [];
    this.cur = null;
    this.start = null;
  }
  moveTo(x: number, y: number): void {
    this.d.push(`M${fmt(x)} ${fmt(y)}`);
    this.cur = { x, y };
    this.start = { x, y };
  }
  lineTo(x: number, y: number): void {
    if (!this.cur) return this.moveTo(x, y);
    this.d.push(`L${fmt(x)} ${fmt(y)}`);
    this.cur = { x, y };
  }
  closePath(): void {
    if (!this.start) return;
    this.d.push("Z");
    this.cur = { ...this.start };
  }
  rect(x: number, y: number, w: number, h: number): void {
    this.d.push(`M${fmt(x)} ${fmt(y)}h${fmt(w)}v${fmt(h)}h${fmt(-w)}Z`);
    this.cur = { x, y };
    this.start = { x, y };
  }
  roundRect(x: number, y: number, w: number, h: number, radii: number | number[] = 0): void {
    const r0 = Array.isArray(radii) ? radii[0] ?? 0 : radii;
    const r = Math.max(0, Math.min(r0, Math.abs(w) / 2, Math.abs(h) / 2));
    if (r === 0) return this.rect(x, y, w, h);
    const a = `A${fmt(r)} ${fmt(r)} 0 0 1`;
    this.d.push(
      `M${fmt(x + r)} ${fmt(y)}` +
        `H${fmt(x + w - r)}${a} ${fmt(x + w)} ${fmt(y + r)}` +
        `V${fmt(y + h - r)}${a} ${fmt(x + w - r)} ${fmt(y + h)}` +
        `H${fmt(x + r)}${a} ${fmt(x)} ${fmt(y + h - r)}` +
        `V${fmt(y + r)}${a} ${fmt(x + r)} ${fmt(y)}Z`
    );
    this.cur = { x, y };
    this.start = { x, y };
  }
  arc(cx: number, cy: number, r: number, a0: number, a1: number, anticlockwise = false): void {
    // Canvas semantics: a sweep of a full turn or more is a full circle; otherwise the
    // angular difference is taken modulo one turn in the requested direction.
    let sweep = a1 - a0;
    if (!anticlockwise) sweep = sweep >= TAU ? TAU : ((sweep % TAU) + TAU) % TAU;
    else sweep = -sweep >= TAU ? -TAU : -((((-sweep) % TAU) + TAU) % TAU);
    const sx = cx + r * Math.cos(a0);
    const sy = cy + r * Math.sin(a0);
    if (this.cur) this.d.push(`L${fmt(sx)} ${fmt(sy)}`);
    else {
      this.d.push(`M${fmt(sx)} ${fmt(sy)}`);
      this.start = { x: sx, y: sy };
    }
    // SVG arcs can't describe more than a half turn unambiguously: split into <= pi pieces.
    const n = Math.max(1, Math.ceil(Math.abs(sweep) / Math.PI - 1e-9));
    const flag = sweep > 0 ? 1 : 0; // positive angles run clockwise on screen in both models
    let a = a0;
    for (let i = 0; i < n; i++) {
      a += sweep / n;
      const ex = cx + r * Math.cos(a);
      const ey = cy + r * Math.sin(a);
      this.d.push(`A${fmt(r)} ${fmt(r)} 0 0 ${flag} ${fmt(ex)} ${fmt(ey)}`);
      this.cur = { x: ex, y: ey };
    }
  }

  // --- painting --------------------------------------------------------------------
  fill(rule?: string): void {
    if (this.d.length === 0) return;
    const c = this.paint(this.st.fillStyle);
    this.emit(
      `<path d="${this.d.join("")}" fill="${c.color}"` +
        (rule === "evenodd" ? ` fill-rule="evenodd"` : "") +
        this.opacity(c.alpha, "fill-opacity") +
        this.alphaAttr() +
        `/>`
    );
  }
  stroke(): void {
    if (this.d.length === 0) return;
    this.emit(`<path d="${this.d.join("")}" fill="none"${this.strokeAttrs()}${this.alphaAttr()}/>`);
  }
  fillRect(x: number, y: number, w: number, h: number): void {
    const c = this.paint(this.st.fillStyle);
    this.emit(
      `<rect x="${fmt(x)}" y="${fmt(y)}" width="${fmt(w)}" height="${fmt(h)}" fill="${c.color}"` +
        this.opacity(c.alpha, "fill-opacity") +
        this.alphaAttr() +
        `/>`
    );
  }
  strokeRect(x: number, y: number, w: number, h: number): void {
    this.emit(
      `<rect x="${fmt(x)}" y="${fmt(y)}" width="${fmt(w)}" height="${fmt(h)}" fill="none"${this.strokeAttrs()}${this.alphaAttr()}/>`
    );
  }
  clearRect(_x: number, _y: number, _w: number, _h: number): void {
    // The renderer clears the whole canvas once per frame; the SVG starts empty.
  }
  fillText(text: string, x: number, y: number): void {
    const f = parseFont(this.st.font);
    const anchor =
      this.st.textAlign === "center" ? "middle"
      : this.st.textAlign === "right" || this.st.textAlign === "end" ? "end"
      : "start";
    const baseline =
      this.st.textBaseline === "middle" ? "central"
      : this.st.textBaseline === "top" || this.st.textBaseline === "hanging" ? "hanging"
      : this.st.textBaseline === "bottom" || this.st.textBaseline === "ideographic" ? "text-after-edge"
      : "alphabetic";
    const c = this.paint(this.st.fillStyle);
    this.emit(
      `<text x="${fmt(x)}" y="${fmt(y)}" fill="${c.color}" text-anchor="${anchor}" dominant-baseline="${baseline}"` +
        ` font-family="${escapeXml(f.family)}" font-size="${f.size}"` +
        (f.weight !== "normal" ? ` font-weight="${f.weight}"` : "") +
        (f.style !== "normal" ? ` font-style="${f.style}"` : "") +
        this.opacity(c.alpha, "fill-opacity") +
        this.alphaAttr() +
        `>${escapeXml(text)}</text>`
    );
  }
  measureText(text: string): { width: number } {
    if (!this.measurer) return { width: text.length * 6 };
    this.measurer.font = this.st.font;
    return { width: this.measurer.measureText(text).width };
  }

  // --- output -----------------------------------------------------------------------
  /**
   * The recorded frame as an SVG document. `crop` (CSS px, canvas-relative) narrows the
   * viewBox to a region - the picture is otherwise the whole canvas.
   */
  toSvg(crop?: { x: number; y: number; w: number; h: number }): string {
    this.closeGroup();
    const k = this.dpr;
    const box = crop
      ? `${fmt(crop.x * k)} ${fmt(crop.y * k)} ${fmt(crop.w * k)} ${fmt(crop.h * k)}`
      : `0 0 ${fmt(this.canvas.width)} ${fmt(this.canvas.height)}`;
    const w = crop ? crop.w : this.canvas.clientWidth;
    const h = crop ? crop.h : this.canvas.clientHeight;
    return (
      `<svg xmlns="http://www.w3.org/2000/svg" width="${fmt(w)}" height="${fmt(h)}" viewBox="${box}">\n` +
      this.out.join("\n") +
      `\n</svg>\n`
    );
  }

  // --- internals ----------------------------------------------------------------------
  private emit(el: string): void {
    const m = this.st.m;
    if (!this.groupMatrix || !sameMatrix(this.groupMatrix, m)) {
      this.closeGroup();
      this.groupMatrix = [...m] as Matrix;
      this.out.push(`<g transform="matrix(${m.map((v) => fmt(v, 5)).join(" ")})">`);
    }
    this.out.push("  " + el);
  }
  private closeGroup(): void {
    if (this.groupMatrix) this.out.push("</g>");
    this.groupMatrix = null;
  }
  private alphaAttr(): string {
    return this.st.globalAlpha < 1 ? ` opacity="${fmt(this.st.globalAlpha)}"` : "";
  }
  private strokeAttrs(): string {
    const c = this.paint(this.st.strokeStyle);
    let s = ` stroke="${c.color}" stroke-width="${fmt(this.st.lineWidth, 4)}"`;
    if (this.st.dash.length && this.st.dash.some((d) => d > 0)) {
      s += ` stroke-dasharray="${this.st.dash.map((d) => fmt(d, 4)).join(" ")}"`;
    }
    if (this.st.lineCap !== "butt") s += ` stroke-linecap="${this.st.lineCap}"`;
    if (this.st.lineJoin !== "miter") s += ` stroke-linejoin="${this.st.lineJoin}"`;
    s += this.opacity(c.alpha, "stroke-opacity");
    return s;
  }
  private opacity(alpha: number, attr: string): string {
    return alpha < 1 ? ` ${attr}="${fmt(alpha)}"` : "";
  }
  /**
   * A canvas colour string as an SVG colour + separate alpha. `#rrggbbaa` splits into
   * colour and alpha (the renderer builds translucent tints that way); a theme colour
   * becomes a CSS variable reference with the literal as its fallback.
   */
  private paint(style: string): { color: string; alpha: number } {
    let s = style.trim().toLowerCase();
    let alpha = 1;
    if (/^#[0-9a-f]{8}$/.test(s)) {
      alpha = parseInt(s.slice(7, 9), 16) / 255;
      s = s.slice(0, 7);
    } else if (/^#[0-9a-f]{4}$/.test(s)) {
      alpha = parseInt(s[4] + s[4], 16) / 255;
      s = "#" + s[1] + s[1] + s[2] + s[2] + s[3] + s[3];
    } else if (/^#[0-9a-f]{3}$/.test(s)) {
      s = "#" + s[1] + s[1] + s[2] + s[2] + s[3] + s[3];
    }
    const v = this.themeVars.get(s);
    return { color: v ? `var(--${v}, ${s})` : s, alpha };
  }
}
