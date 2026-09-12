/**
 * In-app help: the `?` toolbar button, the help drawer (a side panel showing the manual
 * from public/help/) and the "what's this" behaviour that comes with it.
 *
 * - The help button (or `?`) opens the drawer. While it is open, help mode is on: the
 *   cursor becomes a question mark and clicking any control jumps the manual to that
 *   control's topic instead of activating it. Clicking the canvas shows the topic for the
 *   armed tool (or the current mode). The help button, `?`, Esc or the drawer's close
 *   button end it; F1 opens the drawer at the table of contents.
 * - The drawer is a flex sibling of the canvas, so the canvas shrinks beside it (a
 *   `resize` event is dispatched so main.ts re-fits the backing store). Its width is
 *   draggable and remembered. The manual page follows the app theme: the drawer forwards
 *   the `data-theme` attribute of <html> through postMessage.
 *
 * The topic for a clicked element comes from src/helpmap.ts (shared with the manual
 * generator so the two can't drift apart).
 */
import { ChainNode, topicForChain, TOC_TOPIC } from "./helpmap";

const WIDTH_KEY = "disjointed:helpWidth";
const MIN_WIDTH = 300;
const DEFAULT_WIDTH = 440;
/** Relative URL of the manual (served from public/ by Vite; `base` is "./"). */
const MANUAL_URL = "help/index.html";

export interface HelpApi {
  /** Open the drawer at a topic (the last one shown by default). */
  open(topic?: string): void;
  close(): void;
  toggle(): void;
  isOpen(): boolean;
}

export interface HelpHost {
  /** Topic for the element drawn at a canvas point (canvas-relative CSS px). */
  canvasTopic(at: { x: number; y: number }): string;
}

export function installHelp(host: HelpHost): HelpApi {
  const btn = document.getElementById("help-btn") as HTMLButtonElement;
  const drawer = document.getElementById("help-drawer") as HTMLElement;
  const wrap = document.getElementById("canvas-wrap") as HTMLElement;
  const frame = document.getElementById("help-frame") as HTMLIFrameElement;
  const popout = document.getElementById("help-popout") as HTMLAnchorElement;
  const closeBtn = document.getElementById("help-close") as HTMLButtonElement;
  const grip = document.getElementById("help-grip") as HTMLElement;

  let open = false;
  let frameReady = false;
  let pendingTopic: string | null = null;
  let currentTopic = TOC_TOPIC;
  const idleTitle = btn.title;

  // --- drawer --------------------------------------------------------------------
  let width = (() => {
    try {
      const n = Number(localStorage.getItem(WIDTH_KEY));
      return Number.isFinite(n) && n >= MIN_WIDTH ? n : DEFAULT_WIDTH;
    } catch {
      return DEFAULT_WIDTH;
    }
  })();
  function applyWidth(): void {
    const max = Math.max(MIN_WIDTH, Math.floor(wrap.clientWidth * 0.7));
    width = Math.min(Math.max(MIN_WIDTH, width), max);
    drawer.style.width = `${width}px`;
    wrap.style.setProperty("--help-w", `${width}px`);
  }
  function relayout(): void {
    // The canvas keeps its own backing store in step with its CSS size on window resize;
    // the drawer changing the layout is the same situation. Any stray scroll (the page
    // itself never scrolls by design) would leave a blank strip, so undo it too.
    window.scrollTo(0, 0);
    window.dispatchEvent(new Event("resize"));
  }
  function theme(): string {
    return document.documentElement.dataset.theme ?? "dark";
  }
  function post(msg: Record<string, unknown>): void {
    frame.contentWindow?.postMessage(msg, "*");
  }
  function goto(topic: string): void {
    currentTopic = topic;
    popout.href = `${MANUAL_URL}#${topic}`;
    if (!frame.getAttribute("src")) {
      pendingTopic = topic;
      frame.src = `${MANUAL_URL}?theme=${theme()}#${topic}`;
    } else if (frameReady) {
      post({ type: "goto", topic });
    } else {
      pendingTopic = topic;
    }
  }
  function setOpen(on: boolean): void {
    if (open === on) return;
    open = on;
    drawer.classList.toggle("hidden", !on);
    wrap.classList.toggle("help-open", on);
    document.body.classList.toggle("help-armed", on);
    btn.classList.toggle("active", on);
    btn.title = on ? "Close help (?)" : idleTitle;
    if (on) applyWidth();
    relayout();
  }
  function show(topic: string): void {
    setOpen(true);
    goto(topic);
  }
  frame.addEventListener("load", () => {
    frameReady = true;
    post({ type: "theme", theme: theme() });
    if (pendingTopic) {
      post({ type: "goto", topic: pendingTopic });
      pendingTopic = null;
    }
  });
  // Keep the manual on the app's theme.
  new MutationObserver(() => post({ type: "theme", theme: theme() })).observe(document.documentElement, {
    attributes: true,
    attributeFilter: ["data-theme"],
  });
  closeBtn.addEventListener("click", () => setOpen(false));
  btn.addEventListener("click", () => (open ? setOpen(false) : show(currentTopic)));
  window.addEventListener("resize", () => {
    if (open) applyWidth();
  });

  // Drag the grip to resize.
  grip.addEventListener("mousedown", (e) => {
    if (e.button !== 0) return;
    e.preventDefault();
    const startX = e.clientX;
    const startW = width;
    frame.style.pointerEvents = "none"; // the iframe would swallow the move events
    const onMove = (ev: MouseEvent) => {
      width = startW + (startX - ev.clientX);
      applyWidth();
      relayout();
    };
    const onUp = () => {
      window.removeEventListener("mousemove", onMove);
      window.removeEventListener("mouseup", onUp);
      frame.style.pointerEvents = "";
      try {
        localStorage.setItem(WIDTH_KEY, String(width));
      } catch {
        /* storage unavailable */
      }
    };
    window.addEventListener("mousemove", onMove);
    window.addEventListener("mouseup", onUp);
  });

  // --- "what's this" while open -------------------------------------------------------
  function chainOf(el: Element | null): ChainNode[] {
    const chain: ChainNode[] = [];
    for (let n: Element | null = el; n && n !== document.documentElement; n = n.parentElement) {
      const data: Record<string, string | undefined> = {};
      if (n instanceof HTMLElement) for (const k of Object.keys(n.dataset)) data[k] = n.dataset[k];
      chain.push({ id: n.id || undefined, classes: [...n.classList], data, tag: n.tagName.toLowerCase() });
    }
    return chain;
  }
  function topicFor(e: MouseEvent, target: Element | null): string {
    const canvas = target?.closest("#scene");
    if (canvas) {
      const r = canvas.getBoundingClientRect();
      return host.canvasTopic({ x: e.clientX - r.left, y: e.clientY - r.top });
    }
    return topicForChain(chainOf(target));
  }
  function inHelpUi(target: Element | null): boolean {
    return !!target && (!!target.closest("#help-drawer") || target === btn || !!btn.contains(target));
  }

  // Swallow the whole click gesture aimed at a control while help is open - in the
  // capture phase at the window, before the control's own listener (buttons act on
  // click, the canvas on mousedown) can run - and show the topic instead.
  let swallowing = false;
  const onPointer = (e: MouseEvent) => {
    if (!open) return;
    const target = e.target instanceof Element ? e.target : null;
    if (inHelpUi(target)) return; // the help button / drawer keep working normally
    if (e.type === "mousedown") {
      if (e.button !== 0) return; // right button pans the view; let it be
      e.preventDefault();
      e.stopPropagation();
      swallowing = true;
      goto(topicFor(e, target));
    } else if (swallowing) {
      e.preventDefault();
      e.stopPropagation();
      if (e.type === "click") swallowing = false;
    }
  };
  for (const type of ["mousedown", "mouseup", "click", "dblclick"] as const) {
    window.addEventListener(type, onPointer, true);
  }
  window.addEventListener(
    "keydown",
    (e) => {
      if (open && e.key === "Escape") {
        e.preventDefault();
        e.stopPropagation();
        setOpen(false);
      }
    },
    true
  );

  return {
    open: (topic = currentTopic) => show(topic),
    close: () => setOpen(false),
    toggle: () => (open ? setOpen(false) : show(currentTopic)),
    isOpen: () => open,
  };
}
