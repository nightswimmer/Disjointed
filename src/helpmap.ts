/**
 * Help topics: which section of the manual (public/help/index.html) each piece of the UI
 * opens in "what's this" mode. Pure data + a DOM-free resolver, so the manual generator
 * (scripts/manual) can import it in Node to verify that every topic the app can ask for
 * actually exists in the manual.
 *
 * Resolution walks from the clicked element up to the document and takes the first hit:
 *   1. an explicit `data-help="topic"` attribute,
 *   2. a tool button (`data-tool`) → `tool-<name>`, a mode button (`data-mode`) → `mode-<name>`,
 *   3. an element id listed in ID_TOPICS (buttons, fields, panels, the canvas),
 *   4. a toolbar group id listed in GROUP_TOPICS (fields that have no id of their own),
 *   5. the table of contents.
 */

export const TOC_TOPIC = "toc";

/** Element ids → topics. */
export const ID_TOPICS: Record<string, string> = {
  // toolbar
  "run-btn": "animation",
  "autopause-btn": "animation",
  "combine-btn": "combine",
  "mirror-h-btn": "mirror",
  "mirror-v-btn": "mirror",
  "send-back-btn": "stacking-order",
  "bring-front-btn": "stacking-order",
  "make-comp-btn": "components",
  "comp-panel-btn": "component-browser",
  "sketch-vis-btn": "constraints",
  "grid-btn": "grid",
  "snap-btn": "grid",
  "osnap-btn": "object-snap",
  "grid-size-btn": "grid",
  "grid-size-menu": "grid",
  "measure-vis-btn": "dimensions",
  "fit-btn": "view",
  "rotate-view-btn": "view",
  "save-btn": "files",
  "export-btn": "export",
  "backup-btn": "backup",
  "load-btn": "files",
  "clear-btn": "files",
  "theme-btn": "theme",
  "unit-select": "units",
  "body-color": "body-colour",
  "actuator-speed": "tool-linearActuator",
  "actuator-profile": "tool-linearActuator",
  "motor-speed": "tool-motor",
  "poly-sides": "tool-polygon",
  "text-size": "tool-text",
  "text-edit": "tool-text",
  "anim-iter-ctrl": "solver-tuning",
  "cleanup-max-ctrl": "solver-tuning",
  "struct-tol-ctrl": "solver-tuning",
  "break-tol-ctrl": "solver-tuning",
  "help-btn": TOC_TOPIC,
  // canvas overlays
  "scene": "canvas",
  "sim-error": "impossible",
  "crumb-bar": "component-context",
  "comp-panel": "component-browser",
  "export-panel": "export",
  "backup-panel": "backup",
  "dim-edit": "dimensions",
  "view-angle-edit": "view",
  "hint": "canvas",
};

/** Toolbar group ids → topics (for controls inside them that carry no id). */
export const GROUP_TOPICS: Record<string, string> = {
  "mode-group": "modes",
  "tool-group": "tools",
  "edit-group": "editing",
  "color-group": "body-colour",
  "actuator-group": "tool-linearActuator",
  "sketch-group": "constraints",
  "component-group": "components",
  "actuator-props": "tool-linearActuator",
  "motor-props": "tool-motor",
  "role-group": "roles",
  "shape-props": "tools",
};

/** One ancestor on the way up from the clicked element (DOM-free description). */
export interface ChainNode {
  id?: string;
  classes: string[];
  data: Record<string, string | undefined>;
  tag: string;
}

export function topicForChain(chain: ChainNode[]): string {
  for (const n of chain) if (n.data.help) return n.data.help;
  for (const n of chain) {
    if (n.data.tool && n.classes.includes("tool-btn")) return `tool-${n.data.tool}`;
    if (n.data.mode && n.classes.includes("mode-btn")) return `mode-${n.data.mode}`;
    if (n.id && ID_TOPICS[n.id]) return ID_TOPICS[n.id];
  }
  for (const n of chain) if (n.id && GROUP_TOPICS[n.id]) return GROUP_TOPICS[n.id];
  return TOC_TOPIC;
}

/**
 * Topics for things drawn on the canvas (main.ts's `canvasTopicAt` resolves a click in
 * help mode to one of these, using the same hit tests as selection).
 */
export const CANVAS_TOPICS = [
  "el-body",
  "el-grounded-body",
  "el-group",
  "el-hole",
  "el-handles",
  "el-joint",
  "el-free-joint",
  "el-pin",
  "el-weld",
  "el-ground",
  "el-rail",
  "el-rider",
  "el-slider",
  "el-actuator",
  "el-motor",
  "el-guide",
  "el-dimension",
  "el-context-dimension",
  "el-constraint",
  "el-pattern",
  "el-instance",
  "el-ghost",
] as const;
export type CanvasTopic = (typeof CANVAS_TOPICS)[number];

/** Every topic the resolver can produce for the given tool / mode names. */
export function allTopics(tools: string[], modes: string[]): string[] {
  const set = new Set<string>([TOC_TOPIC, ...Object.values(ID_TOPICS), ...Object.values(GROUP_TOPICS), ...CANVAS_TOPICS]);
  for (const t of tools) set.add(`tool-${t}`);
  for (const m of modes) set.add(`mode-${m}`);
  return [...set].sort();
}
