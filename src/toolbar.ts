/**
 * Toolbar section rack: the sections of the toolbar (#tb-sections > .tb-sec) can be
 * dragged into any order by their caption strip, and the order is remembered.
 *
 * The drag is pointer-based rather than HTML5 drag-and-drop. The dragged section follows
 * the cursor on a transform while staying in the flow, so its slot is the hole it will
 * drop into; the moment the cursor passes another section's midpoint the DOM is reordered
 * for real and the sections that moved are animated from where they were to where they now
 * are (a FLIP), so the rack always shows the order you would get by letting go.
 *
 * Insertion is decided on `offsetLeft` / `offsetTop`, which are layout positions and so
 * ignore both the dragged section's transform and any FLIP animation still running.
 *
 * Order lives in localStorage as a list of section ids. Sections the saved list does not
 * know about (a new section in a later version) are slotted back next to the neighbour
 * they have in the markup, so adding one never forces a reset.
 */

const ORDER_KEY = "disjointed:toolbarOrder";
/** Pointer travel before a press on a caption becomes a drag. */
const DRAG_PX = 4;
/** How long a section takes to slide into its new place. */
const SLIDE_MS = 150;

export interface ToolbarApi {
  /** Put the sections back in their markup order and forget the saved one. */
  resetOrder(): void;
}

export interface ToolbarOptions {
  /** While this returns true the rack is inert (help mode swallows clicks itself). */
  blocked?: () => boolean;
  /** Called after a double-click on a caption restored the default order. */
  onReset?: () => void;
}

export function installToolbar(opts: ToolbarOptions = {}): ToolbarApi {
  const blocked = opts.blocked ?? (() => false);
  const found = document.getElementById("tb-sections");
  if (!found) return { resetOrder: () => undefined };
  const rack: HTMLElement = found; // hoisted helpers below need the narrowed type

  const sections = (): HTMLElement[] => [...rack.querySelectorAll<HTMLElement>(".tb-sec")];
  const visible = (): HTMLElement[] => sections().filter((s) => !s.classList.contains("hidden"));
  const defaultOrder = sections().map((s) => s.id);

  for (const cap of rack.querySelectorAll<HTMLElement>(".tb-cap")) {
    cap.title = "Drag to move this group along the toolbar — double-click to restore the default order";
  }

  function readOrder(): string[] {
    try {
      const raw = localStorage.getItem(ORDER_KEY);
      const list = raw ? (JSON.parse(raw) as unknown) : null;
      if (!Array.isArray(list)) return defaultOrder;
      return mergeOrder(list.filter((id): id is string => typeof id === "string"));
    } catch {
      return defaultOrder;
    }
  }

  /** Saved order first; anything it doesn't mention goes back beside its markup neighbour. */
  function mergeOrder(saved: string[]): string[] {
    const known = new Set(defaultOrder);
    const out = saved.filter((id, i) => known.has(id) && saved.indexOf(id) === i);
    for (const id of defaultOrder) {
      if (out.includes(id)) continue;
      let at = 0;
      for (let k = defaultOrder.indexOf(id) - 1; k >= 0; k--) {
        const p = out.indexOf(defaultOrder[k]);
        if (p >= 0) {
          at = p + 1;
          break;
        }
      }
      out.splice(at, 0, id);
    }
    return out;
  }

  function applyOrder(ids: string[]): void {
    for (const id of ids) {
      const el = document.getElementById(id);
      if (el) rack.appendChild(el); // re-appending in order is the reorder
    }
  }

  function saveOrder(): void {
    try {
      localStorage.setItem(ORDER_KEY, JSON.stringify(sections().map((s) => s.id)));
    } catch {
      /* storage unavailable — the order is then session-only */
    }
  }

  applyOrder(readOrder());

  // --- dragging -----------------------------------------------------------------

  /**
   * The section the dragged one should sit before (null = last), from the cursor in
   * client coordinates. Rows are compared before columns so a wrapped rack behaves.
   */
  function dropBefore(cx: number, cy: number, others: HTMLElement[]): HTMLElement | null {
    const rackRect = rack.getBoundingClientRect();
    const x = cx - rackRect.left;
    const y = cy - rackRect.top;
    for (const s of others) {
      if (y < s.offsetTop) return s; // the cursor is above this row entirely
      if (y <= s.offsetTop + s.offsetHeight && x < s.offsetLeft + s.offsetWidth / 2) return s;
    }
    return null;
  }

  rack.addEventListener("pointerdown", (e: PointerEvent) => {
    if (e.button !== 0 || blocked()) return;
    const target = e.target instanceof Element ? e.target : null;
    if (target?.closest("button, input, select, a")) return; // the caption's own eye toggle
    const sec = target?.closest<HTMLElement>(".tb-cap")?.closest<HTMLElement>(".tb-sec");
    if (!sec) return;
    e.preventDefault();

    const startX = e.clientX;
    const startY = e.clientY;
    const grab = sec.getBoundingClientRect();
    const grabX = startX - grab.left;
    const grabY = startY - grab.top;
    let dragging = false;

    /** Park the dragged section under the cursor, wherever its slot has ended up. */
    const lift = (cx: number, cy: number): void => {
      sec.style.transform = "";
      const slot = sec.getBoundingClientRect();
      sec.style.transform = `translate(${cx - grabX - slot.left}px, ${cy - grabY - slot.top}px)`;
    };

    /** Move the section in the DOM and slide everything that shifted to its new place. */
    const reorder = (before: HTMLElement | null): void => {
      const rest = sections().filter((s) => s !== sec);
      const was = new Map(rest.map((s) => [s, s.getBoundingClientRect()])); // where they look now
      for (const s of rest) {
        s.style.transition = "none";
        s.style.transform = "";
      }
      rack.insertBefore(sec, before);
      const moved: HTMLElement[] = [];
      for (const s of rest) {
        const now = s.getBoundingClientRect(); // pure layout: transforms are cleared
        const dx = (was.get(s)?.left ?? now.left) - now.left;
        const dy = (was.get(s)?.top ?? now.top) - now.top;
        if (!dx && !dy) continue;
        s.style.transform = `translate(${dx}px, ${dy}px)`;
        moved.push(s);
      }
      requestAnimationFrame(() => {
        for (const s of moved) {
          s.style.transition = `transform ${SLIDE_MS}ms ease`;
          s.style.transform = "";
        }
      });
    };

    const move = (ev: PointerEvent) => {
      if (!dragging) {
        if (Math.hypot(ev.clientX - startX, ev.clientY - startY) < DRAG_PX) return;
        dragging = true;
        sec.style.transition = "none"; // it follows the cursor exactly, settle comes later
        sec.classList.add("tb-dragging");
        document.body.classList.add("tb-dragging-on");
      }
      const before = dropBefore(ev.clientX, ev.clientY, visible().filter((s) => s !== sec));
      if (before !== sec.nextElementSibling) reorder(before);
      lift(ev.clientX, ev.clientY);
    };

    const up = () => {
      window.removeEventListener("pointermove", move);
      window.removeEventListener("pointerup", up);
      window.removeEventListener("pointercancel", up);
      if (!dragging) return;
      document.body.classList.remove("tb-dragging-on");
      // Let it settle into its slot before it stops being the floating one.
      sec.style.transition = `transform ${SLIDE_MS}ms ease`;
      sec.style.transform = "";
      window.setTimeout(() => {
        for (const s of sections()) {
          s.style.transition = "";
          s.style.transform = "";
        }
        sec.classList.remove("tb-dragging");
      }, SLIDE_MS);
      saveOrder();
    };

    window.addEventListener("pointermove", move);
    window.addEventListener("pointerup", up);
    window.addEventListener("pointercancel", up);
  });

  function resetOrder(): void {
    try {
      localStorage.removeItem(ORDER_KEY);
    } catch {
      /* storage unavailable */
    }
    applyOrder(defaultOrder);
  }

  rack.addEventListener("dblclick", (e) => {
    if (blocked()) return;
    const target = e.target instanceof Element ? e.target : null;
    if (!target?.closest(".tb-cap") || target.closest("button, input, select, a")) return;
    resetOrder();
    opts.onReset?.();
  });

  return { resetOrder };
}
