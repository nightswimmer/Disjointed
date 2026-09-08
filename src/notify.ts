/**
 * Toast notifications — the app's replacement for `window.alert`.
 *
 * `notify(message, kind?)` slides a card into the stack at the top of the canvas
 * area; it auto-dismisses after a few seconds (longer for errors, and the timer
 * pauses while hovered), or on click / the ✕ button. Non-blocking: callers that
 * used to `alert(...)` then `return` behave the same, the user just isn't stopped.
 *
 * Kinds: "warn" (default — an action couldn't be done / needs a different
 * selection), "error" (something failed: a file didn't load), "info" (a neutral
 * result, e.g. an import summary).
 */

export type NotifyKind = "warn" | "error" | "info";

const AUTO_DISMISS_MS: Record<NotifyKind, number> = { warn: 6000, error: 9000, info: 5000 };
const MAX_VISIBLE = 4;

const ICONS: Record<NotifyKind, string> = {
  warn: '<svg viewBox="0 0 20 20" aria-hidden="true"><path d="M10 3 L17.5 16.5 L2.5 16.5 Z"/><line x1="10" y1="8" x2="10" y2="12.3"/><circle cx="10" cy="14.4" r="0.9" fill="currentColor" stroke="none"/></svg>',
  error: '<svg viewBox="0 0 20 20" aria-hidden="true"><circle cx="10" cy="10" r="7"/><line x1="7.3" y1="7.3" x2="12.7" y2="12.7"/><line x1="12.7" y1="7.3" x2="7.3" y2="12.7"/></svg>',
  info: '<svg viewBox="0 0 20 20" aria-hidden="true"><circle cx="10" cy="10" r="7"/><line x1="10" y1="9" x2="10" y2="13.5"/><circle cx="10" cy="6.4" r="0.9" fill="currentColor" stroke="none"/></svg>',
};

let stack: HTMLElement | null = null;

function ensureStack(): HTMLElement {
  if (stack && stack.isConnected) return stack;
  stack = document.getElementById("toast-stack");
  if (!stack) {
    stack = document.createElement("div");
    stack.id = "toast-stack";
    (document.getElementById("canvas-wrap") ?? document.body).appendChild(stack);
  }
  stack.setAttribute("aria-live", "polite");
  return stack;
}

/** Show a toast. Returns a function that dismisses it early. */
export function notify(message: string, kind: NotifyKind = "warn"): () => void {
  const host = ensureStack();

  const el = document.createElement("div");
  el.className = `toast toast-${kind}`;
  el.setAttribute("role", kind === "error" ? "alert" : "status");
  el.innerHTML =
    `<span class="toast-icon">${ICONS[kind]}</span>` +
    `<span class="toast-text"></span>` +
    `<button type="button" class="toast-close" aria-label="Dismiss">` +
    `<svg viewBox="0 0 20 20" aria-hidden="true"><line x1="6" y1="6" x2="14" y2="14"/><line x1="14" y1="6" x2="6" y2="14"/></svg>` +
    `</button>`;
  el.querySelector<HTMLElement>(".toast-text")!.textContent = message;

  let closed = false;
  let timer: ReturnType<typeof setTimeout> | null = null;
  const dismiss = (): void => {
    if (closed) return;
    closed = true;
    if (timer !== null) clearTimeout(timer);
    el.classList.add("toast-out");
    // Remove after the exit transition; the fallback guards against a missed transitionend.
    const done = (): void => el.remove();
    el.addEventListener("transitionend", done, { once: true });
    setTimeout(done, 400);
  };
  const arm = (): void => {
    if (timer !== null) clearTimeout(timer);
    timer = setTimeout(dismiss, AUTO_DISMISS_MS[kind]);
  };

  el.addEventListener("click", dismiss);
  el.addEventListener("mouseenter", () => {
    if (timer !== null) clearTimeout(timer);
    timer = null;
  });
  el.addEventListener("mouseleave", arm);

  // Keep the stack short: drop the oldest when it overflows.
  const live = host.querySelectorAll<HTMLElement>(".toast:not(.toast-out)");
  if (live.length >= MAX_VISIBLE) live[0].click();

  host.appendChild(el);
  // Two frames so the initial (hidden) style is committed before the enter transition.
  requestAnimationFrame(() => requestAnimationFrame(() => el.classList.add("toast-in")));
  arm();
  return dismiss;
}
