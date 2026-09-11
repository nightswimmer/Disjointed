/**
 * File-system access for save / load / backups.
 *
 * Chromium browsers (Chrome, Edge, Opera…) expose the **File System Access API**: the
 * page can hold a handle to a real file or folder and write to it again later — so
 * Ctrl+S overwrites *the* file instead of downloading a new copy, and periodic backups
 * can land in a folder of the user's choosing. Firefox / Safari don't have it; callers
 * check `fsSupported()` and fall back to a plain download / `<input type="file">`.
 *
 * Handles are structured-cloneable, so they survive a reload when stashed in IndexedDB
 * (`storeHandle` / `loadHandle`). The *permission* to use them does not necessarily
 * survive: the browser may come back with "prompt", in which case `requestPermission`
 * has to be called from a user gesture (a click / keypress) before writing.
 *
 * The API is not in TypeScript's lib.dom yet, so the minimal surface used here is
 * declared below and looked up on `window` by name.
 */

export type FSPermission = "granted" | "denied" | "prompt";

export interface FSWritable {
  write(data: string | Blob): Promise<void>;
  close(): Promise<void>;
}

export interface FSHandle {
  readonly kind: "file" | "directory";
  readonly name: string;
  queryPermission(opts?: { mode?: "read" | "readwrite" }): Promise<FSPermission>;
  requestPermission(opts?: { mode?: "read" | "readwrite" }): Promise<FSPermission>;
  isSameEntry(other: FSHandle): Promise<boolean>;
}

export interface FSFileHandle extends FSHandle {
  readonly kind: "file";
  getFile(): Promise<File>;
  createWritable(): Promise<FSWritable>;
}

export interface FSDirectoryHandle extends FSHandle {
  readonly kind: "directory";
  getFileHandle(name: string, opts?: { create?: boolean }): Promise<FSFileHandle>;
  removeEntry(name: string, opts?: { recursive?: boolean }): Promise<void>;
  values(): AsyncIterable<FSFileHandle | FSDirectoryHandle>;
}

interface FilePickerType {
  description?: string;
  accept: Record<string, string[]>;
}

interface PickerWindow {
  showSaveFilePicker?(opts?: { suggestedName?: string; id?: string; types?: FilePickerType[] }): Promise<FSFileHandle>;
  showOpenFilePicker?(opts?: { multiple?: boolean; id?: string; types?: FilePickerType[] }): Promise<FSFileHandle[]>;
  showDirectoryPicker?(opts?: { id?: string; mode?: "read" | "readwrite" }): Promise<FSDirectoryHandle>;
}

const win = window as unknown as PickerWindow;

const JSON_TYPE: FilePickerType = { description: "Disjointed mechanism", accept: { "application/json": [".json"] } };

/** True when the browser can hand out writable file / folder handles. */
export function fsSupported(): boolean {
  return typeof win.showSaveFilePicker === "function" && typeof win.showOpenFilePicker === "function";
}

/** True when the browser can hand out folder handles (needed for auto-backups). */
export function fsDirectorySupported(): boolean {
  return typeof win.showDirectoryPicker === "function";
}

/** A picker dismissed by the user rejects with an AbortError — that's a "no", not a failure. */
function isAbort(err: unknown): boolean {
  return err instanceof DOMException && err.name === "AbortError";
}

/** Ask where to save a `.json`; `null` when the user cancels. */
export async function pickSaveFile(suggestedName: string): Promise<FSFileHandle | null> {
  try {
    return await win.showSaveFilePicker!({ suggestedName, id: "disjointed-doc", types: [JSON_TYPE] });
  } catch (err) {
    if (isAbort(err)) return null;
    throw err;
  }
}

/** Ask for a `.json` to open; `null` when the user cancels. */
export async function pickOpenFile(): Promise<FSFileHandle | null> {
  try {
    const [h] = await win.showOpenFilePicker!({ multiple: false, id: "disjointed-doc", types: [JSON_TYPE] });
    return h ?? null;
  } catch (err) {
    if (isAbort(err)) return null;
    throw err;
  }
}

/** Ask for a folder with read/write access; `null` when the user cancels. */
export async function pickDirectory(): Promise<FSDirectoryHandle | null> {
  try {
    return await win.showDirectoryPicker!({ id: "disjointed-backups", mode: "readwrite" });
  } catch (err) {
    if (isAbort(err)) return null;
    throw err;
  }
}

/**
 * Make sure we may read+write through `handle`. With `request`, a "prompt" state asks
 * the user (only valid inside a user gesture); without it, "prompt" counts as not ok.
 */
export async function ensurePermission(handle: FSHandle, request: boolean): Promise<boolean> {
  const opts = { mode: "readwrite" as const };
  try {
    let state = await handle.queryPermission(opts);
    if (state === "prompt" && request) state = await handle.requestPermission(opts);
    return state === "granted";
  } catch {
    return false;
  }
}

/** Overwrite `handle` with `text`. */
export async function writeFile(handle: FSFileHandle, text: string): Promise<void> {
  const w = await handle.createWritable();
  try {
    await w.write(text);
  } finally {
    await w.close();
  }
}

/** Create (or overwrite) `name` inside `dir` with `text`. */
export async function writeToDirectory(dir: FSDirectoryHandle, name: string, text: string): Promise<void> {
  const h = await dir.getFileHandle(name, { create: true });
  await writeFile(h, text);
}

/** Names of the plain files directly inside `dir`. */
export async function listFiles(dir: FSDirectoryHandle): Promise<string[]> {
  const names: string[] = [];
  for await (const entry of dir.values()) if (entry.kind === "file") names.push(entry.name);
  return names;
}

/** The file-system handle behind a dropped item, when the browser can provide one. */
export function handleFromDrop(dt: DataTransfer | null): Promise<FSFileHandle | null> {
  const item = dt?.items?.[0] as (DataTransferItem & { getAsFileSystemHandle?(): Promise<FSHandle | null> }) | undefined;
  if (!item || typeof item.getAsFileSystemHandle !== "function") return Promise.resolve(null);
  return item
    .getAsFileSystemHandle()
    .then((h) => (h && h.kind === "file" ? (h as FSFileHandle) : null))
    .catch(() => null);
}

// --- handle persistence (IndexedDB) ------------------------------------------
const DB_NAME = "disjointed-fs";
const STORE = "handles";

function openDb(): Promise<IDBDatabase> {
  return new Promise((resolve, reject) => {
    const req = indexedDB.open(DB_NAME, 1);
    req.onupgradeneeded = () => req.result.createObjectStore(STORE);
    req.onsuccess = () => resolve(req.result);
    req.onerror = () => reject(req.error);
  });
}

/** Remember `handle` under `key` across reloads (`null` forgets it). Failures are ignored. */
export async function storeHandle(key: string, handle: FSHandle | null): Promise<void> {
  try {
    const db = await openDb();
    await new Promise<void>((resolve, reject) => {
      const tx = db.transaction(STORE, "readwrite");
      const st = tx.objectStore(STORE);
      if (handle) st.put(handle, key);
      else st.delete(key);
      tx.oncomplete = () => resolve();
      tx.onerror = () => reject(tx.error);
    });
    db.close();
  } catch {
    /* IndexedDB unavailable (private mode / storage disabled) — handles won't persist */
  }
}

/** The handle remembered under `key`, or `null`. Its permission may need re-granting. */
export async function loadHandle<T extends FSHandle>(key: string): Promise<T | null> {
  try {
    const db = await openDb();
    const h = await new Promise<T | null>((resolve, reject) => {
      const tx = db.transaction(STORE, "readonly");
      const req = tx.objectStore(STORE).get(key);
      req.onsuccess = () => resolve((req.result as T | undefined) ?? null);
      req.onerror = () => reject(req.error);
    });
    db.close();
    return h;
  } catch {
    return null;
  }
}
