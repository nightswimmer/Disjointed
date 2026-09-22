# Handoff — open work

What has been **discussed and decided but not built yet**. Read with PROJECT_INSTRUCTIONS.md
(current state); this file holds only the unfinished part, so a future session can pick a piece
up without re-doing the brainstorm. In order: the rest of the **shortcut remap** (next up), the
**shape tools + roles** roadmap (phase 1 shipped 2026-09-13, format v20), the Tangential
follow-up, the sim-mode toolbar, and the **pending manual updates** list.

## The rest of the shortcut remap

The 2026-09-15 remap is **applied and shipped** — Fixed `F`, Circle `C`, Line `L`, Arc `A`,
Fit view `Shift+F`, Rotate view `Ctrl+R`, Polyline (cut) `Ctrl+U`, Connect `Ctrl+Shift+C`,
Linear actuator `Shift+A`, Motor `Shift+M`, Rail unassigned. What is left:

- **Rail lost its key and nothing took `K`.** The board did that; the earlier plan had *Linear
  actuator* as the one to drop. Ask before assuming it was meant — `K` is free either way.
- **Connect is now `Ctrl+Shift+C`**, a three-key chord for a tool used constantly. Worth a
  second look on the board.
- **23 commands have no key at all** (`public/keymap.json`, empty `bindings`): Export, the
  mirrors, the grid / snap / badge / measurement toggles, zoom in / out, theme, Create component,
  Component browser, Auto-pause, Delete everything, the two grid menus, Rail, and the Shortcuts
  panel itself. All have a working `run()`, so the board can hand any of them a letter and it
  works. Subtract and Intersect are tagged `planned` and do nothing until the boolean ops exist.

### Working with the board

**Seed KeyMapper from `public/keymap.json`, never from an older capture.** The board's original
file was hand-written on 2026-09-14 by reading the old `if`-ladder (`"source":
"src/main.ts:8005-8283"`), and its ids were its own invention — `save`, `polybody`, `refline`,
`radplus`, `sidesup`. None of them match a registry id, so importing that file applies *nothing*
(the panel says "0 commands; 80 the app doesn't know, ignored"). The file in the repo carries
the real ids; load that, rearrange, bring it back — it is also what ships, so the round trip is
the whole job now. The artifact is
`cc79ffcf-efa4-4d38-9bd7-e2aade7901a4`.

### How to apply a layout

1. **To ship it:** drop the board's export in as `public/keymap.json` — that file *is* the
   shipped keymap since 2026-09-15, so nothing in `src/` is edited for a remap. Then
   `npm run keymap:docs --readme` (the manual is deferred — see the pending-updates section).
   Bring the letters in `scripts/keymap-live.ts` into line — they are written out by hand on
   purpose — and run `npm run keymap:live`. Also sweep README prose *outside* the tools table:
   the generator only owns the Shortcut column, and the Navigate / Help sections name keys in
   sentences. `npm test` fails if the file and the registry disagree about which commands exist,
   if the tooltips or the README table are behind, or if two commands want one slot in
   overlapping contexts, so a clash cannot ship. `npm run keymap:file` is only needed after
   *adding a command*: it rewrites the file's metadata and keeps every binding.
2. **To try one:** import it through the Shortcuts panel (File group). That writes
   `localStorage["disjointed:keymap"]` for this browser only; Reset undoes it.

An imported keymap that clashes is **accepted, not refused** — the toast counts the clashes, and
on a clashing slot the first command in registry order answers. Deliberate: refusing a file is
worse than showing the user what they did.

### Watch out

- **Ids are the contract.** Renaming a `CommandSpec.id` silently drops that command's binding
  from every saved keymap. Rename labels freely, never ids. (In `public/keymap.json` a rename is
  no longer silent — `npm test` reports the old id as one the app doesn't have — but the binding
  is lost all the same.)
- **Spelling rules are shared with KeyMapper** and specified in its README, not here: Shift is
  part of a letter's or named key's slot and never part of a character's (`?` arrives as `"?"`
  with shiftKey true). `src/keymap.ts` implements them; a change has to happen in both projects.
- Still out of scope, as decided: chords / key sequences, predicates expressed in the file (they
  stay in code as `enabled()`), and rebinding pointer gestures.

## Shape tools & roles — decisions already made (don't re-open)

- **Roles are explicit and sticky** (toolbar switch, keys 1 / 2 / 3), never inferred from
  where a click lands: bodies routinely overlap other bodies, so "first click inside a body
  = cut" is wrong. Mode errors are mitigated instead by the role-styled preview (tint /
  hatching + target highlight / dash-dot), the badge beside the cursor, and the one-shot
  **Ctrl on the first click** flip between Body and Cut.
- **Hole became Cut**: a polygon difference. Inside the material → a plain editable hole
  (exact spec, a disk stays parametric); crossing the outline → a notch; a cut that would
  sever the body is refused (use Split). Target = selected body, else topmost body under the
  first click, else the body the finished shape lies in.
- **Line, arc and text are reference-only** for now. Slots in the Reference role draw the
  axis segment only.
- **Text is for labels first** (anchored to a body when clicked on one; otherwise free).
  Engraving / cutting text is a later phase.
- Curved slots are a nice-to-have, not a priority.

## Not built yet — agreed phases, in order

### Phase 2 — arcs (and ellipses) on outlines
- Add a per-edge **bulge** (DXF convention `tan(sweep/4)`, already what the exporter writes
  and the importer reads) to body / hole control polygons, so an edge can be a true arc.
  Touches: fillet derivation, vertex/pose solvers (arc edges as constraint refs), hit-testing,
  export (exact `A` / bulge output), the DXF importer (stop sampling non-tangent / ≥180° arcs).
- Then: an **Arc edge** in the Body / Cut roles, **curved slots** (arc axis + width), D-shapes.
- **Ellipses**: not an arc — the edge record needs a *kind*, not just a bulge. Keep the edge
  representation open for that when bulges land. DXF has an ELLIPSE entity; export as such or
  sampled.
- Reference arcs / circles already exist (v20) and could become the pick targets for "arc
  through these reference points".

### Phase 3 — regions from reference segments
- Detect **closed chains** of reference segments / arcs (the DXF importer's LINE/ARC
  auto-chaining is reusable) and let the user click inside a region to make a body or a cut
  from it (sketch-profile workflow). Would make Line / Arc useful in the material roles.

### Phase 4 — cut / engraved text
- Needs an outline-font library (e.g. opentype.js) and a bundled **stencil** face: under the
  loop-nesting rule an island inside a hole starts a new solid, so the middle of an "O" cut
  into a plate would become loose material. Text as a *body* has the same problem plus the
  one-outline-per-body model (letters are disjoint → one body per letter, or welded).
- Engraving (a non-cutting layer for the laser) would need an export layer concept.

## Smaller follow-ups from phase 1

- **Role conversions**: promote a closed reference polygon / circle to a body; extract a hole
  as a body of the same shape (a peg for a hole); use a body as a cutter on another body
  (difference of two bodies). All three make the initial role choice non-binding.
- **Cut several bodies at once** (a modifier on the closing click); today one target.
- **General-path cuts dissolve the body's patterns** (hole indices are rebuilt). Keeping
  patterns whose seed and members are untouched would need index remapping.
- A rounded cutter that crosses the outline arrives **sampled** (polygonal notch corners);
  with bulge edges (phase 2) the notch could keep exact arcs.
- **Text**: no angle of its own (free labels are horizontal; anchored ones follow the body),
  a fixed font, no alignment options, width estimated (`0.58 × size × chars`) for hit tests.
  Labels don't travel with copy/paste or into component definitions (guides never did).
- **Reference geometry as constraint targets**: tangent shipped (2026-09-13 — the
  `guideCircle` ref covers reference circles and arcs, `disk` covers disk bodies and round
  holes). Still missing: point-on-circle, concentric, circle–circle tangency, and an arc's
  centre as a reference *point* (derived, not a solver var — a `guideCircle` ref resolves to
  it, but nothing can be made coincident with it).
- **Shortcuts** for the new tools were Shift+letter picks; the 2026-09-15 remap moved Circle to
  `C`, Line to `L` and Arc to `A`, leaving Shift+B rectangle, Shift+P polygon, Shift+S slot,
  Shift+T text. Each is one edit in `keys` in `src/commands.ts`, everything else generated.
- Manual: the new tool topics are text-only; gesture illustrations (before / mid / after)
  for the shape tools would fit the existing `scripts/manual/shots.ts` pipeline (the
  role-styled preview is exposed through `RenderInput.shapeDraft`).

## Tangential constraint — needs another pass (2026-09-13)

Shipped in the "Tangential constraint" commit (design in PROJECT_INSTRUCTIONS.md, tests in
`scripts/tangent-constraint.ts`), but the user's field tests with a scene of their own (two long
reference lines + a reference arc: the rounded end of a slot) found it still wanting. The
session closed before the specifics were written down — **ask what was seen first**, then
start from these known weak spots:

- **The arc is three points** (`a` / `m` / `b`), not centre / radius / angles. When a point is
  held, the others take a Newton step along the line's normal and the radius and sweep change
  as a side effect rather than being parameters the user controls. The structural fix is a
  real arc parametrisation in the sketch solver — either new variable kinds (centre, radius,
  start / end angle) or the regular-polygon route: keep the three points and add a coupling
  item that holds radius / sweep unless a size dimension says otherwise.
- **Who moves in a blend.** With everything free, a coincident that glues an arc end to a
  line end makes the *line* swing about the shared end (its far end moves) as much as the arc
  adapts — a 50/50 split by rank. CAD users expect the arc to absorb it. Options: a lower
  mobility preference for reference arcs, or the arc taking the whole turn whenever both are
  free guides (Fixed / H / V on the line already forces that).
- **No held side.** A circle dragged through its line re-attaches on the far side; a stored
  side (as a driving dimension keeps) would make it stick.
- **Coverage.** Circles are disks, round holes, reference circles and arcs only: no tangency
  to a body's rounded corner (the fillet arc), no circle–circle or arc–arc (G1 between two
  arcs), no point-on-circle / concentric.
- **Pinned-end detection** (`pinnedArcEnd`) recognises a coincident between an arc end and
  the line's end, its midpoint, or the line itself — not an arc end coincident with a joint
  that merely sits on the line.
- **Tuning.** Convergence rests on Newton probes with hand-set floors (0.1 · r for the arc,
  0.1 · L for the line) and a 0.5 rad cap per sweep. `sketchConfig.trace` prints the worst
  residual per sweep — run it on the field file before touching a number.

## Toolbar — next step: sim mode

The toolbar was regrouped into draggable groups **for draw mode only** (the harder half, by
agreement). Sim mode works and inherits the rack, but was not designed:

- Today it shows the mode-neutral groups (File, Mode, Grid, Measure, Snapping, View) plus
  **Animation** (run + auto-pause), with the four solver-tuning knobs in the properties strip.
  Per-mode visibility is the `draw-only` / `sim-only` class on a section, so adding or moving a
  sim group is markup, not code.
- Open questions: whether the solver knobs deserve their own (collapsible?) group rather than the
  properties strip; whether sim wants a selection / drag-behaviour group; whether the two modes
  should keep **separate saved orders** (one `disjointed:toolbarOrder` list today, shared).
- Not decided: whether the mode button should show the mode you are *in* instead of the one it
  switches *to* (the theme button's convention was followed).

## In-app manual — pending updates (UI-tweak batch, started 2026-09-13)

The in-app help (manual generator in `scripts/manual/`, what's-this drawer, shortcut list) is
**deliberately not updated** during the current run of UI-tweak commits. Every UI change that
touches something the manual describes gets a line here instead; a later session will apply
them all in one go. Add to this list as you go — one bullet per change, say which manual topic
/ illustration / shortcut entry is affected and what the new behaviour is.

**Start that pass with `npm run keymap:docs`.** The `#shortcuts` list in
`public/help/index.html` is now generated from the command registry, so every "add / drop the
`X` row of the shortcut list" note below is done by running it — do not hand-edit those lines,
and do not re-add a marker comment, the generator finds the `<ul>` by itself. It rewrites the
list as one line per command group (File, Edit, Mode, …, Help), which is a different shape from
today's hand-written eight lines: read the result before committing it, and if the grouping
reads badly, change `shortcutList` in `scripts/keymap-docs.ts` rather than the HTML. It also
fixes what is already wrong there — the list still offers `L` for the guideline tool, which was
retired. README.md's Shortcut column is generated by the same command and is already correct,
so running it changes nothing there.

- **The Shortcuts panel is new (2026-09-15).** The File group gained a keyboard button that
  opens Import / Export / Reset for the keymap. Manual: it needs a topic (and therefore a
  what's-this entry — `npm run manual` hard-fails on a missing one), covering the `keymap/1`
  file, the round trip through KeyMapper, that a keymap lives in the browser rather than in the
  mechanism file, and that about twenty commands ship with no key and can be given one. The
  README's new *Keyboard shortcuts* section is the text to adapt. Tooltips are now generated
  from the keymap, which is worth a line in the toolbar topic.

- **Infinite construction guidelines removed (2026-09-13).** The Guideline tool (toolbar
  button, key `L`, `guide` tool id) is gone; the finite **reference line** (Shift+L, `line`
  tool) replaces it. Manual: drop the "Guideline" tool topic / its what's-this entry and the
  `L` row of the shortcut list; rewrite every mention of "guideline" / "infinite construction
  line" (snap text, coincident / H / V / parallel / perpendicular / measure topics, the
  overview illustration caption "Bottom row: a guideline, …") to say reference line / reference
  geometry. Behaviour changes to document: snapping and hit-testing on a reference line are
  limited to its span (no more catching the cursor anywhere along an infinite line); `equal`
  and driving length dimensions now work on a reference line; dragging a reference-polyline
  point onto its neighbour is refused (the edge keeps a direction). The what's-this strings
  in `src/main.ts` (`measure`, `coincident`, `horizontal`, `vertical`, `parallel`,
  `perpendicular`) still say "guideline" — reword them in the same pass.
- **The shortcuts were remapped (2026-09-15).** The manual's whole `#shortcuts` list is stale;
  `npm run keymap:docs` rewrites it (see the note at the top of this section). Prose elsewhere in
  the manual that names a key has to be swept by hand: Fixed is `F`, Circle `C`, reference line
  `L`, reference arc `A`, fit view `Shift+F`, view dial `Ctrl+R`, polyline-as-cut `Ctrl+U`,
  Connect `Ctrl+Shift+C`, actuator `Shift+A`, motor `Shift+M`, and **Rail has no key at all**.
- **Regular polygons are now parametric (2026-09-13, format v21).** Manual: rewrite the
  Polygon tool topic — the side count is no longer a toolbar field: it shows beside the
  cursor badge while the tool is armed (↑ / ↓ change it) and, once the polygon exists, as
  an "n sides" tag above the selected polygon (double-click the tag to type a count, ↑ / ↓
  step it; `#poly-sides` and its label are gone from the toolbar). New behaviour to
  document: a polygon (or a polygon hole cut with the Cut role) stays regular — dragging a
  corner grows / spins it about its centre, nodes can't be added or removed by double-click
  (a toast says to change the count instead), the centre is drawn as a small crosshair and
  is a point reference for coincident / measure / object snap, and in the sketch the polygon
  is a rigid shape: its size is set by a *size dimension* (a chord between any two corners — the edge
  length included — centre → corner, centre → edge, or across flats between opposite
  edges); while such a dimension drives it, a corner drag only turns it; a line constraint on the
  polygon (H / V / parallel / perpendicular, or two of its points tied elsewhere) pins its
  rotation, so a corner drag then only resizes along that corner's radial line; with both
  pinned the polygon is a rigid object and a corner drag moves it whole. Other dimensions
  and constraints move or turn it whole (H / V / parallel on an edge turn it in one step,
  a 90° turn included). Split / combine turn it into a free polygon. The
  Polygon what's-this string in `src/main.ts` still says "Sides: the toolbar field" — reword
  in the same pass. A "convert to free polygon" action is planned later (not built).
- **Projected corner-pair sizes (2026-09-13).** Manual (Polygon / Measure topics): a
  dimension between two corners of a regular polygon whose label sits *between* the corners
  is a horizontal / vertical dimension (the Measure tool's placement rule). It is now a *size
  dimension* like a chord, whatever the sketch says about the polygon's rotation: driving it
  scales the polygon and keeps its rotation (a pentagon's height, apex to a base corner, works
  this way; a projection near zero, a chord across the axis, can't drive). Rotation is set
  only by drags and line constraints — the old "an h / v corner pair turns the polygon" rule
  is gone. A size the polygon can't reach, or one that disagrees with another driving size
  on the same polygon, is refused with the conflicting dimension flashed. Field repro:
  `dimensioning.json` (a hexagon with an "h" dimension on one side).
- **Single-click line dimensions (2026-09-13).** Manual (Measure topic + the `measure`
  what's-this string in `src/main.ts`, which still says "Click two references"): a single
  Measure click on a line — a body or hole edge, a reference-line segment, a rail — followed
  by a click on nothing places that line's own length: a plain dimension between the line's
  two ends (corners / reference points / rail joints), so the label-placement rule (above or
  below → horizontal size, beside → vertical, along → true length), driving, `equal`, and the
  regular-polygon size rule all apply exactly as if both ends had been clicked. While one
  line is picked, hovering empty space previews that dimension at the cursor; hovering
  another reference highlights it, and clicking it makes the usual line-to-point /
  line-to-line dimension. A bare body interior no longer counts as a reference for that
  second click (it places the label instead) — to dimension a line against an arbitrary
  point on a body, pick the point first, then the line. A pattern axis and a point-first
  pick keep the old behaviour (a click on nothing waits for a reference).

- **Implicit constraints keep two candidates (2026-09-13).** Manual (*Implicit constraints
  while dragging*, under Grid & snapping, plus the Horizontal / Vertical / Coincident
  constraint topics that mention being "offered implicitly while dragging"): a drag now keeps
  **two** armed alignment candidates instead of one, so a single drag can align a point
  vertically with one reference and horizontally with another and drop both constraints at
  once. New behaviour to document: hovering a third element drops the **oldest** candidate
  (not all of them); **Esc** drops the **most recent** one and the drag continues, so two
  presses clear both; each armed candidate previews its own dotted line + badge and the
  release places every previewed constraint, landing the geometry exactly on the
  intersection; two alignments that pull the same way (two horizontals, a horizontal and a
  horizontal reference line, …) can't both be exact, so only the more recently armed one
  previews and is placed while the other stays armed silently; a placement that can't be
  satisfied now only loses *that* constraint — anything else previewed is still placed.
  Also: a dragged point released **on** an armed candidate point (within the same ~10 px on
  both axes) now takes a **coincident** instead of nothing — the old "on top of it is a
  placement, not an alignment" rule is gone, and the ◎ preview already shows while you hold
  the point there to arm it. A point-on-point coincident spends both degrees of freedom, so it
  always previews and places alone. (This is the "auto-coincident while dragging" backlog item.)
  And the feature now has an on/off **switch**: `#autocon-btn`, the first control in the
  **Constraints** group — `.tb-switch`, a `tb-tall` pill with a slider under its glyph, styled
  so it can't be read as another constraint tool (`ID_TOPICS` points it at the existing
  `constraints` topic). Session state, default on, like the snap toggles. The Constraints
  group's toolbar illustration needs reshooting. README.md is already updated (the manual is
  the only doc still stale).

- **Grid colour and line style (2026-09-13).** Manual (*Grid & snapping*, and the Grid
  group's toolbar entry / illustration): the Grid group gained a **style picker**
  (`#grid-style-btn`, under the size combo) and a **colour swatch** (`#grid-color`, under
  the units) — the group's four controls now sit in two flex rows (spacing + units on top,
  style + colour below) rather than the usual column-major grid, so its toolbar shot needs
  reshooting. Behaviour to document: the style picker is
  a combo like the grid size's, but its button and its four rows are **pictures, not
  words** — a sample of the line itself, with the name on the tooltip; *points* shows as a
  few widely spaced dots, which is what tells it from *dotted* at that size. Styles are
  *Solid*, *Dashed*, *Dotted* and *Points* (a dot at each
  intersection); both settings persist in localStorage (`disjointed:gridLook`); the colour
  is kept **per theme** (the swatch edits the theme you are in, switching themes brings that
  theme's grid back); **right-click the swatch** resets that theme to its default tone (a
  double-click can't work — the first click opens the browser's colour dialog); line weight,
  dash lengths and dot size are constant on screen at any zoom, and the Points grid stops
  drawing above ~20 000 visible intersections, where the dots would merge into a wash. Both
  new ids point at the existing `grid` topic in `ID_TOPICS`, so no new topic is needed.
  README.md is already updated.
- **Toolbar regrouped into draggable sections (2026-09-13).** The flat toolbar became a rack
  of named sections (`#tb-sections > .tb-sec`), each a two-row grid filled column by column
  under a caption that is also its drag handle (`src/toolbar.ts`; order in localStorage
  `disjointed:toolbarOrder`, double-click a caption to restore the default). Manual: the
  overview / toolbar illustrations and every "the toolbar's N group" phrasing need redoing —
  the groups are now File · Mode · Animation (sim) · Role · Shapes · Colour · Pattern ·
  Boolean · Mating · Actuators · Transform · Components · Constraints · Grid · Measure ·
  Snapping · View, in a user-settable order, so text must say *which group* by name rather
  than by position. New behaviour to document: the two mode buttons became **one big toggle**
  showing the mode it switches *to* (caption = the mode you are in); the role switch shows
  **icons** instead of Body / Cut / Ref text; grid / constraint-badge / dimension visibility
  each became a small **eye on that group's caption** (struck through = hidden), since the
  toggle applies to the whole group; dragging a group makes the others slide into the order
  they will have when it lands; the armed-tool **hint moved out
  of the toolbar into a status bar** along the bottom of the window (one line, full text on
  its tooltip); the text-size, actuator/motor and solver-tuning fields live in a fixed
  properties strip right of the rack. Placeholder buttons (dimmed, "not implemented yet"
  toast) now hold places for **Subtract**, **Intersect** and **Tangential** — the manual
  should not describe them as working tools. (**Fixed**, **Symmetrical** and **Tangential**
  were placeholders here too and are now real tools — see the entries below; only Subtract
  and Intersect remain dimmed.) `GROUP_TOPICS` in
  `src/helpmap.ts` now keys on the `sec-*` ids; the manual generator reads section membership
  through `.tb-sec[id], .group[id]` (`scripts/manual/shoot.ts`), so `glyphs.js` /
  `DISJOINTED_GROUPS` will change shape on the next `npm run manual`.

- **Line midpoints as snap / constraint targets (2026-09-13).** Manual (*Grid & snapping* —
  the Object snap and *Implicit constraints while dragging* paragraphs; the *Reference
  geometry* "Placement snaps to existing elements" bullet; the Coincident / Horizontal /
  Vertical topics where they list what a point can be): the midpoint of a **body / hole
  edge, a rail or a reference segment** is now a point target everywhere a corner is — object
  snap (the midpoints of rails and reference segments are new targets; body-edge midpoints
  already snapped), the drag-time implicit constraints (hold the middle of a line to arm its
  midpoint — coincident / H / V onto it, badge on the midpoint), and shape / reference-point
  placement (a click near the middle of an edge lands on the midpoint and records a
  coincident). Rule to state: **a point target always wins over the line it sits on** (point
  first, then line, in every pick). A body grabbed nearest an edge midpoint is dragged *by*
  that midpoint and can take alignments itself. New ref kind `midpoint` (nested `of` line
  ref) in `src/model.ts`; the constraint *tools* and the Measure tool still don't pick
  midpoints (deliberate — CAD convention: midpoints are an inference, not a click target).
  README.md is already updated.
- **Dimension direction fixed at placement, shown on the pill (2026-09-13).** Manual (the
  *Dimensions* topic `dimensions`, the *Dimension* element topic `el-dimension` — "Drag the
  label to reposition it" — and the Measure tool topic / `measure` what's-this string in
  `src/main.ts`): a point–point dimension's horizontal / vertical / direct choice is made
  only by where the value is **first** placed; dragging the pill afterwards only moves the
  pill (it used to re-derive the axis from the new position, so a label dragged beside the
  pair silently turned a horizontal size into a vertical one — context dimensions onto the
  ghost behaved the same and are fixed too). The pill now **leads with a small double
  arrow** — horizontal for h, vertical for v, a rising diagonal for a direct distance —
  drawn as vector art (`drawAxisGlyph` in `src/renderer.ts`, same reasoning as the Fixed
  padlock), on the placement preview, context dimensions and placed dimensions alike;
  diameter (⌀), radius (R), point–line, line–line and angle dimensions carry no glyph.
  The glyph is also a **button**: a click cycles the direction h → v → direct (the only
  way to change it after placement; pointer cursor over it; a double-click on the glyph
  does not open the value editor). A driving dimension keeps its target and the geometry
  re-solves along the new axis, or the switch is refused with the usual red flash.
  Context dimensions onto the ghost cycle the same way. Also new: a pill dragged **past
  the end of its dimension line** gets a dashed **leader** from that end (h / v / direct,
  point–line and parallel line–line dimensions; the diameter / radius leaders already
  existed). Label picking is now the whole pill rather than a 16 px circle at its centre.
  Every illustration that shows a dimension label (overview "Bottom row: … a dimension",
  the Measure / Dimensions / context-dimension shots) will need reshooting. README.md is
  already updated.
- **Fixed constraint implemented (2026-09-13).** The dimmed `#fixed-btn` placeholder became a
  real one-shot tool (`data-tool="fixed"`, key `L` — F was already fit-view; plain L had been
  left unbound for the planned shortcut remap, so reconsider it there). A **topic was written**
  (`tool-fixed` in `public/help/index.html`, plus the `L` entry on the Constraints line of the
  shortcut list) because `npm run manual` hard-fails on a missing topic — but it is
  **text-only**: it still needs the usual illustration treatment, and the Constraints group's
  toolbar illustration needs reshooting anyway (the auto-constraints entry above asks for the
  same shot). Also to reword: the *Constraints* overview says "the six constraint tools" and
  lists the badge glyphs `◎ H V ∥ ⊥ =` — now seven, with a drawn **padlock** for
  Fixed (vector art, not a character: `drawLockGlyph` in `src/renderer.ts`). README.md is
  already updated.
- **Editing inside a group (2026-09-13).** Manual (*Multi-select & groups* under Draw, the
  `el-group` element topic, and the overview's "a group behaves as one object" phrasing):
  a **double-click on a member** now opens the group for editing from the inside — its
  bodies and joints select, drag and reshape one at a time, while everything outside drops
  to 20% strength and goes **inert** (not selectable or draggable; still a snap / constraint
  target, so the parts can be aligned and dimensioned against it). Dimensions, constraint
  badges and patterns that **touch** the group stay full strength and stay editable; the
  rest fades with the surroundings; the **grid never fades**. The breadcrumb bar — until now
  a component-only thing, so the *Components* topic's "Click a breadcrumb… to go back out"
  needs rewording — gains an `Assembly ▸ Group (n parts)` crumb. Leaving: **Esc** (twice if
  something is selected), a double-click on empty space, or a crumb. Draw mode only; it is
  a view, not a mode: nothing is saved and nothing lands in undo, and it ends by itself if
  the group is dissolved or undone away. Caveat worth documenting: **Ctrl+G inside a group
  dissolves it**. Needs an illustration (inside vs outside, the veil + crumb), and the
  status-bar line is new text. A second illustration candidate: the same group before /
  after, showing that leaving re-selects it whole. README.md is already updated.
- **Recolouring a whole selection (2026-09-13).** Manual (*Body colour*, and the Colour
  group's toolbar entry / what's-this string): the swatch now recolours **every body of a
  multi-selection or group** at once, not just a single selection; with mixed colours it
  shows the first member's. **Component-instance bodies are skipped** (their colour comes
  from the definition and would revert on the next re-expansion) with a toast saying how
  many. The `#body-color` title attribute in index.html is already reworded; the manual's
  copy of it is not. README.md is already updated.
- **Refusals are spoken, conflicts flash longer (2026-09-13).** Manual (*Sketch constraints
  & driving dimensions*, every constraint tool topic that says an impossible constraint
  "is rejected / flashes red", and the Dimensions topic): a rejected constraint or
  dimension value now also **toasts why**, and the flash lasts **4 s** (was 1.2) so the
  named items can still be found after reading it. Three shapes to document: a pick the
  tool can't use (*"Parallel needs two lines…"*, *"That element is already fixed."*) —
  including a click on bare material or on the wrong shape, which used to be **silently
  ignored**; a conflict, naming the items that flash (*"…conflicts with the Horizontal
  constraint and the 40 mm dimension"*, identical ones collapsed into a count); and a
  dimension value the sketch can't reach. Also covers the polygon side-count refusal, the
  dimension-direction glyph refusal and context dimensions. README.md is already updated.
- **Symmetrical constraint implemented (2026-09-13).** The dimmed `#symmetric-btn`
  placeholder became a real tool (`data-tool="symmetric"`, key `Y` — S is the slider; fold
  into the planned shortcut remap). A **text-only** `tool-symmetric` topic was written (same
  reason as Fixed: `npm run manual` hard-fails on a missing topic) plus the `Y` entry on the
  Constraints line of the shortcut list; it needs the illustration pass, and the Constraints
  group's toolbar shot needs reshooting anyway. Behaviour to document: **three clicks** — two
  points (joint, body corner, polygon centre, reference point) *or* two lines (body edge,
  rail, reference segment), then the **mirror line** (body edge, rail or reference segment);
  the status bar says which click comes next. Two points become true mirror images (equal
  perpendicular distances, opposite sides, on one perpendicular); two lines mirror as whole
  lines — angle and offset — with their endpoints free (lengths may differ, and each keeps
  its length when it moves). Who moves: the usual rank rule, applied mirror-first — a free
  reference line used as the mirror is re-placed onto the pair's bisector by its first
  symmetry; a line carrying two symmetries, snapped to geometry (tied), or locked with Fixed
  is the reference and the pair comes to it; a body edge among geometry shares the move.
  Dragging one of a symmetric pair swings the partner. The badge is a drawn **dashed mirror
  with a dot each side** (`drawMirrorGlyph`, the toolbar icon) on both elements *and* on
  the mirror line; hovering a badge links the pair. The *Constraints* overview now counts
  **eight** tools. README.md is already updated.
- **Tangential constraint implemented (2026-09-13).** The dimmed `#tangent-btn` placeholder
  became a real tool (`data-tool="tangent"`, key `Z` — the last free plain letter, T being
  perpendicular; fold into the planned shortcut remap). A **text-only** `tool-tangent` topic
  was written (same reason as Fixed / Symmetrical: `npm run manual` hard-fails on a missing
  topic) plus the `Z` entry on the Constraints line of the shortcut list; it needs the
  illustration pass, and the Constraints group's toolbar shot needs reshooting anyway.
  Behaviour to document: **two clicks, either order** — a **circle or arc** (a disk body's
  rim, a round hole, a reference circle, a reference arc) and a **line** (body edge, rail,
  reference segment); the status bar says which one is still missing, and a wrong pick is
  explained ("Tangential already has its circle — click the line here…"). The line becomes
  tangent to the circle: the centre sits one radius off the line, on the side it already is.
  The radius never changes (a disk is sized by its diameter dimension / rim handle, an arc
  moves as a rigid piece); who moves is the usual rank rule — a free reference circle comes to
  a body edge, a disk comes to a Fixed edge, a Fixed disk pushes the edge, two free bodies meet
  halfway; dragging the disk slides it along the line; driving a tangent disk's diameter
  re-solves so the line follows the new rim; a circle dragged through its line re-attaches on
  the far side. The badge is a drawn **circle touching a line** (`drawTangentGlyph`, the
  toolbar icon) — **one** badge, at the contact point, offset away from the circle; hovering
  it highlights the rim / arc and the line (no link line). While the tool is armed, a circle
  under the cursor highlights as its whole rim (an arc as its arc), like the Measure tool's
  diameter pick. Also worth a sentence (and an illustration: two lines + arc, before /
  after): **blending a line into an arc** — tangent, then a Coincident between the arc's
  end and the line's end — makes the line touch the arc exactly at the shared end, and
  dragging that end swings the line. The *Constraints* overview now counts **nine** tools
  and its badge list gains the tangent glyph. README.md is already updated.

- **Coincident toolbar icon gained a crosshair (2026-09-13).** The Coincident button's glyph
  (ring + centre dot) was almost indistinguishable from the **Joint** tool's at 22 px, so it
  became a reticle: the ring plus four short ticks outside it, north / south / east / west.
  Nothing about the tool changed, and the on-canvas coincident **badge** (`◎`) is untouched —
  but the manual's generated glyph table (`glyphs.js`) and the Constraints group's toolbar
  illustration both render the toolbar icon, so both need reshooting with `npm run manual`.
- **Dimensions on a patterned seed (2026-09-22).** Manual (Patterns topic, Measure topic): a
  dimension with both ends on the seed — its diameter, the width between its own corners, its
  distance to a body edge — drives it like any hole and every instance copies the result (in a
  linear pattern the same works from one instance's own corners). Only a distance between two
  instances (seed ↔ copy, copy ↔ copy) is refused, with a toast pointing at the pattern's
  spacing label. Also (Driving dimensions topic): a dimension with an end on *any* hole no
  longer takes the first-dimension uniform-scale shortcut — the hole reshapes or moves, the
  outline stays.
- **Multi-seed patterns (2026-09-22, format v22).** Manual (Linear / Circular pattern tool
  topics, Patterns topic): with a pattern tool armed, Ctrl+click picks several holes / joints of
  one body (Ctrl+click a picked one drops it; the layout preview shows once Ctrl is released,
  and Ctrl+click still adds / removes seeds after the row exists); the group repeats as one, so
  every instance reproduces the seeds' relative placement; a dimension between two seeds drives
  that placement for every copy; the first pick is the anchor (dotted line / centre offset);
  deleting one seed drops its copies from the array (they stay plain) and the pattern lives on.
  The what's-this strings for the two pattern tools (`patternLinear` / `patternCircular` in
  `src/main.ts`) do not mention Ctrl yet — reword in the same pass.
