# Handoff — shape tools & roles

Design notes and open items for the **shape tools + roles** feature (phase 1 shipped
2026-09-13, format v20). Read together with PROJECT_INSTRUCTIONS.md (current state) — this
file holds what was **discussed and decided but not built yet**, so a future session can
pick the next phase up without re-doing the brainstorm.

## Decisions already made (don't re-open)

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
- **Reference geometry as constraint targets**: point-on-circle / tangent constraints,
  concentric; an arc's centre as a reference point (it's derived today, so not a solver var).
- **Shortcuts** for the new tools are Shift+letter picks (Shift+B rectangle, Shift+C circle,
  Shift+P polygon, Shift+S slot, Shift+L line, Shift+A arc, Shift+T text) — easy to change
  in `SHIFT_TOOL_KEYS` (src/main.ts) and the manual's shortcut list.
- Manual: the new tool topics are text-only; gesture illustrations (before / mid / after)
  for the shape tools would fit the existing `scripts/manual/shots.ts` pipeline (the
  role-styled preview is exposed through `RenderInput.shapeDraft`).

## In-app manual — pending updates (UI-tweak batch, started 2026-09-13)

The in-app help (manual generator in `scripts/manual/`, what's-this drawer, shortcut list) is
**deliberately not updated** during the current run of UI-tweak commits. Every UI change that
touches something the manual describes gets a line here instead; a later session will apply
them all in one go. Add to this list as you go — one bullet per change, say which manual topic
/ illustration / shortcut entry is affected and what the new behaviour is.

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
- Plain **`L` is unbound** for now (whole shortcut map to be redone later).
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
