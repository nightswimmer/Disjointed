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
