/**
 * Verifies the impossible-assembly handling: grounded joints stay *exactly* fixed even
 * when other constraints can't be satisfied, and solve() reports the unsatisfiable
 * constraints (the points that can't meet) as "breaks" — while a reachable layout
 * resolves cleanly with no breaks.
 */
import { Scene } from "../src/model";
import { solve } from "../src/solver";
import { dist } from "../src/geometry";

let failures = 0;
function check(label: string, ok: boolean, detail = "") {
  console.log(`${ok ? "PASS" : "FAIL"}  ${label}${detail ? `  (${detail})` : ""}`);
  if (!ok) failures++;
}

// A body grounded at one joint, pinned at another to a far anchor it can never reach.
// jHandle can only travel on a 40px circle around the grounded jGround; the anchor sits
// 500px away, so the pin is impossible.
function makeScene(anchorAt: { x: number; y: number }) {
  const scene = new Scene();
  const body = scene.addBody([
    { x: 0, y: -10 }, { x: 40, y: -10 }, { x: 40, y: 10 }, { x: 0, y: 10 },
  ]);
  const jGround = scene.addJoint(body.id, { x: 0, y: 0 });
  const jHandle = scene.addJoint(body.id, { x: 40, y: 0 }); // 40px from jGround
  scene.addGround(jGround.id, { x: 0, y: 0 });
  const anchor = scene.addFreeJoint(anchorAt);
  scene.addGround(anchor.id, anchorAt);
  scene.addPin(jHandle.id, anchor.id);
  return { scene, jGround, jHandle, anchor };
}

// --- 1) Impossible: a far anchor the handle can't reach. ---
{
  const { scene, jGround, jHandle, anchor } = makeScene({ x: 0, y: 500 });
  const breaks = solve(scene, null, 120);

  check("grounded body joint stays exactly on its anchor",
    dist(scene.jointWorld(jGround), { x: 0, y: 0 }) < 1e-6,
    `drift ${dist(scene.jointWorld(jGround), { x: 0, y: 0 }).toExponential(2)} px`);
  check("grounded free anchor never moves",
    dist(scene.jointWorld(anchor), { x: 0, y: 500 }) < 1e-9);
  check("handle stays on its 40px circle around the ground",
    Math.abs(dist(scene.jointWorld(jHandle), { x: 0, y: 0 }) - 40) < 0.5,
    `radius ${dist(scene.jointWorld(jHandle), { x: 0, y: 0 }).toFixed(2)} px`);

  check("solve reports a break", breaks.length === 1, `${breaks.length} break(s)`);
  if (breaks.length === 1) {
    const b = breaks[0];
    // The break connects the handle to the anchor (the unsatisfiable pin), in either order.
    const h = scene.jointWorld(jHandle);
    const linksHandleAnchor =
      (dist(b.a, h) < 1e-6 && dist(b.b, { x: 0, y: 500 }) < 1e-6) ||
      (dist(b.b, h) < 1e-6 && dist(b.a, { x: 0, y: 500 }) < 1e-6);
    check("the break connects the handle to the unreachable anchor", linksHandleAnchor);
    // The handle can reach at most 40px from the ground; the anchor is 500px away, so the
    // gap is large and positive (nearest possible is 460; an unstable rest leaves more).
    check("break error reflects the unreachable gap", b.error > 400, `${b.error.toFixed(2)} px`);
    // The break names both pin endpoints so the UI can paint them red.
    const flagsBoth = b.joints.includes(jHandle.id) && b.joints.includes(anchor.id);
    check("break names both stuck joints", flagsBoth, `joints ${b.joints.join(",")}`);
  }
}

// --- 2) Reachable: the same rig with the anchor within the handle's circle. ---
{
  const { scene, jGround } = makeScene({ x: 40, y: 0 }); // exactly on the handle's start
  const breaks = solve(scene, null, 120);
  check("reachable assembly has no breaks", breaks.length === 0, `${breaks.length} break(s)`);
  check("ground still exactly fixed when solvable",
    dist(scene.jointWorld(jGround), { x: 0, y: 0 }) < 1e-6);
}

// --- 3) A solvable four-bar with an impossible pendant hanging off its coupler. The
//        impossible pendant must NOT break the four-bar: only the pendant's pin is flagged,
//        the four-bar pins stay satisfied, and the pendant's ground stays exactly fixed. ---
{
  const s = new Scene();
  const crank = s.addBody([{ x: -5, y: -5 }, { x: 45, y: -5 }, { x: 45, y: 5 }, { x: -5, y: 5 }]);
  const c0 = s.addJoint(crank.id, { x: 0, y: 0 }); s.addGround(c0.id, { x: 0, y: 0 });
  const c1 = s.addJoint(crank.id, { x: 40, y: 0 });
  const cpl = s.addBody([{ x: 38, y: -3 }, { x: 142, y: -3 }, { x: 142, y: 3 }, { x: 38, y: 3 }]);
  const p0 = s.addJoint(cpl.id, { x: 40, y: 0 }); s.addPin(c1.id, p0.id);
  const p1 = s.addJoint(cpl.id, { x: 140, y: 0 });
  const rocker = s.addBody([{ x: 135, y: -5 }, { x: 165, y: -5 }, { x: 165, y: 85 }, { x: 135, y: 85 }]);
  const r0 = s.addJoint(rocker.id, { x: 140, y: 0 }); s.addPin(p1.id, r0.id);
  const r1 = s.addJoint(rocker.id, { x: 150, y: 80 }); s.addGround(r1.id, { x: 150, y: 80 });
  // Pendant pinned to the coupler, with a ground far out of reach -> impossible.
  const pend = s.addBody([{ x: 80, y: 10 }, { x: 110, y: 10 }, { x: 110, y: 40 }, { x: 80, y: 40 }]);
  const pj = s.addJoint(pend.id, { x: 90, y: 20 });
  const cm = s.addJoint(cpl.id, { x: 90, y: 0 }); s.addPin(pj.id, cm.id);
  const pg = s.addJoint(pend.id, { x: 100, y: 30 }); s.addGround(pg.id, { x: 100, y: 900 });

  const breaks = solve(s, null, 120);
  check("only one break (the impossible pendant pin)", breaks.length === 1, `${breaks.length} break(s)`);
  const fourbar = Math.max(
    dist(s.jointWorld(c1), s.jointWorld(p0)),
    dist(s.jointWorld(p1), s.jointWorld(r0))
  );
  check("the four-bar stays intact (its pins satisfied)", fourbar < 0.5, `worst pin ${fourbar.toFixed(3)} px`);
  check("the pendant's ground stays exactly fixed", dist(s.jointWorld(pg), { x: 100, y: 900 }) < 1e-6,
    `drift ${dist(s.jointWorld(pg), { x: 100, y: 900 }).toExponential(2)} px`);
  check("the crank ground stays exactly fixed", dist(s.jointWorld(c0), { x: 0, y: 0 }) < 1e-6);
}

// --- 4) Two grounded bodies whose free joints pin together (solvable by rotating both),
//        plus a third body pinned to those two *grounded* joints at an impossible span. The
//        third piece connects only to grounded points, so it must NOT disturb the first two:
//        the good pin solves, the grounds stay fixed, and only the third piece is flagged. ---
{
  const s = new Scene();
  // Left piece: grounded at gL=(0,0), free joint fL up at (0,200).
  const bl = s.addBody([{ x: -10, y: -10 }, { x: 10, y: -10 }, { x: 10, y: 210 }, { x: -10, y: 210 }]);
  const gL = s.addJoint(bl.id, { x: 0, y: 0 }); s.addGround(gL.id, { x: 0, y: 0 });
  const fL = s.addJoint(bl.id, { x: 0, y: 200 });
  // Right piece: grounded at gR=(300,0), free joint fR up at (300,200).
  const br = s.addBody([{ x: 290, y: -10 }, { x: 310, y: -10 }, { x: 310, y: 210 }, { x: 290, y: 210 }]);
  const gR = s.addJoint(br.id, { x: 300, y: 0 }); s.addGround(gR.id, { x: 300, y: 0 });
  const fR = s.addJoint(br.id, { x: 300, y: 200 });
  // The two free ends pin together — reachable by rotating both pieces inward.
  s.addPin(fL.id, fR.id);
  // Third piece: spans only 100 but is pinned to the two grounds 300 apart -> impossible.
  const bx = s.addBody([{ x: 0, y: 290 }, { x: 100, y: 290 }, { x: 100, y: 310 }, { x: 0, y: 310 }]);
  const xL = s.addJoint(bx.id, { x: 0, y: 300 });
  const xR = s.addJoint(bx.id, { x: 100, y: 300 });
  s.addPin(xL.id, gL.id);
  s.addPin(xR.id, gR.id);

  const breaks = solve(s, null, 120);
  check("only the impossible third piece is flagged", breaks.length === 1, `${breaks.length} break(s)`);
  check("the good pin (two free ends) is satisfied",
    dist(s.jointWorld(fL), s.jointWorld(fR)) < 0.5, `gap ${dist(s.jointWorld(fL), s.jointWorld(fR)).toFixed(3)} px`);
  check("left ground stays exactly fixed", dist(s.jointWorld(gL), { x: 0, y: 0 }) < 1e-6);
  check("right ground stays exactly fixed", dist(s.jointWorld(gR), { x: 300, y: 0 }) < 1e-6);
}

console.log(failures === 0 ? "\nALL PASS" : `\n${failures} FAILURE(S)`);
process.exit(failures === 0 ? 0 : 1);
