# 2D Physics Engine (from scratch)

A small, dependency-free 2D rigid body physics engine written in plain
JavaScript, plus an interactive canvas demo you can drop shapes into and
watch gravity, restitution, and friction play out in real time.

## What it does

`physics.js` implements the core pieces of a real time physics engine:

- **Vec2** — a minimal 2D vector type (add, sub, scale, dot, length, normalize).
- **Body** — a rigid body with position, velocity, mass, restitution,
  friction, and one of two shapes: `circle` or axis aligned `box`. Static
  bodies (infinite mass, `invMass = 0`) act as immovable floors, walls, and
  platforms.
- **Broad phase** — a cheap AABB overlap test over every body pair each
  step, so the expensive narrow phase checks only run on pairs that could
  plausibly be touching.
- **Narrow phase** — exact collision routines for circle-circle,
  circle-box, and box-box, each returning a manifold (`normal` + `penetration
  depth`) when two shapes overlap.
- **Resolution** — sequential impulse resolution along the contact normal
  (using each body's restitution), Coulomb friction along the contact
  tangent (clamped by the normal impulse), and Baumgarte-style positional
  correction so resting bodies don't slowly sink into the floor, a classic
  failure mode of velocity-only resolution under constant gravity.
- **Integration** — semi-implicit (symplectic) Euler: velocity is updated
  from gravity first, then position is updated from the new velocity. It's
  a few lines of code but noticeably more stable than explicit Euler for
  anything under constant acceleration.

Why build this instead of using Box2D/Matter.js? Because the point of the
project is to actually understand *why* physics engines behave the way
they do — why naive resolution causes jitter, why friction has to be
clamped by the normal impulse (Coulomb's law), why integration order
matters. The engine is small enough to read end to end in one sitting.

## How to run it

**Tests** (Node.js 18+, no install needed — zero dependencies):

```bash
node tests.js
# or: npm test
```

**Interactive demo** (needs to be served over HTTP because it uses native
ES modules, which browsers block from `file://`):

```bash
# any static file server works, for example:
npx serve .
# or
python3 -m http.server 8000
```

Then open `demo.html` (e.g. `http://localhost:8000/demo.html`). Click
anywhere on the canvas to drop a circle, hold Shift and click to drop a
box. The sliders control gravity strength and the restitution/friction
that new shapes spawn with; "Drop 20 shapes" rains a batch of random
circles and boxes from the top of the canvas.

## Design decisions

- **Boxes are axis aligned, bodies don't rotate.** This is a deliberate
  scope cut, not an oversight. A fully general rotational rigid body
  engine needs oriented bounding boxes, SAT with polygon clipping for
  multi point contact manifolds, and torque/angular momentum — each of
  which roughly doubles the surface area for bugs. Restricting shapes to
  circles and axis aligned boxes keeps every collision routine exactly
  solvable in closed form (see `collideCircleCircle`, `collideCircleBox`,
  `collideBoxBox` in `physics.js`), which made it realistic to write a
  test suite that checks *exact* penetration depths and normals rather
  than "close enough" behavior.
- **Single contact point per pair, resolved sequentially.** Production
  engines batch multiple contact points per manifold and iterate the
  solver several times per step for stiffer stacks. This engine resolves
  one impulse per colliding pair per step, which is simpler and works
  well for the moderate stacking demonstrated in the demo, at the cost of
  being less stable under very tall, heavy stacks.
- **Friction coefficient is `sqrt(muA * muB)`.** A common, simple way to
  combine two surfaces' friction values without needing a full friction
  lookup table.

## Testing

`tests.js` is a plain-Node assertion suite (no test framework, just
`node:assert/strict`) with 15 checks across three levels:

1. **Unit** — `Vec2` arithmetic, `Body` mass/AABB computation.
2. **Collision detection** — exact penetration depth and normal direction
   for overlapping/non-overlapping circle-circle, circle-box, and
   box-box pairs, including a check that `collide(a, b)` and
   `collide(b, a)` are consistent mirror images of each other.
3. **Simulation** — full `World.step()` behavior: a free falling body's
   trajectory is checked against the closed form `y = 1/2 g t^2` (within
   the expected numerical drift of semi-implicit Euler), a dropped ball
   settles to rest just above the floor instead of sinking through it, an
   elastic head-on collision between equal masses exchanges velocities,
   and a sliding box's speed measurably decreases under friction.

Run `node tests.js` — it prints each check and exits non-zero if
anything fails, so it's CI-friendly without any extra tooling.

## Files

- `physics.js` — the engine (`Vec2`, `Body`, `World`, `collide`).
- `demo.html` / `demo.js` — interactive canvas demo built on the engine.
- `tests.js` — the test suite described above.
- `package.json` — just declares `type: module` and a `test` script.
