// tests.js
// Plain Node assertion test suite, no external dependencies.
// Run with: node tests.js

import assert from "node:assert/strict";
import { Vec2, Body, World, collide } from "./physics.js";

let passed = 0;
let failed = 0;

function test(name, fn) {
  try {
    fn();
    passed++;
    console.log(`  ok  - ${name}`);
  } catch (err) {
    failed++;
    console.log(`FAIL  - ${name}`);
    console.log(`        ${err.message}`);
  }
}

function approx(a, b, eps = 1e-6) {
  return Math.abs(a - b) <= eps;
}

console.log("Vec2");
test("add/sub/mul are componentwise", () => {
  const a = new Vec2(1, 2);
  const b = new Vec2(3, 4);
  assert.deepEqual(a.add(b), new Vec2(4, 6));
  assert.deepEqual(b.sub(a), new Vec2(2, 2));
  assert.deepEqual(a.mul(2), new Vec2(2, 4));
});

test("normalize produces a unit vector", () => {
  const v = new Vec2(3, 4).normalize();
  assert.ok(approx(v.length(), 1));
  assert.ok(approx(v.x, 0.6));
  assert.ok(approx(v.y, 0.8));
});

test("normalize of the zero vector does not throw or divide by zero", () => {
  const v = new Vec2(0, 0).normalize();
  assert.deepEqual(v, new Vec2(0, 0));
});

console.log("\nBody");
test("static bodies have zero inverse mass", () => {
  const body = new Body({ shape: { type: "box", width: 10, height: 10 }, isStatic: true });
  assert.equal(body.invMass, 0);
});

test("dynamic bodies compute invMass = 1/mass", () => {
  const body = new Body({ shape: { type: "circle", radius: 5 }, mass: 4 });
  assert.ok(approx(body.invMass, 0.25));
});

test("aabb for a circle is centered on its position", () => {
  const body = new Body({ shape: { type: "circle", radius: 2 }, position: new Vec2(10, 10) });
  const box = body.aabb();
  assert.ok(approx(box.minX, 8) && approx(box.maxX, 12));
  assert.ok(approx(box.minY, 8) && approx(box.maxY, 12));
});

console.log("\nCollision detection");
test("overlapping circles report a collision with correct penetration", () => {
  const a = new Body({ shape: { type: "circle", radius: 5 }, position: new Vec2(0, 0) });
  const b = new Body({ shape: { type: "circle", radius: 5 }, position: new Vec2(8, 0) });
  const manifold = collide(a, b);
  assert.ok(manifold);
  assert.ok(approx(manifold.penetration, 2));
  assert.deepEqual(manifold.normal, new Vec2(1, 0));
});

test("non overlapping circles report no collision", () => {
  const a = new Body({ shape: { type: "circle", radius: 5 }, position: new Vec2(0, 0) });
  const b = new Body({ shape: { type: "circle", radius: 5 }, position: new Vec2(20, 0) });
  assert.equal(collide(a, b), null);
});

test("circle resting on top of a box collides along +y/-y", () => {
  const box = new Body({ shape: { type: "box", width: 20, height: 4 }, position: new Vec2(0, 10), isStatic: true });
  const circle = new Body({ shape: { type: "circle", radius: 3 }, position: new Vec2(0, 6) });
  const manifold = collide(circle, box);
  assert.ok(manifold);
  assert.ok(manifold.penetration > 0);
});

test("box vs box uses the minimum translation vector axis", () => {
  const a = new Body({ shape: { type: "box", width: 10, height: 10 }, position: new Vec2(0, 0) });
  const b = new Body({ shape: { type: "box", width: 10, height: 10 }, position: new Vec2(9, 0) });
  const manifold = collide(a, b);
  assert.ok(manifold);
  assert.ok(approx(manifold.normal.y, 0));
  assert.ok(approx(manifold.penetration, 1));
});

test("box vs circle is the mirror image of circle vs box", () => {
  const box = new Body({ shape: { type: "box", width: 10, height: 10 }, position: new Vec2(0, 0) });
  const circle = new Body({ shape: { type: "circle", radius: 3 }, position: new Vec2(7, 0) });
  const m1 = collide(circle, box);
  const m2 = collide(box, circle);
  assert.ok(m1 && m2);
  assert.ok(approx(m1.penetration, m2.penetration));
  assert.ok(approx(m1.normal.x, -m2.normal.x));
});

console.log("\nSimulation");
test("a free falling body follows y = 1/2 g t^2 before any collision", () => {
  const g = 500;
  const world = new World({ gravity: new Vec2(0, g) });
  const body = world.addBody(new Body({ shape: { type: "circle", radius: 1 }, position: new Vec2(0, 0) }));

  const dt = 1 / 240; // small timestep for close agreement with the analytic solution
  const totalTime = 0.5;
  for (let t = 0; t < totalTime; t += dt) world.step(dt);

  const expectedY = 0.5 * g * totalTime * totalTime;
  const relError = Math.abs(body.position.y - expectedY) / expectedY;
  // Semi-implicit Euler is only a first order approximation of the analytic
  // solution, so a few percent of drift over 0.5s at 240Hz is expected.
  assert.ok(relError < 0.03, `expected ~${expectedY}, got ${body.position.y}`);
});

test("a ball dropped on the ground comes to rest above it, not through it", () => {
  const world = new World({ gravity: new Vec2(0, 500) });
  const ground = world.addBody(
    new Body({ shape: { type: "box", width: 400, height: 20 }, position: new Vec2(0, 110), isStatic: true, friction: 0.5 })
  );
  const ball = world.addBody(
    new Body({ shape: { type: "circle", radius: 10 }, position: new Vec2(0, 0), restitution: 0.1, friction: 0.5 })
  );

  for (let i = 0; i < 600; i++) world.step(1 / 60);

  const groundTop = ground.position.y - ground.shape.height / 2;
  assert.ok(ball.position.y < groundTop, "ball should rest above the ground surface");
  assert.ok(ball.position.y > groundTop - ball.shape.radius - 2, "ball should not float far above the ground");
});

test("an elastic head on collision between equal masses exchanges velocities", () => {
  const world = new World({ gravity: new Vec2(0, 0) });
  const a = world.addBody(
    new Body({ shape: { type: "circle", radius: 5 }, position: new Vec2(-6, 0), velocity: new Vec2(100, 0), restitution: 1, friction: 0 })
  );
  const b = world.addBody(
    new Body({ shape: { type: "circle", radius: 5 }, position: new Vec2(6, 0), velocity: new Vec2(0, 0), restitution: 1, friction: 0 })
  );

  // Step until the circles actually touch (they start slightly apart).
  for (let i = 0; i < 20; i++) world.step(1 / 240);

  assert.ok(a.velocity.x < 10, `expected a to have slowed down, got ${a.velocity.x}`);
  assert.ok(b.velocity.x > 90, `expected b to have sped up, got ${b.velocity.x}`);
});

test("friction decelerates a sliding body on a static surface over time", () => {
  const world = new World({ gravity: new Vec2(0, 500) });
  world.addBody(
    new Body({ shape: { type: "box", width: 1000, height: 20 }, position: new Vec2(0, 20), isStatic: true, friction: 0.2 })
  );
  const box = world.addBody(
    new Body({ shape: { type: "box", width: 10, height: 10 }, position: new Vec2(0, 9.9), velocity: new Vec2(200, 0), friction: 0.2, restitution: 0 })
  );

  for (let i = 0; i < 10; i++) world.step(1 / 60); // let it settle onto the surface
  const speedAfterLanding = Math.abs(box.velocity.x);
  assert.ok(speedAfterLanding > 0, "sanity check: body should still be moving right after landing");

  for (let i = 0; i < 10; i++) world.step(1 / 60); // slide with friction
  const speedLater = Math.abs(box.velocity.x);

  assert.ok(speedLater < speedAfterLanding, "friction should reduce horizontal speed over time");
});

console.log(`\n${passed} passed, ${failed} failed`);
if (failed > 0) process.exit(1);
