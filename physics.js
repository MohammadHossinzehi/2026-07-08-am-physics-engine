// physics.js
// A small 2D rigid body physics engine built from scratch.
//
// Scope (documented design decision — see README "Design decisions"):
// this engine models LINEAR rigid body dynamics only (position, velocity,
// mass, restitution, friction) for two shape types: circles and
// axis aligned boxes. There is no rotational dynamics / torque. That
// tradeoff keeps every collision routine exactly solvable and testable
// instead of shipping a half finished rotational system.
//
// Works in a browser (via <script type="module">) or in Node (for tests).

export class Vec2 {
  constructor(x = 0, y = 0) {
    this.x = x;
    this.y = y;
  }

  static zero() {
    return new Vec2(0, 0);
  }

  add(v) {
    return new Vec2(this.x + v.x, this.y + v.y);
  }

  sub(v) {
    return new Vec2(this.x - v.x, this.y - v.y);
  }

  mul(s) {
    return new Vec2(this.x * s, this.y * s);
  }

  dot(v) {
    return this.x * v.x + this.y * v.y;
  }

  lengthSq() {
    return this.x * this.x + this.y * this.y;
  }

  length() {
    return Math.sqrt(this.lengthSq());
  }

  normalize() {
    const len = this.length();
    if (len === 0) return new Vec2(0, 0);
    return new Vec2(this.x / len, this.y / len);
  }

  clone() {
    return new Vec2(this.x, this.y);
  }
}

let nextBodyId = 1;

export class Body {
  constructor({
    shape,
    position = new Vec2(0, 0),
    velocity = new Vec2(0, 0),
    mass = 1,
    restitution = 0.5,
    friction = 0.3,
    isStatic = false,
  }) {
    if (shape.type !== "circle" && shape.type !== "box") {
      throw new Error(`Unknown shape type: ${shape.type}`);
    }
    this.id = nextBodyId++;
    this.shape = shape; // { type: 'circle', radius } | { type: 'box', width, height }
    this.position = position.clone();
    this.velocity = velocity.clone();
    this.restitution = clamp(restitution, 0, 1);
    this.friction = Math.max(0, friction);
    this.isStatic = isStatic;
    this.mass = isStatic ? Infinity : Math.max(mass, 1e-6);
    this.invMass = isStatic ? 0 : 1 / this.mass;
  }

  // Axis aligned bounding box, used for broad phase and for box shapes
  // directly (boxes in this engine are always axis aligned).
  aabb() {
    if (this.shape.type === "circle") {
      const r = this.shape.radius;
      return {
        minX: this.position.x - r,
        maxX: this.position.x + r,
        minY: this.position.y - r,
        maxY: this.position.y + r,
      };
    }
    const hw = this.shape.width / 2;
    const hh = this.shape.height / 2;
    return {
      minX: this.position.x - hw,
      maxX: this.position.x + hw,
      minY: this.position.y - hh,
      maxY: this.position.y + hh,
    };
  }
}

function clamp(v, lo, hi) {
  return Math.max(lo, Math.min(hi, v));
}

function aabbOverlap(a, b) {
  return a.minX <= b.maxX && a.maxX >= b.minX && a.minY <= b.maxY && a.maxY >= b.minY;
}

// --- Narrow phase collision detection ------------------------------------
// Every collide* function returns either null (no collision) or a manifold:
//   { normal: Vec2 (points from A to B), penetration: number > 0 }

function collideCircleCircle(a, b) {
  const delta = b.position.sub(a.position);
  const distSq = delta.lengthSq();
  const radiusSum = a.shape.radius + b.shape.radius;
  if (distSq >= radiusSum * radiusSum) return null;

  const dist = Math.sqrt(distSq);
  const normal = dist === 0 ? new Vec2(1, 0) : delta.mul(1 / dist);
  const penetration = radiusSum - dist;
  return { normal, penetration };
}

function collideCircleBox(circle, box) {
  const half = new Vec2(box.shape.width / 2, box.shape.height / 2);
  const rel = circle.position.sub(box.position);

  // Closest point on the box to the circle's center, clamped to the box.
  const closest = new Vec2(clamp(rel.x, -half.x, half.x), clamp(rel.y, -half.y, half.y));

  const inside = rel.x === closest.x && rel.y === closest.y;
  let normal, penetration;

  if (!inside) {
    const diff = rel.sub(closest);
    const distSq = diff.lengthSq();
    if (distSq >= circle.shape.radius * circle.shape.radius) return null;
    const dist = Math.sqrt(distSq);
    normal = dist === 0 ? new Vec2(0, -1) : diff.mul(1 / dist);
    penetration = circle.shape.radius - dist;
  } else {
    // Circle center is inside the box: push out along the shallowest axis.
    const dx = half.x - Math.abs(rel.x);
    const dy = half.y - Math.abs(rel.y);
    if (dx < dy) {
      normal = new Vec2(rel.x < 0 ? -1 : 1, 0);
      penetration = dx + circle.shape.radius;
    } else {
      normal = new Vec2(0, rel.y < 0 ? -1 : 1);
      penetration = dy + circle.shape.radius;
    }
  }

  // Manifold normal convention is "from A to B" = from box to circle here,
  // callers pass (circle, box) so we flip to keep (a -> b) consistent.
  return { normal: normal.mul(-1), penetration };
}

function collideBoxBox(a, b) {
  const ah = new Vec2(a.shape.width / 2, a.shape.height / 2);
  const bh = new Vec2(b.shape.width / 2, b.shape.height / 2);
  const delta = b.position.sub(a.position);

  const overlapX = ah.x + bh.x - Math.abs(delta.x);
  if (overlapX <= 0) return null;
  const overlapY = ah.y + bh.y - Math.abs(delta.y);
  if (overlapY <= 0) return null;

  // Minimum translation vector: push apart along the axis of least overlap.
  if (overlapX < overlapY) {
    return {
      normal: new Vec2(delta.x < 0 ? -1 : 1, 0),
      penetration: overlapX,
    };
  }
  return {
    normal: new Vec2(0, delta.y < 0 ? -1 : 1),
    penetration: overlapY,
  };
}

export function collide(a, b) {
  if (a.shape.type === "circle" && b.shape.type === "circle") return collideCircleCircle(a, b);
  if (a.shape.type === "box" && b.shape.type === "box") return collideBoxBox(a, b);
  if (a.shape.type === "circle" && b.shape.type === "box") return collideCircleBox(a, b);
  // box vs circle: flip a/b, then flip the resulting normal back
  const manifold = collideCircleBox(b, a);
  if (!manifold) return null;
  return { normal: manifold.normal.mul(-1), penetration: manifold.penetration };
}

// --- Collision resolution --------------------------------------------------
// Sequential impulse resolution (velocity) + positional correction
// ("Baumgarte style" slop-corrected push out) to stop resting bodies from
// slowly sinking into each other, a well known issue with pure velocity
// based resolution under constant gravity.

const POSITION_CORRECTION_PERCENT = 0.8;
const POSITION_SLOP = 0.01;

function resolveCollision(a, b, manifold) {
  const { normal, penetration } = manifold;
  const invMassSum = a.invMass + b.invMass;
  if (invMassSum === 0) return; // both static, nothing to do

  const relativeVelocity = b.velocity.sub(a.velocity);
  const velAlongNormal = relativeVelocity.dot(normal);

  // Only resolve if bodies are moving toward each other.
  if (velAlongNormal <= 0) {
    const restitution = Math.min(a.restitution, b.restitution);
    const j = (-(1 + restitution) * velAlongNormal) / invMassSum;
    const impulse = normal.mul(j);
    a.velocity = a.velocity.sub(impulse.mul(a.invMass));
    b.velocity = b.velocity.add(impulse.mul(b.invMass));

    // Coulomb friction along the tangent of the contact normal.
    const relVelAfter = b.velocity.sub(a.velocity);
    const tangent = relVelAfter.sub(normal.mul(relVelAfter.dot(normal)));
    const tangentLen = tangent.length();
    if (tangentLen > 1e-9) {
      const t = tangent.mul(1 / tangentLen);
      const jt = -relVelAfter.dot(t) / invMassSum;
      const mu = Math.sqrt(a.friction * b.friction);
      const maxFriction = Math.abs(j) * mu;
      const frictionImpulse = t.mul(clamp(jt, -maxFriction, maxFriction));
      a.velocity = a.velocity.sub(frictionImpulse.mul(a.invMass));
      b.velocity = b.velocity.add(frictionImpulse.mul(b.invMass));
    }
  }

  // Positional correction (independent of whether velocity was resolved
  // this step, since resting contacts have velAlongNormal ~ 0).
  const correctionMag = (Math.max(penetration - POSITION_SLOP, 0) / invMassSum) * POSITION_CORRECTION_PERCENT;
  const correction = normal.mul(correctionMag);
  a.position = a.position.sub(correction.mul(a.invMass));
  b.position = b.position.add(correction.mul(b.invMass));
}

export class World {
  constructor({ gravity = new Vec2(0, 500) } = {}) {
    this.gravity = gravity;
    this.bodies = [];
  }

  addBody(body) {
    this.bodies.push(body);
    return body;
  }

  removeBody(body) {
    this.bodies = this.bodies.filter((b) => b !== body);
  }

  // Advance the simulation by dt seconds. Uses semi-implicit (symplectic)
  // Euler integration: update velocity from forces first, then update
  // position from the new velocity. This is numerically more stable for
  // springs/gravity than explicit Euler while staying simple to implement.
  step(dt) {
    for (const body of this.bodies) {
      if (body.isStatic) continue;
      body.velocity = body.velocity.add(this.gravity.mul(dt));
    }

    // Broad phase: cheap AABB overlap test to build a candidate pair list
    // before running the more expensive narrow phase checks.
    const pairs = [];
    for (let i = 0; i < this.bodies.length; i++) {
      for (let j = i + 1; j < this.bodies.length; j++) {
        const a = this.bodies[i];
        const b = this.bodies[j];
        if (a.isStatic && b.isStatic) continue;
        if (aabbOverlap(a.aabb(), b.aabb())) pairs.push([a, b]);
      }
    }

    for (const [a, b] of pairs) {
      const manifold = collide(a, b);
      if (manifold) resolveCollision(a, b, manifold);
    }

    for (const body of this.bodies) {
      if (body.isStatic) continue;
      body.position = body.position.add(body.velocity.mul(dt));
    }
  }
}
