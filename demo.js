// demo.js
// Interactive canvas demo driving the physics.js engine.

import { Vec2, Body, World } from "./physics.js";

const canvas = document.getElementById("scene");
const ctx = canvas.getContext("2d");
const W = canvas.width;
const H = canvas.height;

const gravitySlider = document.getElementById("gravity");
const restitutionSlider = document.getElementById("restitution");
const frictionSlider = document.getElementById("friction");
const gravityLabel = document.getElementById("gravityLabel");
const restitutionLabel = document.getElementById("restitutionLabel");
const frictionLabel = document.getElementById("frictionLabel");
const bodyCountLabel = document.getElementById("bodyCount");
const fpsLabel = document.getElementById("fps");
const clearBtn = document.getElementById("clearBtn");
const rainBtn = document.getElementById("rainBtn");

const world = new World({ gravity: new Vec2(0, Number(gravitySlider.value)) });

const WALL = 20;
const dynamicColors = ["#5b8def", "#e6543a", "#f2b134", "#33b679", "#a06be0"];

function addStaticWalls() {
  // floor
  world.addBody(
    new Body({
      shape: { type: "box", width: W + WALL * 4, height: WALL },
      position: new Vec2(W / 2, H - WALL / 2),
      isStatic: true,
      friction: 0.5,
      restitution: 0.3,
    })
  );
  // left wall
  world.addBody(
    new Body({
      shape: { type: "box", width: WALL, height: H + WALL * 4 },
      position: new Vec2(WALL / 2, H / 2),
      isStatic: true,
      friction: 0.5,
      restitution: 0.3,
    })
  );
  // right wall
  world.addBody(
    new Body({
      shape: { type: "box", width: WALL, height: H + WALL * 4 },
      position: new Vec2(W - WALL / 2, H / 2),
      isStatic: true,
      friction: 0.5,
      restitution: 0.3,
    })
  );
  // a small static ledge, just to make the scene more interesting
  world.addBody(
    new Body({
      shape: { type: "box", width: 160, height: 16 },
      position: new Vec2(W * 0.65, H * 0.6),
      isStatic: true,
      friction: 0.4,
      restitution: 0.3,
    })
  );
}

addStaticWalls();
const staticCount = world.bodies.length;

function currentTuning() {
  return {
    restitution: Number(restitutionSlider.value),
    friction: Number(frictionSlider.value),
  };
}

function spawnCircle(x, y) {
  const { restitution, friction } = currentTuning();
  const radius = 8 + Math.random() * 16;
  const body = new Body({
    shape: { type: "circle", radius },
    position: new Vec2(x, y),
    velocity: new Vec2((Math.random() - 0.5) * 80, 0),
    mass: radius * radius * 0.05,
    restitution,
    friction,
  });
  body.color = dynamicColors[Math.floor(Math.random() * dynamicColors.length)];
  world.addBody(body);
}

function spawnBox(x, y) {
  const { restitution, friction } = currentTuning();
  const size = 16 + Math.random() * 24;
  const body = new Body({
    shape: { type: "box", width: size, height: size },
    position: new Vec2(x, y),
    velocity: new Vec2((Math.random() - 0.5) * 80, 0),
    mass: size * size * 0.02,
    restitution,
    friction,
  });
  body.color = dynamicColors[Math.floor(Math.random() * dynamicColors.length)];
  world.addBody(body);
}

canvas.addEventListener("click", (e) => {
  const rect = canvas.getBoundingClientRect();
  const x = e.clientX - rect.left;
  const y = e.clientY - rect.top;
  if (e.shiftKey) spawnBox(x, y);
  else spawnCircle(x, y);
});

clearBtn.addEventListener("click", () => {
  world.bodies = world.bodies.slice(0, staticCount);
});

rainBtn.addEventListener("click", () => {
  for (let i = 0; i < 20; i++) {
    const x = 40 + Math.random() * (W - 80);
    const y = -Math.random() * 200;
    if (Math.random() < 0.5) spawnCircle(x, y);
    else spawnBox(x, y);
  }
});

gravitySlider.addEventListener("input", () => {
  world.gravity = new Vec2(0, Number(gravitySlider.value));
  gravityLabel.textContent = gravitySlider.value;
});

restitutionSlider.addEventListener("input", () => {
  restitutionLabel.textContent = restitutionSlider.value;
});

frictionSlider.addEventListener("input", () => {
  frictionLabel.textContent = frictionSlider.value;
});

gravityLabel.textContent = gravitySlider.value;
restitutionLabel.textContent = restitutionSlider.value;
frictionLabel.textContent = frictionSlider.value;

function draw() {
  ctx.clearRect(0, 0, W, H);
  ctx.fillStyle = "#11151c";
  ctx.fillRect(0, 0, W, H);

  for (const body of world.bodies) {
    ctx.fillStyle = body.color || "#555f6e";
    if (body.shape.type === "circle") {
      ctx.beginPath();
      ctx.arc(body.position.x, body.position.y, body.shape.radius, 0, Math.PI * 2);
      ctx.fill();
    } else {
      const hw = body.shape.width / 2;
      const hh = body.shape.height / 2;
      ctx.fillRect(body.position.x - hw, body.position.y - hh, body.shape.width, body.shape.height);
    }
  }
}

const MAX_DT = 1 / 30;
let lastTime = performance.now();
let frames = 0;
let fpsTimer = 0;

function loop(now) {
  const dt = Math.min((now - lastTime) / 1000, MAX_DT);
  lastTime = now;

  world.step(dt);
  draw();

  frames++;
  fpsTimer += dt;
  if (fpsTimer >= 0.5) {
    fpsLabel.textContent = Math.round(frames / fpsTimer);
    frames = 0;
    fpsTimer = 0;
  }
  bodyCountLabel.textContent = String(world.bodies.length - staticCount);

  requestAnimationFrame(loop);
}

requestAnimationFrame(loop);
