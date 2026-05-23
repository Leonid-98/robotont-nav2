const defaultWorld = {
  version: 1,
  name: "robotont_room",
  resolution: 0.05,
  origin: { x: -3.0, y: -2.0 },
  bounds: { min_x: -3.0, min_y: -2.0, max_x: 3.0, max_y: 2.0 },
  walls: [
    { x1: -3.0, y1: -2.0, x2: 3.0, y2: -2.0 },
    { x1: 3.0, y1: -2.0, x2: 3.0, y2: 2.0 },
    { x1: 3.0, y1: 2.0, x2: -3.0, y2: 2.0 },
    { x1: -3.0, y1: 2.0, x2: -3.0, y2: -2.0 }
  ],
  boxes: [
    { name: "obstacle_1", min_x: -1.2, min_y: -0.8, max_x: -0.8, max_y: 0.8 },
    { name: "obstacle_2", min_x: 0.8, min_y: -1.3, max_x: 1.2, max_y: -0.2 },
    { name: "obstacle_3", min_x: 1.6, min_y: 0.6, max_x: 2.2, max_y: 1.1 }
  ]
};

const canvas = document.getElementById("worldCanvas");
const ctx = canvas.getContext("2d");
const statusEl = document.getElementById("status");
const jsonPreview = document.getElementById("jsonPreview");
const selectionForm = document.getElementById("selectionForm");
const worldName = document.getElementById("worldName");
const resolution = document.getElementById("resolution");
const minX = document.getElementById("minX");
const minY = document.getElementById("minY");
const maxX = document.getElementById("maxX");
const maxY = document.getElementById("maxY");

let world = structuredClone(defaultWorld);
let tool = "select";
let selected = null;
let draft = null;
let dragging = false;

function setStatus(text) {
  statusEl.textContent = text;
}

function metersPerPixel() {
  const bounds = getBounds();
  const width = bounds.max_x - bounds.min_x;
  const height = bounds.max_y - bounds.min_y;
  return Math.max(width / canvas.width, height / canvas.height);
}

function getBounds() {
  return {
    min_x: Number(minX.value),
    min_y: Number(minY.value),
    max_x: Number(maxX.value),
    max_y: Number(maxY.value)
  };
}

function worldToCanvas(point) {
  const bounds = getBounds();
  const scale = Math.min(
    canvas.width / (bounds.max_x - bounds.min_x),
    canvas.height / (bounds.max_y - bounds.min_y)
  );
  const worldWidth = bounds.max_x - bounds.min_x;
  const worldHeight = bounds.max_y - bounds.min_y;
  const offsetX = (canvas.width - worldWidth * scale) / 2;
  const offsetY = (canvas.height - worldHeight * scale) / 2;
  return {
    x: offsetX + (point.x - bounds.min_x) * scale,
    y: offsetY + (bounds.max_y - point.y) * scale
  };
}

function canvasToWorld(point) {
  const bounds = getBounds();
  const scale = Math.min(
    canvas.width / (bounds.max_x - bounds.min_x),
    canvas.height / (bounds.max_y - bounds.min_y)
  );
  const worldWidth = bounds.max_x - bounds.min_x;
  const worldHeight = bounds.max_y - bounds.min_y;
  const offsetX = (canvas.width - worldWidth * scale) / 2;
  const offsetY = (canvas.height - worldHeight * scale) / 2;
  return {
    x: roundToGrid(bounds.min_x + (point.x - offsetX) / scale),
    y: roundToGrid(bounds.max_y - (point.y - offsetY) / scale)
  };
}

function roundToGrid(value) {
  return Math.round(value / 0.05) * 0.05;
}

function pointerPosition(event) {
  const rect = canvas.getBoundingClientRect();
  return {
    x: (event.clientX - rect.left) * (canvas.width / rect.width),
    y: (event.clientY - rect.top) * (canvas.height / rect.height)
  };
}

function normalizedBox(box) {
  return {
    name: box.name || `obstacle_${world.boxes.length + 1}`,
    min_x: Math.min(Number(box.min_x), Number(box.max_x)),
    min_y: Math.min(Number(box.min_y), Number(box.max_y)),
    max_x: Math.max(Number(box.min_x), Number(box.max_x)),
    max_y: Math.max(Number(box.min_y), Number(box.max_y))
  };
}

function currentWorld() {
  return {
    version: 1,
    name: worldName.value || "robotont_world",
    resolution: Number(resolution.value) || 0.05,
    origin: {
      x: Number(minX.value),
      y: Number(minY.value)
    },
    bounds: getBounds(),
    walls: world.walls.map((wall) => ({
      x1: Number(wall.x1),
      y1: Number(wall.y1),
      x2: Number(wall.x2),
      y2: Number(wall.y2)
    })),
    boxes: world.boxes.map(normalizedBox)
  };
}

function syncInputsFromWorld() {
  worldName.value = world.name || "robotont_world";
  resolution.value = world.resolution || 0.05;
  const bounds = world.bounds || defaultWorld.bounds;
  minX.value = bounds.min_x;
  minY.value = bounds.min_y;
  maxX.value = bounds.max_x;
  maxY.value = bounds.max_y;
}

function updatePreview() {
  jsonPreview.value = JSON.stringify(currentWorld(), null, 2);
}

function drawGrid() {
  const bounds = getBounds();
  const step = 0.5;
  ctx.lineWidth = 1;
  ctx.strokeStyle = "#edf1f5";
  ctx.fillStyle = "#667789";
  ctx.font = "12px sans-serif";

  for (let x = Math.ceil(bounds.min_x / step) * step; x <= bounds.max_x; x += step) {
    const a = worldToCanvas({ x, y: bounds.min_y });
    const b = worldToCanvas({ x, y: bounds.max_y });
    ctx.beginPath();
    ctx.moveTo(a.x, a.y);
    ctx.lineTo(b.x, b.y);
    ctx.stroke();
  }

  for (let y = Math.ceil(bounds.min_y / step) * step; y <= bounds.max_y; y += step) {
    const a = worldToCanvas({ x: bounds.min_x, y });
    const b = worldToCanvas({ x: bounds.max_x, y });
    ctx.beginPath();
    ctx.moveTo(a.x, a.y);
    ctx.lineTo(b.x, b.y);
    ctx.stroke();
  }

  const origin = worldToCanvas({ x: 0, y: 0 });
  ctx.strokeStyle = "#c8d2dc";
  ctx.beginPath();
  ctx.moveTo(origin.x, 0);
  ctx.lineTo(origin.x, canvas.height);
  ctx.moveTo(0, origin.y);
  ctx.lineTo(canvas.width, origin.y);
  ctx.stroke();
}

function drawWall(wall, active = false) {
  const a = worldToCanvas({ x: wall.x1, y: wall.y1 });
  const b = worldToCanvas({ x: wall.x2, y: wall.y2 });
  ctx.lineWidth = active ? 6 : 4;
  ctx.strokeStyle = active ? "#1677b8" : "#202833";
  ctx.lineCap = "round";
  ctx.beginPath();
  ctx.moveTo(a.x, a.y);
  ctx.lineTo(b.x, b.y);
  ctx.stroke();
}

function drawBox(box, active = false) {
  const normalized = normalizedBox(box);
  const a = worldToCanvas({ x: normalized.min_x, y: normalized.max_y });
  const b = worldToCanvas({ x: normalized.max_x, y: normalized.min_y });
  ctx.lineWidth = active ? 4 : 2;
  ctx.strokeStyle = active ? "#1677b8" : "#3c8d6d";
  ctx.fillStyle = active ? "rgba(22, 119, 184, 0.22)" : "rgba(60, 141, 109, 0.26)";
  ctx.beginPath();
  ctx.rect(a.x, a.y, b.x - a.x, b.y - a.y);
  ctx.fill();
  ctx.stroke();
}

function render() {
  ctx.clearRect(0, 0, canvas.width, canvas.height);
  drawGrid();

  world.boxes.forEach((box, index) => {
    drawBox(box, selected && selected.type === "box" && selected.index === index);
  });

  world.walls.forEach((wall, index) => {
    drawWall(wall, selected && selected.type === "wall" && selected.index === index);
  });

  if (draft) {
    if (draft.type === "wall") drawWall(draft, true);
    if (draft.type === "box") drawBox(draft, true);
  }

  updateSelectionForm();
  updatePreview();
}

function distanceToSegment(point, wall) {
  const a = worldToCanvas({ x: wall.x1, y: wall.y1 });
  const b = worldToCanvas({ x: wall.x2, y: wall.y2 });
  const dx = b.x - a.x;
  const dy = b.y - a.y;
  const lengthSq = dx * dx + dy * dy;
  if (lengthSq === 0) return Math.hypot(point.x - a.x, point.y - a.y);
  const t = Math.max(0, Math.min(1, ((point.x - a.x) * dx + (point.y - a.y) * dy) / lengthSq));
  return Math.hypot(point.x - (a.x + t * dx), point.y - (a.y + t * dy));
}

function selectAt(canvasPoint) {
  for (let index = world.walls.length - 1; index >= 0; index -= 1) {
    if (distanceToSegment(canvasPoint, world.walls[index]) < 10) {
      return { type: "wall", index };
    }
  }

  const point = canvasToWorld(canvasPoint);
  for (let index = world.boxes.length - 1; index >= 0; index -= 1) {
    const box = normalizedBox(world.boxes[index]);
    if (point.x >= box.min_x && point.x <= box.max_x && point.y >= box.min_y && point.y <= box.max_y) {
      return { type: "box", index };
    }
  }

  return null;
}

function updateSelectionForm() {
  if (!selected) {
    selectionForm.className = "selection empty";
    selectionForm.textContent = "Nothing selected";
    return;
  }

  selectionForm.className = "selection";
  const item = selected.type === "wall" ? world.walls[selected.index] : world.boxes[selected.index];
  const fields = selected.type === "wall"
    ? ["x1", "y1", "x2", "y2"]
    : ["name", "min_x", "min_y", "max_x", "max_y"];

  selectionForm.innerHTML = "";
  fields.forEach((field) => {
    const label = document.createElement("label");
    label.textContent = field;
    const input = document.createElement("input");
    input.value = item[field] ?? "";
    input.type = field === "name" ? "text" : "number";
    if (input.type === "number") input.step = "0.05";
    input.addEventListener("change", () => {
      item[field] = input.type === "number" ? Number(input.value) : input.value;
      render();
    });
    label.appendChild(input);
    selectionForm.appendChild(label);
  });
}

function setTool(nextTool) {
  tool = nextTool;
  document.querySelectorAll(".tool").forEach((button) => {
    button.classList.toggle("active", button.dataset.tool === tool);
  });
  setStatus(tool[0].toUpperCase() + tool.slice(1));
}

function onPointerDown(event) {
  const canvasPoint = pointerPosition(event);
  const worldPoint = canvasToWorld(canvasPoint);
  dragging = true;

  if (tool === "select") {
    selected = selectAt(canvasPoint);
    render();
    return;
  }

  selected = null;
  if (tool === "wall") {
    draft = { type: "wall", x1: worldPoint.x, y1: worldPoint.y, x2: worldPoint.x, y2: worldPoint.y };
  }
  if (tool === "box") {
    draft = { type: "box", name: `obstacle_${world.boxes.length + 1}`, min_x: worldPoint.x, min_y: worldPoint.y, max_x: worldPoint.x, max_y: worldPoint.y };
  }
  render();
}

function onPointerMove(event) {
  if (!dragging || !draft) return;
  const worldPoint = canvasToWorld(pointerPosition(event));
  if (draft.type === "wall") {
    draft.x2 = worldPoint.x;
    draft.y2 = worldPoint.y;
  }
  if (draft.type === "box") {
    draft.max_x = worldPoint.x;
    draft.max_y = worldPoint.y;
  }
  render();
}

function onPointerUp() {
  if (!dragging) return;
  dragging = false;

  if (draft && draft.type === "wall") {
    const length = Math.hypot(draft.x2 - draft.x1, draft.y2 - draft.y1);
    if (length > 0.05) {
      world.walls.push({ x1: draft.x1, y1: draft.y1, x2: draft.x2, y2: draft.y2 });
      selected = { type: "wall", index: world.walls.length - 1 };
    }
  }

  if (draft && draft.type === "box") {
    const box = normalizedBox(draft);
    if ((box.max_x - box.min_x) > 0.05 && (box.max_y - box.min_y) > 0.05) {
      world.boxes.push(box);
      selected = { type: "box", index: world.boxes.length - 1 };
    }
  }

  draft = null;
  render();
}

function deleteSelected() {
  if (!selected) return;
  if (selected.type === "wall") world.walls.splice(selected.index, 1);
  if (selected.type === "box") world.boxes.splice(selected.index, 1);
  selected = null;
  render();
}

function loadWorld(nextWorld) {
  world = {
    version: 1,
    name: nextWorld.name || "robotont_world",
    resolution: nextWorld.resolution || 0.05,
    origin: nextWorld.origin || { x: -3, y: -2 },
    bounds: nextWorld.bounds || defaultWorld.bounds,
    walls: Array.isArray(nextWorld.walls) ? nextWorld.walls : [],
    boxes: Array.isArray(nextWorld.boxes) ? nextWorld.boxes : []
  };
  selected = null;
  draft = null;
  syncInputsFromWorld();
  render();
}

function downloadJson() {
  const data = JSON.stringify(currentWorld(), null, 2);
  const blob = new Blob([data], { type: "application/json" });
  const url = URL.createObjectURL(blob);
  const link = document.createElement("a");
  link.href = url;
  link.download = `${currentWorld().name || "robotont_world"}.json`;
  link.click();
  URL.revokeObjectURL(url);
}

document.querySelectorAll(".tool").forEach((button) => {
  button.addEventListener("click", () => setTool(button.dataset.tool));
});

document.getElementById("deleteSelected").addEventListener("click", deleteSelected);
document.getElementById("resetWorld").addEventListener("click", () => loadWorld(structuredClone(defaultWorld)));
document.getElementById("downloadJson").addEventListener("click", downloadJson);
document.getElementById("copyJson").addEventListener("click", async () => {
  const data = JSON.stringify(currentWorld(), null, 2);
  try {
    await navigator.clipboard.writeText(data);
    setStatus("Copied");
  } catch {
    jsonPreview.focus();
    jsonPreview.select();
    setStatus("Selected");
  }
});

document.getElementById("importFile").addEventListener("change", async (event) => {
  const file = event.target.files[0];
  if (!file) return;
  const text = await file.text();
  loadWorld(JSON.parse(text));
  setStatus("Imported");
});

[worldName, resolution, minX, minY, maxX, maxY].forEach((input) => {
  input.addEventListener("change", render);
});

canvas.addEventListener("pointerdown", onPointerDown);
canvas.addEventListener("pointermove", onPointerMove);
canvas.addEventListener("pointerup", onPointerUp);
canvas.addEventListener("pointerleave", onPointerUp);

syncInputsFromWorld();
render();
