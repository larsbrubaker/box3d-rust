// Reusable UI control builders (mirrors box2d-rust demo/src/controls.ts).

export function createSlider(
  label: string, min: number, max: number, value: number, step: number,
  onChange: (val: number) => void,
): HTMLElement {
  const group = document.createElement("div");
  group.className = "control-group";
  const lbl = document.createElement("label");
  const labelText = document.createTextNode(label);
  const valSpan = document.createElement("span");
  valSpan.className = "slider-value";
  valSpan.textContent = formatSliderValue(value, step);
  lbl.appendChild(labelText);
  lbl.appendChild(valSpan);

  const input = document.createElement("input");
  input.type = "range";
  input.min = String(min);
  input.max = String(max);
  input.step = String(step);
  input.value = String(value);
  input.addEventListener("input", () => {
    const v = parseFloat(input.value);
    valSpan.textContent = formatSliderValue(v, step);
    onChange(v);
  });

  group.appendChild(lbl);
  group.appendChild(input);
  return group;
}

function formatSliderValue(v: number, step: number): string {
  if (step >= 1) return String(Math.round(v));
  if (step >= 0.1) return v.toFixed(1);
  return String(v);
}

export function createButton(label: string, onClick: () => void, active = false): HTMLButtonElement {
  const btn = document.createElement("button");
  btn.className = "control-btn" + (active ? " active" : "");
  btn.textContent = label;
  btn.addEventListener("click", onClick);
  return btn;
}

export function createCheckbox(
  label: string,
  checked: boolean,
  onChange: (val: boolean) => void,
  opts: { disabled?: boolean; title?: string } = {},
): HTMLElement {
  const wrap = document.createElement("label");
  wrap.className = "control-checkbox";
  if (opts.title) wrap.title = opts.title;
  const input = document.createElement("input");
  input.type = "checkbox";
  input.checked = checked;
  if (opts.disabled) input.disabled = true;
  input.addEventListener("change", () => onChange(input.checked));
  wrap.appendChild(input);
  wrap.appendChild(document.createTextNode(label));
  return wrap;
}

export function createButtonGroup(
  buttons: { label: string; value: string }[],
  defaultValue: string,
  onChange: (val: string) => void,
): HTMLElement {
  const row = document.createElement("div");
  row.className = "control-row";
  let activeBtn: HTMLButtonElement | null = null;

  for (const b of buttons) {
    const btn = createButton(b.label, () => {
      activeBtn?.classList.remove("active");
      btn.classList.add("active");
      activeBtn = btn;
      onChange(b.value);
    }, b.value === defaultValue);
    if (b.value === defaultValue) activeBtn = btn;
    row.appendChild(btn);
  }
  return row;
}

export function createInfoBox(html: string): HTMLElement {
  const box = document.createElement("div");
  box.className = "info-box";
  box.innerHTML = html;
  return box;
}

/** Create a `.sample-draw-text` HUD overlay pinned over the demo canvas and return it.
 *  Mirrors the C samples drawing text lines directly over the viewport (`DrawTextLine`). */
export function createCanvasOverlay(page: HTMLElement): HTMLElement {
  const canvasArea = page.querySelector(".demo-canvas-area") as HTMLElement;
  const overlay = document.createElement("div");
  overlay.className = "sample-draw-text";
  canvasArea.appendChild(overlay);
  return overlay;
}

/** Two significant figures, matching C `printf("%.2g")` for sample HUD readouts. */
export function fmt2g(x: number): string {
  if (x === 0) return "0";
  return parseFloat(x.toPrecision(2)).toString();
}

export function createReadout(): HTMLElement {
  const box = document.createElement("div");
  box.className = "info-readout";
  return box;
}

export function updateReadout(el: HTMLElement, entries: { label: string; value: string }[]) {
  el.innerHTML = entries.map((e) =>
    `<span class="label">${e.label}:</span> <span class="value">${e.value}</span>`
  ).join("<br>");
}

export function createTextInput(
  label: string,
  value: string,
  onChange: (val: string) => void,
): { root: HTMLElement; input: HTMLInputElement } {
  const group = document.createElement("div");
  group.className = "control-group";
  const lbl = document.createElement("label");
  lbl.textContent = label;
  const input = document.createElement("input");
  input.type = "text";
  input.className = "control-text";
  input.value = value;
  input.addEventListener("input", () => onChange(input.value));
  group.appendChild(lbl);
  group.appendChild(input);
  return { root: group, input };
}

/** Collapsing section matching C ImGui CollapsingHeader (open by default). */
export function createCollapsingSection(
  title: string,
  open = true,
): { root: HTMLElement; body: HTMLElement; setOpen: (v: boolean) => void } {
  const root = document.createElement("div");
  root.className = "collapse-section" + (open ? " open" : "");

  const header = document.createElement("button");
  header.type = "button";
  header.className = "collapse-header";
  header.innerHTML = `<span class="collapse-chevron">▾</span><span>${title}</span>`;

  const body = document.createElement("div");
  body.className = "collapse-body";

  const setOpen = (v: boolean) => {
    root.classList.toggle("open", v);
  };

  header.addEventListener("click", () => {
    setOpen(!root.classList.contains("open"));
  });

  root.append(header, body);
  return { root, body, setOpen };
}
