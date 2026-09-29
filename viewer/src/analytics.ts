import { PlaybackFile, PlaybackFrame } from "./playback";

type NumericSummary = {
  elapsed_s?: number;
  total_agents?: number;
  evacuated_agents?: number;
  active_agents?: number;
  waiting_agents?: number;
  trapped_agents?: number;
  average_evacuation_time_s?: number | null;
  clearance_time_s?: number | null;
  max_queue?: number;
  exit_usage?: Record<string, number>;
};

type TimelinePoint = {
  time: number;
  moving: number;
  waiting: number;
  evacuated: number;
  trapped: number;
};

export class StudyAnalyticsPanel {
  constructor(private readonly host: HTMLElement) {
    this.clear();
  }

  clear() {
    this.host.innerHTML = `<div class="analytics-empty">Run or load a simulation to see evacuation analytics.</div>`;
  }

  render(data: PlaybackFile) {
    const summary = normalizeSummary(data.summary);
    const timeline = buildTimeline(data.frames ?? []);
    const total = summary.total_agents ?? timeline.at(-1)?.moving ?? 0;
    const clearance = finiteNumber(summary.clearance_time_s);
    const average = finiteNumber(summary.average_evacuation_time_s);
    const elapsed = finiteNumber(summary.elapsed_s) ?? data.frames?.at(-1)?.time_s ?? 0;
    const evacuated = summary.evacuated_agents ?? timeline.at(-1)?.evacuated ?? 0;
    const trapped = summary.trapped_agents ?? timeline.at(-1)?.trapped ?? 0;
    const maxQueue = summary.max_queue ?? 0;
    const exits = Object.entries(summary.exit_usage ?? {}).sort((a, b) => b[1] - a[1]);

    this.host.innerHTML = `
      <div class="analytics-metrics">
        ${metric("Agents", total)}
        ${metric("Evacuated", evacuated)}
        ${metric("Trapped", trapped)}
        ${metric("Clearance", clearance == null ? "—" : `${clearance.toFixed(1)} s`)}
        ${metric("Avg evac", average == null ? "—" : `${average.toFixed(1)} s`)}
        ${metric("Max queue", maxQueue)}
        ${metric("Duration", `${elapsed.toFixed(1)} s`)}
      </div>
      <div class="analytics-chart-wrap">
        <div class="analytics-title">Population state over time</div>
        ${timelineSvg(timeline)}
      </div>
      <div class="analytics-exits">
        <div class="analytics-title">Exit usage</div>
        ${exits.length ? exits.map(([id, count]) => `<div><span title="${escapeHtml(id)}">${escapeHtml(shortId(id))}</span><b>${count}</b></div>`).join("") : "<em>No exit usage reported.</em>"}
      </div>
    `;
  }
}

function normalizeSummary(value: Record<string, unknown> | undefined): NumericSummary {
  if (!value) return {};
  const result: NumericSummary = {};
  for (const key of [
    "elapsed_s", "total_agents", "evacuated_agents", "active_agents", "waiting_agents",
    "trapped_agents", "average_evacuation_time_s", "clearance_time_s", "max_queue",
  ] as const) {
    const raw = value[key];
    if (raw === null && (key === "average_evacuation_time_s" || key === "clearance_time_s")) {
      result[key] = null;
    } else if (typeof raw === "number" && Number.isFinite(raw)) {
      (result as Record<string, unknown>)[key] = raw;
    }
  }
  const exitUsage = value.exit_usage;
  if (exitUsage && typeof exitUsage === "object" && !Array.isArray(exitUsage)) {
    result.exit_usage = {};
    for (const [key, raw] of Object.entries(exitUsage as Record<string, unknown>)) {
      if (typeof raw === "number" && Number.isFinite(raw)) result.exit_usage[key] = raw;
    }
  }
  return result;
}

function buildTimeline(frames: PlaybackFrame[]): TimelinePoint[] {
  if (!frames.length) return [];
  const maxPoints = 180;
  const stride = Math.max(1, Math.ceil(frames.length / maxPoints));
  const sampled = frames.filter((_, index) => index % stride === 0 || index === frames.length - 1);
  return sampled.map((frame) => {
    let moving = 0, waiting = 0, evacuated = 0, trapped = 0;
    for (const agent of frame.agents ?? []) {
      if (agent.status === "evacuated") evacuated += 1;
      else if (agent.status === "trapped") trapped += 1;
      else if (agent.status === "waiting" || agent.status === "elevator_waiting") waiting += 1;
      else moving += 1;
    }
    return { time: frame.time_s, moving, waiting, evacuated, trapped };
  });
}

function timelineSvg(points: TimelinePoint[]) {
  if (!points.length) return `<div class="analytics-empty">No playback frames.</div>`;
  const width = 520, height = 130, pad = 20;
  const maxTime = Math.max(1e-6, points.at(-1)!.time);
  const maxAgents = Math.max(1, ...points.map((p) => p.moving + p.waiting + p.evacuated + p.trapped));
  const x = (time: number) => pad + (time / maxTime) * (width - pad * 2);
  const y = (value: number) => height - pad - (value / maxAgents) * (height - pad * 2);
  const series: Array<[keyof Omit<TimelinePoint, "time">, string, string]> = [
    ["moving", "Moving", "#2c7be5"],
    ["waiting", "Waiting", "#f59e0b"],
    ["evacuated", "Evacuated", "#2eb85c"],
    ["trapped", "Trapped", "#d9534f"],
  ];
  const lines = series.map(([key, label, color]) => {
    const path = points.map((point, index) => `${index ? "L" : "M"}${x(point.time).toFixed(1)},${y(point[key]).toFixed(1)}`).join(" ");
    return `<path d="${path}" fill="none" stroke="${color}" stroke-width="2"/><text x="${pad + series.findIndex((item) => item[0] === key) * 105}" y="12" fill="${color}" font-size="10">${label}</text>`;
  }).join("");
  return `<svg class="analytics-chart" viewBox="0 0 ${width} ${height}" role="img" aria-label="Evacuation population state timeline">
    <line x1="${pad}" y1="${height - pad}" x2="${width - pad}" y2="${height - pad}" stroke="#aab3bd"/>
    <line x1="${pad}" y1="${pad}" x2="${pad}" y2="${height - pad}" stroke="#aab3bd"/>
    ${lines}
    <text x="${width - pad}" y="${height - 4}" text-anchor="end" fill="#65707c" font-size="9">${maxTime.toFixed(1)} s</text>
    <text x="3" y="${pad + 3}" fill="#65707c" font-size="9">${maxAgents}</text>
  </svg>`;
}

function metric(label: string, value: string | number) {
  return `<div class="metric"><span>${escapeHtml(label)}</span><b>${escapeHtml(String(value))}</b></div>`;
}

function finiteNumber(value: number | null | undefined) {
  return typeof value === "number" && Number.isFinite(value) ? value : null;
}

function shortId(value: string) {
  return value.length > 28 ? `${value.slice(0, 12)}…${value.slice(-10)}` : value;
}

function escapeHtml(value: string) {
  return value.replace(/[&<>'"]/g, (char) => ({ "&": "&amp;", "<": "&lt;", ">": "&gt;", "'": "&#39;", '"': "&quot;" }[char]!));
}
