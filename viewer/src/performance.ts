export type PerformanceSnapshot = {
  frames: number;
  medianFrameMs: number;
  p95FrameMs: number;
  averageFps: number;
  p95Fps: number;
};

/** Small allocation-bounded frame-time recorder shared by benchmarks/debug UI. */
export class FramePerformanceMonitor {
  private readonly samples: number[] = [];

  constructor(private readonly capacity = 600) {}

  push(frameMs: number) {
    if (!Number.isFinite(frameMs) || frameMs <= 0) return;
    if (this.samples.length >= this.capacity) this.samples.shift();
    this.samples.push(frameMs);
  }

  clear() {
    this.samples.length = 0;
  }

  snapshot(): PerformanceSnapshot {
    if (!this.samples.length) {
      return { frames: 0, medianFrameMs: 0, p95FrameMs: 0, averageFps: 0, p95Fps: 0 };
    }
    const sorted = [...this.samples].sort((a, b) => a - b);
    const medianFrameMs = percentile(sorted, 0.5);
    const p95FrameMs = percentile(sorted, 0.95);
    const averageMs = this.samples.reduce((sum, value) => sum + value, 0) / this.samples.length;
    return {
      frames: this.samples.length,
      medianFrameMs,
      p95FrameMs,
      averageFps: 1000 / averageMs,
      p95Fps: 1000 / p95FrameMs,
    };
  }
}

function percentile(sorted: number[], value: number) {
  if (!sorted.length) return 0;
  const index = Math.max(0, Math.min(sorted.length - 1, Math.ceil(sorted.length * value) - 1));
  return sorted[index];
}
