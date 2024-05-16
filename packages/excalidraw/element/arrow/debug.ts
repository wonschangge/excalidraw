import { normalize, scaleVector } from "../../math";
import { Segment, Vector } from "../../types";

export function debugDrawClear() {
  if (import.meta.env.DEV) {
    window.v.lines = [];
  }
}

export function debugDrawNormal(
  normal: Vector,
  segment: Segment,
  color: string = "cyan",
) {
  if (import.meta.env.DEV && normal && segment) {
    const [cx, cy] = [
      (segment[0][0] + segment[1][0]) / 2,
      (segment[0][1] + segment[1][1]) / 2,
    ];
    const [nx, ny] = scaleVector(normalize(normal), 20);
    window.v.lines.push([[cx, cy], [cx + nx, cy + ny], color]);
  }
}

export function debugDrawSegments(
  segments?: Segment | Segment[] | null,
  color: string = "green",
) {
  if (import.meta.env.DEV) {
    if (segments && !isSegment(segments)) {
      segments.forEach((segment) =>
        window.v.lines.push([segment[0], segment[1], color]),
      );
    } else if (segments) {
      window.v.lines.push([segments[0], segments[1], color]);
    }
  }
}

const isSegment = (candidate: Segment | Segment[]): candidate is Segment =>
  candidate.length > 0 ? !Array.isArray(candidate[0][0]) : true;
