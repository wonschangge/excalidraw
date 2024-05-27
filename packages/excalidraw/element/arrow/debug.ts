import { normalize, scaleVector } from "../../math";
import type { Point, Segment, Vector } from "../../types";
import type { BoundingBox, Bounds } from "../bounds";

export function debugClear() {
  if (import.meta.env.DEV || import.meta.env.MODE === "test") {
    window.v.clearFrames();
  }
}

export function debugNewFrame() {
  if (import.meta.env.DEV || import.meta.env.MODE === "test") {
    window.v.newFrame();
  }
}

export function debugDrawNormal(
  normal: Vector,
  segment: Segment,
  color: string = "cyan",
) {
  if (
    (import.meta.env.DEV || import.meta.env.MODE === "test") &&
    normal &&
    segment
  ) {
    const [cx, cy] = [
      (segment[0][0] + segment[1][0]) / 2,
      (segment[0][1] + segment[1][1]) / 2,
    ];
    const [nx, ny] = scaleVector(normalize(normal), 20);
    window.v.frames[window.v.frames.length - 1].push([
      [cx, cy],
      [cx + nx, cy + ny],
      color,
    ]);
  }
}

export function debugDrawSegments(
  segments?: Readonly<Segment> | Readonly<Segment>[] | null,
  color: string = "green",
) {
  if (import.meta.env.DEV || import.meta.env.MODE === "test") {
    if (segments && !isSegment(segments)) {
      segments.forEach((segment) =>
        window.v.frames[window.v.frames.length - 1].push([
          segment[0],
          segment[1],
          color,
        ]),
      );
    } else if (segments) {
      window.v.frames[window.v.frames.length - 1].push([
        segments[0],
        segments[1],
        color,
      ]);
    }
  }
}

const isSegment = (
  candidate: Readonly<Segment> | Readonly<Segment>[],
): candidate is Readonly<Segment> =>
  candidate.length > 0 ? !Array.isArray(candidate[0][0]) : true;

export function debugDrawPoint(
  p: Point,
  color: string = "#FF1493",
  fuzzy = false,
) {
  if (import.meta.env.DEV || import.meta.env.MODE === "test") {
    const xOffset = fuzzy ? Math.random() * 3 : 0;
    const yOffset = fuzzy ? Math.random() * 3 : 0;

    window.v.frames[window.v.frames.length - 1].push([
      [p[0] + xOffset - 10, p[1] + yOffset - 10],
      [p[0] + xOffset + 10, p[1] + yOffset + 10],
      color,
    ]);
    window.v.frames[window.v.frames.length - 1].push([
      [p[0] + xOffset - 10, p[1] + yOffset + 10],
      [p[0] + xOffset + 10, p[1] + yOffset - 10],
      color,
    ]);
  }
}

export const debugDrawBoundingBox = (bbox: BoundingBox) => {
  debugDrawSegments([
    [
      [bbox.minX, bbox.minY],
      [bbox.maxX, bbox.minY],
    ],
    [
      [bbox.maxX, bbox.minY],
      [bbox.maxX, bbox.maxY],
    ],
    [
      [bbox.maxX, bbox.maxY],
      [bbox.minX, bbox.maxY],
    ],
    [
      [bbox.minX, bbox.maxY],
      [bbox.minX, bbox.minY],
    ],
  ]);
};

export const debugDrawBounds = (bbox: Bounds, color: string = "green") => {
  debugDrawSegments(
    [
      [
        [bbox[0], bbox[1]],
        [bbox[2], bbox[1]],
      ],
      [
        [bbox[2], bbox[1]],
        [bbox[2], bbox[3]],
      ],
      [
        [bbox[2], bbox[3]],
        [bbox[0], bbox[3]],
      ],
      [
        [bbox[0], bbox[3]],
        [bbox[0], bbox[1]],
      ],
    ],
    color,
  );
};
