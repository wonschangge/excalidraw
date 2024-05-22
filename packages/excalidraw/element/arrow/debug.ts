import { normalize, scaleVector } from "../../math";
import { Point, Segment, Vector } from "../../types";
import { BoundingBox, Bounds } from "../bounds";

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
  segments?: Readonly<Segment> | Readonly<Segment>[] | null,
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

const isSegment = (
  candidate: Readonly<Segment> | Readonly<Segment>[],
): candidate is Readonly<Segment> =>
  candidate.length > 0 ? !Array.isArray(candidate[0][0]) : true;

export function debugDrawPoint(
  p: Point,
  color: string = "#FF1493",
  fuzzy = false,
) {
  if (import.meta.env.DEV) {
    const xOffset = fuzzy ? Math.random() * 3 : 0;
    const yOffset = fuzzy ? Math.random() * 3 : 0;

    window.v.lines.push([
      [p[0] + xOffset - 10, p[1] + yOffset - 10],
      [p[0] + xOffset + 10, p[1] + yOffset + 10],
      color,
    ]);
    window.v.lines.push([
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

export const debugDrawBounds = (bbox: Bounds) => {
  debugDrawSegments([
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
  ]);
};
