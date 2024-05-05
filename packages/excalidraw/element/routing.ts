import { arePointsEqual, dot, normalize, pointToVector } from "../math";
import { Point, Vector } from "../types";
import { Bounds } from "./bounds";
import { mutateElement } from "./mutateElement";
import { ExcalidrawArrowElement } from "./types";

type LocalPoint = [number, number];

// ========================================
// The main idea is to Ray March the arrow
// ========================================

/// Recalculates the points of the arrow, except the start point
export const routeArrow = (
  arrow: ExcalidrawArrowElement,
  target: Point,
  boundingBoxes: Bounds[],
) => {
  const firstPoint = arrow.points[0] as LocalPoint;
  const points = [
    toWorldSpace(arrow, firstPoint),
    toWorldSpace(arrow, [firstPoint[0] - 40, firstPoint[1]]),
  ];
  const endPoints = [[target[0] + 40, target[1]], target] as Point[];
  console.log("----");
  // Limit max step to avoid infinite loop
  for (let step = 0; step < 5; step++) {
    const next = kernel(points, endPoints, boundingBoxes);
    if (arePointsEqual(endPoints[0], next)) {
      break;
    }
    points.push(next);
  }

  points.push(endPoints[0]);
  points.push(endPoints[1]);

  mutateElement(arrow, {
    points: points.map((point) => toLocalSpace(arrow, point)),
  });
};

const kernel = (
  points: Point[],
  target: Point[],
  boundingBoxes: Bounds[],
): Point => {
  const start = points[points.length - 1];
  const end = target[0];
  const startVector =
    points.length < 2
      ? ([1, 0] as Vector) // TODO: Fixed right start attachment
      : normalize(pointToVector(start, points[points.length - 2]));
  const endVector =
    target.length < 2
      ? ([-1, 0] as Vector) // TODO: Fixed left end attachment
      : normalize(pointToVector(target[1], end));
  const rightStartNormalDot = dot(
    [1, 0],
    rotateVector(startVector, Math.PI / 2),
  );

  const next: Point =
    rightStartNormalDot === 0
      ? [start[0], end[1]] // Last segment from start is horizontal
      : [end[0], start[1]]; // Last segment from start is vertical
  const nextVector = normalize(pointToVector(next, end));
  const nextEndDot = dot(nextVector, endVector);

  if (nextEndDot === 1) {
    // Facing opposite - make a half pass toward the target
    return rightStartNormalDot === 0
      ? [start[0], start[1] + (end[1] - start[1]) / 2]
      : [start[0] + (end[0] - start[0]) / 2, start[1]];
  }

  return next;
};

const toLocalSpace = (arrow: ExcalidrawArrowElement, p: Point): LocalPoint => [
  p[0] - arrow.x,
  p[1] - arrow.y,
];

const toWorldSpace = (arrow: ExcalidrawArrowElement, p: LocalPoint): Point => [
  p[0] + arrow.x,
  p[1] + arrow.y,
];

const rotateVector = (vector: Vector, rads: number): Vector => [
  cutoff(vector[0] * Math.cos(rads) - vector[1] * Math.sin(rads)),
  cutoff(vector[0] * Math.sin(rads) + vector[1] * Math.cos(rads)),
];

const cutoff = (num: number): number =>
  Math.round(num * 1000000000) / 1000000000;

/*
 * Implement a simplified A* heuristics
 *
const avoidanceKernel = (
  origin: Point,
  target: Point,
  boundingBoxes: Bounds[],
): Point => {
  const targetUnitVec = normalize(pointToVector(target, origin));
  const rightUnitVec = [1, 0] as Vector;

  if (Math.abs(dot(rightUnitVec, targetUnitVec)) < 0.5) {
    // Horizontal
    const horizontalPoint = [target[0], origin[1]] as Point;
    const horizontalObstacle = naiveRaycast(
      origin,
      horizontalPoint,
      boundingBoxes,
    );
    if (horizontalObstacle) {
      const verticalPoint = [origin[0], target[1]] as Point;
      const verticalObstacle = naiveRaycast(
        origin,
        verticalPoint,
        boundingBoxes,
      );
      if (horizontalObstacle && verticalObstacle) {
        // TODO: We don't consider going around in the opposite direction yet
        const y =
          origin[1] > verticalObstacle[1]
            ? verticalObstacle[1] - 10 // Bumped into top
            : verticalObstacle[3] + 10; // Bumped into bottom
        return [verticalPoint[0], y];
      }

      return verticalPoint;
    }

    return horizontalPoint;
  }

  // Vertical
  const verticalPoint = [origin[0], target[1]] as Point;
  const verticalObstacle = naiveRaycast(origin, verticalPoint, boundingBoxes);
  if (verticalObstacle) {
    const horizontalPoint = [target[0], origin[1]] as Point;
    const horizontalObstacle = naiveRaycast(
      origin,
      horizontalPoint,
      boundingBoxes,
    );
    if (verticalObstacle && horizontalObstacle) {
      // TODO: We don't consider going around in the opposite direction yet
      const x =
        origin[0] > horizontalObstacle[0]
          ? horizontalObstacle[0] + 10 // Bumped into left
          : horizontalObstacle[2] - 10; // Bumped into right
      return [x, horizontalPoint[1]];
    }

    return horizontalPoint;
  }

  return verticalPoint;
};

const naiveRaycast = (
  origin: Point,
  target: Point,
  boundingBoxes: Bounds[],
) => {
  // Optimization assumptions:
  // 1) We only test against bounding boxes
  // 2) Bounding boxes are always axis-aligned
  // 3) Arrow segments are always axis-aligned
  //
  // Therefore we only test against perpendicular sides to the actual arrow segment
  const dx = target[0] - origin[0];
  const dy = target[1] - origin[1];

  switch (true) {
    case dx < -PRECISION:
      return naiveRaycastLeft(origin, target, boundingBoxes);
    case dx > PRECISION:
      return naiveRaycastRight(origin, target, boundingBoxes);
    case dy < -PRECISION:
      return naiveRaycastTop(origin, target, boundingBoxes);
    case dy > PRECISION:
      return naiveRaycastBottom(origin, target, boundingBoxes);
    default:
      //console.error("origin and target are not axis-aligned!");
      return [];
  }
};

// Check right sides of bounding boxes against a left pointing ray
const naiveRaycastLeft = (
  origin: Point,
  target: Point,
  boundingBoxes: Bounds[],
) => {
  const hits = boundingBoxes.filter((box) =>
    doSegmentsIntersect(origin, target, [box[2], box[1]], [box[2], box[3]]),
  );
  hits.sort((a, b) => a[2] - b[2]);

  return hits.pop();
};

// Check left sides of bounding boxes against a right pointing ray
const naiveRaycastRight = (
  origin: Point,
  target: Point,
  boundingBoxes: Bounds[],
) => {
  const hits = boundingBoxes.filter((box) =>
    doSegmentsIntersect(origin, target, [box[0], box[1]], [box[0], box[3]]),
  );
  hits.sort((a, b) => a[0] - b[0]);

  return hits.pop();
};

// Check bottom sides of bounding boxes against a top pointing ray
const naiveRaycastTop = (
  origin: Point,
  target: Point,
  boundingBoxes: Bounds[],
) => {
  const hits = boundingBoxes.filter((box) =>
    doSegmentsIntersect(origin, target, [box[0], box[3]], [box[2], box[3]]),
  );
  hits.sort((a, b) => a[3] - b[3]);

  return hits.pop();
};

// Check top sides of bounding boxes against a bottom pointing ray
const naiveRaycastBottom = (
  origin: Point,
  target: Point,
  boundingBoxes: Bounds[],
) => {
  const hits = boundingBoxes.filter((box) =>
    doSegmentsIntersect(origin, target, [box[0], box[1]], [box[2], box[1]]),
  );
  hits.sort((a, b) => a[1] - b[1]);

  return hits.pop();
};

*/
