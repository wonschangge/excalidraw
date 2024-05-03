import {
  arePointsEqual,
  doSegmentsIntersect,
  dot,
  isValueInRange,
  normalize,
  pointToVector,
} from "../math";
import { Point, Vector } from "../types";
import { Bounds } from "./bounds";
import { mutateElement } from "./mutateElement";
import { ExcalidrawArrowElement } from "./types";

const PRECISION = 0.000005;

type RoutingKernel = (
  points: Point[],
  target: Point,
  boundingBoxes: Bounds[],
) => Point;
type LocalPoint = [number, number];

// ========================================
// The main idea is to Ray March the arrow
// ========================================

/// Recalculates the points of the arrow, except the start point
export const routeArrow = (
  arrow: ExcalidrawArrowElement,
  firstPointIsStart: boolean,
  target: Point,
  boundingBoxes: Bounds[],
) => {
  const firstPoint = firstPointIsStart
    ? (arrow.points[0] as LocalPoint)
    : (arrow.points[arrow.points.length - 1] as LocalPoint);
  const points = [toWorldSpace(arrow, firstPoint)];

  // Limit max step to avoid infinite loop
  for (let step = 0; step < 5; step++) {
    const next = kernel(points, target, boundingBoxes);
    if (
      arePointsEqual(target, next) ||
      arePointsEqual(points[points.length - 1], next) // Shouldn't be needed
    ) {
      break;
    }
    points.push(next);
  }

  points.push(target);

  mutateElement(arrow, {
    points: points.map((point) => toLocalSpace(arrow, point)),
  });
};

const kernel = (
  points: Point[],
  target: Point,
  boundingBoxes: Bounds[],
): Point => {
  const last = points[points.length - 1];
  const segmentVector =
    points.length < 2
      ? ([1, 0] as Vector) // TODO: Fixed right attachment
      : normalize(
          pointToVector(points[points.length - 1], points[points.length - 2]),
        );
  const targetVector = normalize(
    pointToVector(target, points[points.length - 1]),
  );
  //console.log(segmentVector, targetVector);
  const targetDirection = dot(segmentVector, targetVector);
  const targetNormal = dot(
    rotateVector(segmentVector, Math.PI / 2),
    targetVector,
  );

  switch (true) {
    case targetDirection > PRECISION && targetNormal > PRECISION:
      // Right bottom
      return [target[0], last[1]];
    case targetDirection > PRECISION && targetNormal < PRECISION:
      // Right top
      return [target[0], last[1]];
    case targetDirection < PRECISION && targetNormal < PRECISION:
      // Left top
      return [last[0], target[1]];
    case targetDirection < PRECISION && targetNormal > PRECISION:
      // Left bottom
      return [last[0], target[1]];
  }

  return target;
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
  vector[0] * Math.cos(rads) - vector[1] * Math.sin(rads),
  vector[0] * Math.sin(rads) + vector[1] * Math.cos(rads),
];

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
