import { arePointsEqual, dot, normalize, pointToVector } from "../../math";
import { LocalPoint, Point, Vector } from "../../types";
import { ExcalidrawArrowElement } from "../types";
import { Bounds, getElementBounds } from "../bounds";
import Scene from "../../scene/Scene";

// ========================================
// The main idea is to Ray March the arrow
// ========================================

const generatePointPairs = (points: Readonly<Point[]>) => {
  const [first, ...restOfThePoints] = points;
  let latest = first;

  return restOfThePoints.map((point) => {
    const res = [latest, point];
    latest = point;
    return res;
  }) as [Point, Point][];
};

export const calculatePoints = (
  arrow: ExcalidrawArrowElement,
): readonly LocalPoint[] => {
  if (arrow.points.length < 2) {
    // Arrow being created
    return arrow.points;
  }

  const target = arrow.points[arrow.points.length - 1] as LocalPoint;
  const firstPoint = arrow.points[0] as LocalPoint;

  const boundingBoxes = getStartEndBounds(arrow);
  const [startBox, endBox] = boundingBoxes;
  const startCenter = startBox && getCenter(startBox);
  const endCenter = endBox && getCenter(endBox);

  const points = [toWorldSpace(arrow, firstPoint)];
  // if (startHeading) {
  //   const startDongle = toWorldSpace(
  //     arrow,
  //     addVectors(firstPoint, scaleVector(startHeading, 40)),
  //   );
  //   points.push(startDongle);
  // }
  const endPoints = [];
  // if (endHeading) {
  //   const endDongle = toWorldSpace(
  //     arrow,
  //     addVectors(target, scaleVector(endHeading, 40)),
  //   );
  //   endPoints.push(endDongle);
  // }
  endPoints.push(toWorldSpace(arrow, target));

  return calculateSegment(
    points,
    endPoints,
    boundingBoxes.filter((bb): bb is Bounds => bb !== null),
  ).map((point) => toLocalSpace(arrow, point));
};

const calculateSegment = (
  start: readonly Point[],
  end: Point[],
  boundingBoxes: Bounds[],
): Point[] => {
  const points: Point[] = Array.from(start);
  // Limit max step to avoid infinite loop
  for (let step = 0; step < 50; step++) {
    const next = kernel(points, end, boundingBoxes);
    if (arePointsEqual(end[0], next)) {
      break;
    }

    points.push(next);
  }

  return points.concat(end);
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

const getCenter = (box: Bounds): Point => {
  const [minX, minY, maxX, maxY] = box;

  return [(minX + maxX) / 2, (minY + maxY) / 2];
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

const scaleVector = (vector: Vector, scalar: number): Vector => [
  vector[0] * scalar,
  vector[1] * scalar,
];

const addVectors = (vec1: Vector, vec2: Vector): Vector => [
  vec1[0] + vec2[0],
  vec1[1] + vec1[1],
];

const cutoff = (num: number): number =>
  Math.round(num * 1000000000) / 1000000000;

const vectorToHeading = (vec: Vector): Vector => {
  const x = vec[0];
  const y = vec[1];
  const absX = Math.abs(x);
  const absY = Math.abs(y);
  if (x > absY) {
    return [1, 0];
  } else if (x <= -absY) {
    return [-1, 0];
  } else if (y > absX) {
    return [0, 1];
  }
  return [0, -1];
};

const getStartEndBounds = (
  arrow: ExcalidrawArrowElement,
): [Bounds | null, Bounds | null] => {
  const scene = Scene.getScene(arrow);
  if (!scene) {
    return [null, null];
  }

  const elementsMap = scene.getNonDeletedElementsMap();
  const startEndElements = [
    arrow.startBinding
      ? elementsMap.get(arrow.startBinding.elementId) ?? null
      : null,
    arrow.endBinding
      ? elementsMap.get(arrow.endBinding.elementId) ?? null
      : null,
  ];

  return startEndElements.map(
    (el) => el && getElementBounds(el, elementsMap),
  ) as [Bounds | null, Bounds | null];
};

const getAvoidanceBounds = (el: ExcalidrawArrowElement): (Bounds | null)[] => {
  const scene = Scene.getScene(el);
  if (!scene) {
    return [null, null];
  }

  const elementsMap = scene.getNonDeletedElementsMap();
  const bindings = scene
    .getNonDeletedElements()
    .filter(
      (element) =>
        element.id === el.startBinding?.elementId ||
        element.id === el.endBinding?.elementId,
    );
  bindings.sort((a, b) =>
    a.id === el.startBinding?.elementId
      ? -1
      : b.id === el.endBinding?.elementId
      ? 1
      : -1,
  );

  return bindings.map((element) =>
    element ? getElementBounds(element, elementsMap) : null,
  );
};

// enum Quadrant {
//   NW, // Top Left
//   NE, // Top Right
//   SW, // Bottom Left
//   SE, // Bottom Right
// }

// const vectorToQuadrant = (vec: Vector): Quadrant => {
//   if (vec[0] > 0) {
//     if (vec[1] > 0) {
//       return Quadrant.SE;
//     }
//     return Quadrant.NE;
//   }
//   if (vec[1] > 0) {
//     return Quadrant.SW;
//   }
//   return Quadrant.NW;
// };

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
