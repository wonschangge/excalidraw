import {
  addVectors,
  arePointsEqual,
  clamp,
  distanceSqared,
  doSegmentsIntersect,
  dotProduct,
  normalize,
  pointToVector,
  rotatePoint,
  rotateVector,
  rounding,
  scaleVector,
  toLocalSpace,
  toWorldSpace,
  vectorToHeading,
} from "../../math";
import { LocalPoint, Point, Segment, Vector } from "../../types";
import {
  ElementsMap,
  ExcalidrawArrowElement,
  ExcalidrawBindableElement,
  ExcalidrawElement,
  NonDeletedSceneElementsMap,
} from "../types";
import { Bounds, getElementAbsoluteCoords, getElementBounds } from "../bounds";
import Scene from "../../scene/Scene";
import { debugDrawClear, debugDrawNormal, debugDrawSegments } from "./debug";

// ========================================
// The main idea is to Ray March the arrow
// ========================================

export const calculatePoints = (
  arrow: ExcalidrawArrowElement,
): readonly LocalPoint[] => {
  if (arrow.points.length < 2) {
    // Arrow being created
    return arrow.points;
  }

  debugDrawClear();

  const target = toWorldSpace(arrow, arrow.points[arrow.points.length - 1]);
  const firstPoint = toWorldSpace(arrow, arrow.points[0]);

  const [startClosestSegment, endClosestSegment] =
    getClosestStartEndLineSegments(arrow, firstPoint, target);

  const startNormal =
    startClosestSegment &&
    getNormalVectorForSegment(startClosestSegment, firstPoint);
  const endNormal =
    endClosestSegment && getNormalVectorForSegment(endClosestSegment, target);

  // TODO: These headings can be memoized by the binding shape coords
  const startHeading = startNormal && vectorToHeading(startNormal);
  const endHeading = endNormal && vectorToHeading(endNormal);

  const points = [firstPoint];
  if (startHeading) {
    const startDongle = addVectors(firstPoint, scaleVector(startHeading, 40));
    points.push(startDongle);
    debugDrawNormal(startNormal, startClosestSegment);
  }

  const endPoints = [];
  if (endHeading) {
    const endDongle = addVectors(target, scaleVector(endHeading, 40));
    endPoints.push(endDongle);
    debugDrawNormal(endNormal, endClosestSegment);
  }
  endPoints.push(target);

  return calculateSegment(
    points,
    endPoints,
    getStartEndBounds(arrow).filter((bb): bb is Bounds => bb !== null),
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
  const rightStartNormalDot = dotProduct(
    [1, 0],
    rotateVector(startVector, Math.PI / 2),
  );

  const next: Point =
    rightStartNormalDot === 0
      ? [start[0], end[1]] // Last segment from start is horizontal
      : [end[0], start[1]]; // Last segment from start is vertical
  const nextVector = normalize(pointToVector(next, end));
  const nextEndDot = dotProduct(nextVector, endVector);

  if (nextEndDot === 1) {
    // Facing opposite - make a half pass toward the target
    return rightStartNormalDot === 0
      ? [start[0], start[1] + (end[1] - start[1]) / 2]
      : [start[0] + (end[0] - start[0]) / 2, start[1]];
  }

  return next;
};

const getElementsMap = (
  arrow: ExcalidrawArrowElement,
): NonDeletedSceneElementsMap | null => {
  const scene = Scene.getScene(arrow);
  if (!scene) {
    return null;
  }

  return scene.getNonDeletedElementsMap();
};

const getStartEndElements = (
  arrow: ExcalidrawArrowElement,
): [ExcalidrawBindableElement | null, ExcalidrawBindableElement | null] => {
  const elementsMap = getElementsMap(arrow);
  if (!elementsMap) {
    return [null, null];
  }

  return [
    arrow.startBinding
      ? (elementsMap.get(
          arrow.startBinding.elementId,
        ) as ExcalidrawBindableElement) ?? null
      : null,
    arrow.endBinding
      ? (elementsMap.get(
          arrow.endBinding.elementId,
        ) as ExcalidrawBindableElement) ?? null
      : null,
  ];
};

const getStartEndBounds = (
  arrow: ExcalidrawArrowElement,
): [Bounds | null, Bounds | null] => {
  const elementsMap = getElementsMap(arrow);
  if (!elementsMap) {
    return [null, null];
  }
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

const getStartEndLineSegments = (
  arrow: ExcalidrawArrowElement,
): [Segment[] | null, Segment[] | null] => {
  const elementsMap = getElementsMap(arrow);
  if (!elementsMap) {
    return [null, null];
  }

  const [startElement, endElement] = getStartEndElements(arrow);
  const startLineSegments: Segment[] | null =
    startElement && estimateShape(startElement, elementsMap);
  const endLineSegments: Segment[] | null =
    endElement && estimateShape(endElement, elementsMap);

  debugDrawSegments(startLineSegments);
  debugDrawSegments(endLineSegments);

  return [startLineSegments, endLineSegments] as [
    Segment[] | null,
    Segment[] | null,
  ];
};

const getClosestLineSegment = (
  segments: Segment[],
  p: Point,
): Segment | null => {
  if (segments.length === 0) {
    return null;
  }

  const idx = segments
    .map((segment) => distanceOfPointFromSegment(p, segment))
    .reduce(
      (idxOfSmallest, distance, idx, distances) =>
        distances[idxOfSmallest] > distance ? idx : idxOfSmallest,
      0,
    );

  return segments[idx];
};

const distanceOfPointFromSegment = (p: Point, segment: Segment): number => {
  const [p1, p2] = segment;

  const segmentLengthSquared = distanceSqared(p1, p2);
  if (segmentLengthSquared === 0) {
    return distanceSqared(p, p1);
  }

  const t = clamp(
    ((p[0] - p1[0]) * (p2[0] - p1[0]) + (p[1] - p1[1]) * (p2[1] - p1[1])) /
      segmentLengthSquared,
    0,
    1,
  );

  return rounding(
    distanceSqared(p, [
      p1[0] + t * (p2[0] - p1[0]),
      p1[1] + t * (p2[1] - p1[1]),
    ]),
    3, // Avoids jumpy arows with small shapes due to float resolution issues
  );
};

const getClosestStartEndLineSegments = (
  arrow: ExcalidrawArrowElement,
  startPoint: Point,
  endPoint: Point,
) => {
  const [startLineSegments, endLineSegments] = getStartEndLineSegments(arrow);

  const startClosestLineSegment =
    startLineSegments && getClosestLineSegment(startLineSegments, startPoint);
  const endClosestLineSegment =
    endLineSegments && getClosestLineSegment(endLineSegments, endPoint);

  debugDrawSegments(startClosestLineSegment, "red");
  debugDrawSegments(endClosestLineSegment, "red");

  return [startClosestLineSegment, endClosestLineSegment];
};

const getNormalVectorForSegment = (segment: [Point, Point], p: Point): Vector =>
  // Because of the winding order and convex shapes,
  // the normal is always PI/2 rads rotation
  normalize(rotateVector(pointToVector(segment[0], segment[1]), Math.PI / 2));

const estimateShape = (
  element: ExcalidrawElement,
  elementsMap: ElementsMap,
): Segment[] => {
  const [x1, y1, x2, y2, cx, cy] = getElementAbsoluteCoords(
    element,
    elementsMap,
  );

  switch (element.type) {
    case "rectangle":
    case "iframe":
    case "embeddable":
    case "image":
    case "ellipse":
      return [
        [
          rotatePoint([x1, y1], [cx, cy], element.angle),
          rotatePoint([x2, y1], [cx, cy], element.angle),
        ],
        [
          rotatePoint([x2, y1], [cx, cy], element.angle),
          rotatePoint([x2, y2], [cx, cy], element.angle),
        ],
        [
          rotatePoint([x2, y2], [cx, cy], element.angle),
          rotatePoint([x1, y2], [cx, cy], element.angle),
        ],
        [
          rotatePoint([x1, y2], [cx, cy], element.angle),
          rotatePoint([x1, y1], [cx, cy], element.angle),
        ],
      ];
    case "diamond":
      const N = rotatePoint(
        [x1 + (x2 - x1) / 2, y1],
        [cx, cy],
        element.angle,
      ) as Point;
      const W = rotatePoint(
        [x1, y1 + (y2 - y1) / 2],
        [cx, cy],
        element.angle,
      ) as Point;
      const E = rotatePoint(
        [x2, y1 + (y2 - y1) / 2],
        [cx, cy],
        element.angle,
      ) as Point;
      const S = rotatePoint(
        [x1 + (x2 - x1) / 2, y2],
        [cx, cy],
        element.angle,
      ) as Point;
      const segments = [
        [W, N] as Segment,
        [N, E] as Segment,
        [E, S] as Segment,
        [S, W] as Segment,
      ];

      return segments;
    default:
      console.error(`Not supported shape: ${element.type}`);
      return [];
  }
};

/*
const generatePointPairs = (points: Readonly<Point[]>) => {
  const [first, ...restOfThePoints] = points;
  let latest = first;

  return restOfThePoints.map((point) => {
    const res = [latest, point];
    latest = point;
    return res;
  }) as [Point, Point][];
};
*/

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


*/

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

  switch (true) {
    case target[0] < origin[0]:
      return naiveRaycastLeft(origin, target, boundingBoxes);
    case target[0] >= origin[0]:
      return naiveRaycastRight(origin, target, boundingBoxes);
    case target[1] < origin[1]:
      return naiveRaycastTop(origin, target, boundingBoxes);
    case target[1] >= origin[1]:
      return naiveRaycastBottom(origin, target, boundingBoxes);
    default:
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
