import {
  arePointsEqual,
  dot,
  normalize,
  pointToVector,
  rotatePoint,
} from "../../math";
import { LocalPoint, Point, Vector } from "../../types";
import {
  ElementsMap,
  ExcalidrawArrowElement,
  ExcalidrawBindableElement,
  ExcalidrawElement,
  NonDeletedSceneElementsMap,
} from "../types";
import { Bounds, getElementAbsoluteCoords, getElementBounds } from "../bounds";
import Scene from "../../scene/Scene";

type Segment = [Point, Point];

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

  window.v.lines = [];

  const target = toWorldSpace(arrow, arrow.points[arrow.points.length - 1]);
  const firstPoint = toWorldSpace(arrow, arrow.points[0]);

  const boundingBoxes = getStartEndBounds(arrow);
  const [startClosestSegment, endClosestSegment] =
    getClosestStartEndLineSegments(arrow, firstPoint, target);

  const startNormal =
    startClosestSegment &&
    getNormalVectorForSegment(startClosestSegment, firstPoint);
  const endNormal =
    endClosestSegment && getNormalVectorForSegment(endClosestSegment, target);

  const startHeading = startNormal && vectorToHeading(startNormal);
  const endHeading = endNormal && vectorToHeading(endNormal);

  const points = [firstPoint];
  if (startHeading) {
    const startDongle = addVectors(firstPoint, scaleVector(startHeading, 40));
    points.push(startDongle);

    if (startNormal) {
      const [cx, cy] = getCenter(boundingBoxes[0]!);
      const [nx, ny] = scaleVector(normalize(startNormal), 60);
      window.v.lines.push([[cx, cy], [cx + nx, cy + ny], "cyan"]);
    }
  }

  const endPoints = [];
  if (endHeading) {
    const endDongle = addVectors(target, scaleVector(endHeading, 40));
    endPoints.push(endDongle);

    if (endNormal) {
      const [cx, cy] = getCenter(boundingBoxes[1]!);
      const [nx, ny] = scaleVector(normalize(endNormal), 60);
      window.v.lines.push([[cx, cy], [cx + nx, cy + ny], "cyan"]);
    }
  }
  endPoints.push(target);

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

const toLocalSpace = (arrow: ExcalidrawElement, p: Point): LocalPoint => [
  p[0] - arrow.x,
  p[1] - arrow.y,
];

const toWorldSpace = (element: ExcalidrawElement, p: LocalPoint): Point => [
  p[0] + element.x,
  p[1] + element.y,
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
  vec1[1] + vec2[1],
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

  startLineSegments?.forEach((segment) =>
    window.v.lines.push([segment[0], segment[1], "green"]),
  );
  endLineSegments?.forEach((segment) =>
    window.v.lines.push([segment[0], segment[1], "green"]),
  );

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

  const distances = segments.map((segment, idx) => {
    const distance = distanceOfPointFromSegment(p, segment);

    return { distance, idx };
  });

  distances.sort((a, b) => a.distance - b.distance);

  return segments[distances[0].idx];
};

const distanceOfPointFromSegment = (p: Point, segment: Segment): number => {
  const [p1, p2] = segment;

  // NOTE: sqrt is expensive, so if the exact distance is not needed
  // using the square of the distance can be more efficient
  const pointToPointDistanceSquared = (a: Point, b: Point) => {
    const i = a[0] - b[0];
    const j = a[1] - b[1];

    return i * i + j * j;
  };

  const segmentLengthSquared = pointToPointDistanceSquared(p1, p2);
  if (segmentLengthSquared === 0) {
    return pointToPointDistanceSquared(p, p1);
  }

  const t = clamp(
    ((p[0] - p1[0]) * (p2[0] - p1[0]) + (p[1] - p1[1]) * (p2[1] - p1[1])) /
      segmentLengthSquared,
    0,
    1,
  );
  return pointToPointDistanceSquared(p, [
    p1[0] + t * (p2[0] - p1[0]),
    p1[1] + t * (p2[1] - p1[1]),
  ]);

  // const numerator = Math.abs((x2 - x1) * (y0 - y1) - (x0 - x1) * (y2 - y1));
  // const denominator = Math.sqrt(Math.pow(x2 - x1, 2) + Math.pow(y2 - y1, 2));

  // return numerator / denominator;
};

const clamp = (value: number, min: number, max: number) =>
  Math.max(min, Math.min(max, value));

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

  startClosestLineSegment &&
    window.v.lines.push([
      startClosestLineSegment[0],
      startClosestLineSegment[1],
      "red",
    ]);
  endClosestLineSegment &&
    window.v.lines.push([
      endClosestLineSegment[0],
      endClosestLineSegment[1],
      "red",
    ]);

  return [startClosestLineSegment, endClosestLineSegment];
};

// Arrow end/start points to the center of the start/end shape = Target Vector
// Target Vector DOT Normal < 0 means the Normal POINTS OUTSIDE, because it's a convex shape
const getNormalVectorForSegment = (segment: [Point, Point], p: Point): Vector =>
  // Because of the winding order and convex shapes, the normal
  // is always PI/2 rads rotation
  rotateVector(pointToVector(segment[0], segment[1]), Math.PI / 2);

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
    case "ellipse":
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

const generatePointPairs = (points: Readonly<Point[]>) => {
  const [first, ...restOfThePoints] = points;
  let latest = first;

  return restOfThePoints.map((point) => {
    const res = [latest, point];
    latest = point;
    return res;
  }) as [Point, Point][];
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
