import {
  addVectors,
  arePointsEqual,
  crossProduct,
  distanceSq,
  dotProduct,
  normalize,
  pointToVector,
  rotatePoint,
  rotateVector,
  scaleVector,
  subtractVectors,
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
import {
  debugDrawClear,
  debugDrawNormal,
  debugDrawPoint,
  debugDrawSegments,
} from "./debug";

export const calculateElbowArrowJointPoints = (
  arrow: ExcalidrawArrowElement,
): readonly LocalPoint[] => {
  if (arrow.points.length < 2) {
    // Arrow being created
    return arrow.points;
  }

  debugDrawClear();

  const target = toWorldSpace(arrow, arrow.points[arrow.points.length - 1]);
  const firstPoint = toWorldSpace(arrow, arrow.points[0]);

  const [startHeading, endHeading] = getNormalVectorsForStartEndElements(
    arrow,
    firstPoint,
    target,
  );

  const points = [firstPoint];
  if (startHeading) {
    const startDongle = addVectors(firstPoint, scaleVector(startHeading, 40));
    points.push(startDongle);
    //debugDrawNormal(startNormal, startClosestSegment);
  }

  const endPoints = [];
  if (endHeading) {
    const endDongle = addVectors(target, scaleVector(endHeading, 40));
    endPoints.push(endDongle);
    //debugDrawNormal(endNormal, endClosestSegment);
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
    //const next = avoidanceKernel(points, end, boundingBoxes);
    if (arePointsEqual(end[0], next)) {
      break;
    }

    points.push(next);
  }

  return points.concat(end);
};

const avoidanceKernel = (
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
  const startNormal = rotateVector(startVector, Math.PI / 2);
  const startEndDot = dotProduct(startVector, endVector);
  const startNormalEndDot = dotProduct(startNormal, endVector);
  const rightStartNormalDot = dotProduct([1, 0], startNormal);

  console.log(intersectionDistance(start, end, boundingBoxes));
  return end;

  // 1.a If start and end are colinear and pointing
  //     to the same direction, just jump to the end.
  if (startEndDot === 1) {
    const touchPoint = intersectionDistance(start, end, boundingBoxes);
  }
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

const getNormalVectorsForStartEndElements = (
  arrow: ExcalidrawArrowElement,
  startPoint: Point,
  endPoint: Point,
) => {
  const [startSegments, endSegments] = getStartEndLineSegments(arrow);

  const startVectors = startSegments
    ?.map(segmentMidpoint)
    .map((midpoint) => normalize(pointToVector(startPoint, midpoint)));
  const endVectors = endSegments
    ?.map(segmentMidpoint)
    .map((midpoint) => normalize(pointToVector(startPoint, midpoint)));

  const startNormals = startSegments
    ?.map((segment) => {
      debugDrawNormal(getNormalVectorForSegment(segment), segment);

      return segment;
    })
    .map(getNormalVectorForSegment);
  const endNormals = endSegments
    ?.map((segment) => {
      debugDrawNormal(getNormalVectorForSegment(segment), segment);

      return segment;
    })
    .map(getNormalVectorForSegment);

  const startDots =
    startVectors &&
    startNormals &&
    merge(startVectors, startNormals).map(([o, n]) => dotProduct(n, o));
  const endDots =
    endVectors &&
    endNormals &&
    merge(endVectors, endNormals).map(([o, n]) => dotProduct(n, o));

  const startClosestNormal =
    startDots &&
    startNormals &&
    merge(startDots, startNormals).reduce(
      (selection, [dot, normal]: [number, Vector]) =>
        dot >= 0 && dot > selection[0] ? [dot, normal] : selection,
    )[1];
  const endClosestNormal =
    endDots &&
    endNormals &&
    merge(endDots, endNormals).reduce(
      (selection, [dot, normal]: [number, Vector]) =>
        dot >= 0 && dot > selection[0] ? [dot, normal] : selection,
    )[1];

  return [
    startClosestNormal && vectorToHeading(startClosestNormal),
    endClosestNormal && vectorToHeading(endClosestNormal),
  ];
};

const segmentMidpoint = (segment: Segment): Point => [
  (segment[0][0] + segment[1][0]) / 2,
  (segment[0][1] + segment[1][1]) / 2,
];

const merge = <A, B>(a: A[], b: B[]) => {
  let _a;
  let _b;
  const result = [];
  for (let i = 0; i < Math.max(a.length, b.length); i++) {
    _a = a[i] ?? _a;
    _b = b[i] ?? _b;
    result.push([_a, _b]);
  }

  return result as [A, B][];
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

const getNormalVectorForSegment = (segment: [Point, Point]): Vector =>
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

const intersectionDistance = (
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
      return Math.sqrt(intersectionDistanceLeft(origin, target, boundingBoxes));
    case target[0] >= origin[0]:
      return Math.sqrt(
        intersectionDistanceRight(origin, target, boundingBoxes),
      );
    case target[1] < origin[1]:
      return Math.sqrt(intersectionDistanceTop(origin, target, boundingBoxes));
    case target[1] >= origin[1]:
      return Math.sqrt(
        intersectionDistanceBottom(origin, target, boundingBoxes),
      );
    default:
      return Infinity;
  }
};

// Check right sides of bounding boxes against a left pointing ray
const intersectionDistanceLeft = (
  origin: Point,
  target: Point,
  boundingBoxes: Bounds[],
) => {
  return boundingBoxes
    .map(
      (box) =>
        directedSegmentsIntersectionPointWithObtuseAngle(
          [origin, target],
          [
            [box[2], box[1]],
            [box[2], box[3]],
          ],
        ) ?? ([Infinity, Infinity] as Point),
    )
    .map((p) => {
      debugDrawPoint(p);
      return p;
    })
    .reduce((acc, value) => {
      const dist = distanceSq(origin, value);
      return dist < acc ? dist : acc;
    }, Infinity);
};
// Check left sides of bounding boxes against a right pointing ray
const intersectionDistanceRight = (
  origin: Point,
  target: Point,
  boundingBoxes: Bounds[],
) =>
  boundingBoxes
    .map(
      (box) =>
        directedSegmentsIntersectionPointWithObtuseAngle(
          [origin, target],
          [
            [box[0], box[1]],
            [box[0], box[3]],
          ],
        ) ?? ([Infinity, Infinity] as Point),
    )
    .reduce((acc, value) => {
      const dist = distanceSq(origin, value);
      return dist < acc ? dist : acc;
    }, Infinity);

// Check bottom sides of bounding boxes against a top pointing ray
const intersectionDistanceTop = (
  origin: Point,
  target: Point,
  boundingBoxes: Bounds[],
) =>
  boundingBoxes
    .map(
      (box) =>
        directedSegmentsIntersectionPointWithObtuseAngle(
          [origin, target],
          [
            [box[0], box[3]],
            [box[2], box[3]],
          ],
        ) ?? ([Infinity, Infinity] as Point),
    )
    .reduce((acc, value) => {
      const dist = distanceSq(origin, value);
      return dist < acc ? dist : acc;
    }, Infinity);

// Check top sides of bounding boxes against a bottom pointing ray
const intersectionDistanceBottom = (
  origin: Point,
  target: Point,
  boundingBoxes: Bounds[],
) =>
  boundingBoxes
    .map(
      (box) =>
        directedSegmentsIntersectionPointWithObtuseAngle(
          [origin, target],
          [
            [box[0], box[1]],
            [box[2], box[1]],
          ],
        ) ?? ([Infinity, Infinity] as Point),
    )
    .reduce((acc, value) => {
      const dist = distanceSq(origin, value);
      return dist < acc ? dist : acc;
    }, Infinity);

const arePointsEpsilonClose = (a: Point, b: Point) =>
  a[0] - b[0] < 0.0005 && a[1] - b[1] < 0.0005;

const segmentsIntersectAt = (
  a: Readonly<Segment>,
  b: Readonly<Segment>,
): Point | null => {
  const r = subtractVectors(a[1], a[0]);
  const s = subtractVectors(b[1], b[0]);
  const denominator = crossProduct(r, s);

  if (denominator === 0) {
    return null;
  }

  const u = crossProduct(subtractVectors(a[0], b[0]), r) / denominator;
  const t = crossProduct(subtractVectors(a[0], b[0]), s) / denominator;

  if (
    arePointsEpsilonClose(
      addVectors(a[0], scaleVector(r, t)),
      addVectors(b[0], scaleVector(r, u)),
    )
  ) {
    debugDrawSegments(b, "red");
    return addVectors(a[0], scaleVector(normalize(r), t));
  }

  return null;
};

const directedSegmentsIntersectionPointWithObtuseAngle = (
  a: Readonly<Segment>,
  b: Readonly<Segment>,
): Point | null => {
  const aVector = pointToVector(a[1], a[0]);
  const bVector = pointToVector(b[1], b[0]);

  if (dotProduct(aVector, bVector) < 0) {
    return segmentsIntersectAt(a, b);
  }

  return null;
};

/*
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

 */
