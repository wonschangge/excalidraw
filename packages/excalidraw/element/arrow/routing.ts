import {
  PointInTriangle,
  addVectors,
  arePointsEqual,
  areSegmentsColinear,
  distanceSq,
  dotProduct,
  isPointInsideBoundingBox,
  normalize,
  pointToVector,
  rotatePoint,
  rotateVector,
  scaleUp,
  scaleVector,
  segmentsIntersectAt,
  toLocalSpace,
  toWorldSpace,
} from "../../math";
import Scene from "../../scene/Scene";
import { LocalPoint, Point, Segment, Vector } from "../../types";
import { BoundingBox, Bounds } from "../bounds";
import {
  ExcalidrawArrowElement,
  ExcalidrawElement,
  NonDeletedSceneElementsMap,
} from "../types";
import {
  debugDrawBounds,
  debugDrawClear,
  debugDrawPoint,
  debugDrawSegments,
} from "./debug";

const STEP_COUNT_LIMIT = 50;
const MIN_SELF_BOX_OFFSET = 10;

type Heading = [1, 0] | [-1, 0] | [0, 1] | [0, -1];
const UP = [0, -1] as Heading;
const RIGHT = [1, 0] as Heading;
const DOWN = [0, 1] as Heading;
const LEFT = [-1, 0] as Heading;

export const calculateElbowArrowJointPoints = (
  arrow: ExcalidrawArrowElement,
): readonly LocalPoint[] => {
  if (arrow.points.length < 2) {
    // Arrow being created
    return arrow.points;
  }

  console.log("---");
  debugDrawClear();

  const target = toWorldSpace(arrow, arrow.points[arrow.points.length - 1]);
  const firstPoint = toWorldSpace(
    arrow,
    arrow.points[0], //arrow.points.length - 2
  );
  const avoidBounds = getStartEndBounds(arrow).filter(
    (bb): bb is Bounds => bb !== null,
  );

  const [startHeading, endHeading] = getHeadingForStartEndElements(
    arrow,
    firstPoint,
    target,
  );

  const points = [firstPoint];
  if (startHeading) {
    const dongle = extendSegmentToBoundingBoxEdge(
      [firstPoint, addVectors(firstPoint, startHeading)],
      true,
      avoidBounds,
    );
    points.push(dongle);
  } else {
    points.push(firstPoint);
  }

  const endPoints = [];
  if (endHeading) {
    const dongle = extendSegmentToBoundingBoxEdge(
      [addVectors(target, endHeading), target],
      false,
      avoidBounds,
    );
    endPoints.push(dongle);
  } else {
    endPoints.push(target);
  }
  endPoints.push(target);

  return calculateSegment(points, endPoints, avoidBounds).map((point) =>
    toLocalSpace(arrow, point),
  );
};

const calculateSegment = (
  start: readonly Point[],
  end: Point[],
  boundingBoxes: Bounds[],
): Point[] => {
  const points: Point[] = Array.from(start);
  // Limit max step to avoid infinite loop
  for (let step = 0; step < STEP_COUNT_LIMIT; step++) {
    const next = kernel(points, end, boundingBoxes);
    if (arePointsEqual(end[0], next)) {
      break;
    }
    points.push(next);
  }

  if (points.length > STEP_COUNT_LIMIT) {
    console.error("Elbow arrow routing step count limit reached", points);
  }

  return points.concat(end);
};

const extendSegmentToBoundingBoxEdge = (
  segment: Segment,
  segmentIsStart: boolean,
  boundingBoxes: Bounds[],
): Point => {
  const [start, end] = segment;
  const vector = pointToVector(end, start);
  const normal = rotateVector(vector, Math.PI / 2);
  const rightSegmentNormalDot = dotProduct([1, 0], normal);
  const segmentIsHorizontal = rightSegmentNormalDot === 0;
  const rightSegmentDot = dotProduct([1, 0], vector);

  const containing = boundingBoxes.filter((bBox) =>
    isPointInsideBoundingBox(segmentIsStart ? start : end, bBox),
  );

  // TODO: If this is > 1 it means the arrow is in an overlapping shape
  if (containing.length > 0) {
    const minDist = containing
      .map((bbox) =>
        segmentIsHorizontal ? bbox[2] - bbox[0] : bbox[3] - bbox[1],
      )
      .reduce((largest, value) => (value > largest ? value : largest), 0);

    // TODO: Simplify this by having the segment for end provided backwards
    const candidate: Segment = segmentIsStart
      ? segmentIsHorizontal
        ? [
            start,
            addVectors(start, [rightSegmentDot > 0 ? minDist : -minDist, 0]),
          ]
        : [
            start,
            addVectors(start, [
              0,
              rightSegmentNormalDot > 0 ? -minDist : minDist,
            ]),
          ]
      : segmentIsHorizontal
      ? [end, addVectors(end, [rightSegmentDot > 0 ? -minDist : minDist, 0])]
      : [
          end,
          addVectors(end, [0, rightSegmentNormalDot > 0 ? minDist : -minDist]),
        ];

    const intersection = containing
      .map(bboxToClockwiseWoundingSegments) // TODO: This could be calcualted once in createRoute
      .flatMap((segments) =>
        segments!.map((segment) => segmentsIntersectAt(candidate, segment)),
      )
      .filter((x) => x !== null)[0]!;

    return addVectors(
      intersection,
      scaleVector(
        segmentIsStart
          ? normalize(vector)
          : normalize(pointToVector(start, end)),
        10,
      ),
    );
  }

  return segmentIsStart ? segment[0] : segment[1];
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
      ? ([0, 0] as Vector) // TODO: Fixed right start attachment
      : normalize(pointToVector(start, points[points.length - 2]));
  const endVector =
    target.length < 2
      ? ([0, 0] as Vector) // TODO: Fixed left end attachment
      : normalize(pointToVector(target[1], end));
  const startNormal = rotateVector(startVector, Math.PI / 2);
  const rightStartNormalDot = dotProduct([1, 0], startNormal);
  const startEndVector = pointToVector(end, start);
  const endAhead = dotProduct(startVector, startEndVector) > 0;

  let next: Point =
    rightStartNormalDot === 0 // Last segment from start is horizontal
      ? endAhead
        ? [end[0], start[1]]
        : [start[0], end[1]] // Turn up/down all the way to end
      : endAhead
      ? [start[0], end[1]]
      : [end[0], start[1]]; // Turn left/right all the way to end

  const nextEndVector = normalize(pointToVector(end, next));
  const nextEndDot = dotProduct(nextEndVector, endVector);
  const alignedButNotRightThere =
    (end[0] - next[0] === 0) !== (end[1] - next[1] === 0);

  if (nextEndDot === -1 && alignedButNotRightThere) {
    next =
      rightStartNormalDot === 0
        ? [start[0], end[1] + 40]
        : [end[0] + 40, start[1]];
  }

  next = resolveIntersections(points, next, boundingBoxes);

  return next;
};

const getHitOffset = (start: Point, next: Point, boundingBoxes: Bounds[]) => {
  return (
    boundingBoxes
      .map(bboxToClockwiseWoundingSegments)
      .flatMap((segments) =>
        segments!.map((segment) => {
          const p = segmentsIntersectAt([start, next], segment);

          if (p) {
            // We can use the p -> segment[1] because all bbox segments are in winding order
            debugDrawSegments(segment, "red");
            return [
              distanceSq(start, p),
              Math.sqrt(distanceSq(segment[0], p)),
              Math.sqrt(distanceSq(segment[1], p)),
            ];
          }

          return [Infinity, 0, 0];
        }),
      )
      .filter((x) => x != null)
      .reduce(
        (acc, value) => (value![0] < acc![0] ? value : acc),
        [Infinity, 0, 0],
      )
      ?.slice(1) ?? [0, 0]
  );
};

const resolveIntersections = (
  points: Point[],
  next: Point,
  boundingBoxes: Bounds[],
): Point => {
  debugDrawClear();
  debugDrawPoint(next);
  const start = points[points.length - 1];
  const [offsetLeft, offsetRight] = getHitOffset(start, next, boundingBoxes);

  if (offsetLeft - offsetRight !== 0) {
    const altNextLeftDirection = rotateVector(
      normalize(pointToVector(next, start)),
      -Math.PI / 2,
    );
    const nextLeftCandidate = addVectors(
      start,
      scaleVector(altNextLeftDirection, offsetRight + 10),
    );
    const altNextRightDirection = rotateVector(
      normalize(pointToVector(next, start)),
      Math.PI / 2,
    );
    const nextRightCandidate = addVectors(
      start,
      scaleVector(altNextRightDirection, offsetLeft + 10),
    );
    const nextLeft =
      getHitOffset(start, nextLeftCandidate, boundingBoxes)[0] > 0 ||
      segmentsOverlap(
        [points[points.length - 1], points[points.length - 2]],
        [points[points.length - 1], nextLeftCandidate],
      )
        ? null
        : nextLeftCandidate;
    const nextRight =
      getHitOffset(start, nextRightCandidate, boundingBoxes)[0] > 0 ||
      segmentsOverlap(
        [points[points.length - 1], points[points.length - 2]],
        [points[points.length - 1], nextRightCandidate],
      )
        ? null
        : nextRightCandidate;

    nextLeft && debugDrawPoint(nextLeft, "red");
    nextRight && debugDrawPoint(nextRight, "green");

    if (offsetLeft > offsetRight) {
      //console.log("LEFT", nextLeft, nextRight);
      return nextLeft ? nextLeft : nextRight ? nextRight : next;
    }
    //console.log("RIGHT", nextLeft, nextRight);
    return nextRight ? nextRight : nextLeft ? nextLeft : next;
  }
  debugDrawPoint(next, "blue");
  return next;
};

const rint = (p0: number, p1: number, q0: number, q1: number) => {
  const p = p0 < p1 ? [p0, p1] : [p1, p0];
  const q = q0 < q1 ? [q0, q1] : [q1, q0];

  if (p[1] - q[0] >= 0 && q[1] - p[0] >= 0) {
    const overlap = [Math.max(p[0], q[0]), Math.min(p[1], q[1])];

    return overlap[1] - overlap[0] > 0;
  }
  return false;
};

const segmentsOverlap = (a: Segment, b: Segment) => {
  const x = rint(a[0][0], a[1][0], b[0][0], b[1][0]);
  const y = rint(a[0][1], a[1][1], b[0][1], b[1][1]);

  return areSegmentsColinear(a, b) && (x || y);
};

const getHeadingForStartEndElements = (
  arrow: ExcalidrawArrowElement,
  startPoint: Point,
  endPoint: Point,
): [Vector | null, Vector | null] => {
  const startBoundElement =
    arrow.startBinding && getElementForId(arrow, arrow.startBinding.elementId);
  const endBoundElement =
    arrow.endBinding && getElementForId(arrow, arrow.endBinding.elementId);

  return [
    startBoundElement &&
      getHeadingForWorldPointFromElement(startBoundElement, startPoint),
    endBoundElement &&
      getHeadingForWorldPointFromElement(endBoundElement, endPoint),
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
    (el) => el && extendedBoundingBoxForElement(el, MIN_SELF_BOX_OFFSET),
  ) as [Bounds | null, Bounds | null];
};

const extendedBoundingBoxForElement = (
  element: ExcalidrawElement,
  offset: number = 0,
) => {
  const bbox = {
    minX: element.x,
    minY: element.y,
    maxX: element.x + element.width,
    maxY: element.y + element.height,
    midX: element.x + element.width / 2,
    midY: element.y + element.height / 2,
  } as BoundingBox;

  const center = [bbox.midX, bbox.midY] as Point;
  const [topLeftX, topLeftY] = rotatePoint(
    [bbox.minX, bbox.minY],
    center,
    element.angle,
  );
  const [topRightX, topRightY] = rotatePoint(
    [bbox.maxX, bbox.minY],
    center,
    element.angle,
  );
  const [bottomRightX, bottomRightY] = rotatePoint(
    [bbox.maxX, bbox.maxY],
    center,
    element.angle,
  );
  const [bottomLeftX, bottomLeftY] = rotatePoint(
    [bbox.minX, bbox.maxY],
    center,
    element.angle,
  );

  const extendedBounds = [
    Math.min(topLeftX, topRightX, bottomRightX, bottomLeftX) - offset,
    Math.min(topLeftY, topRightY, bottomRightY, bottomLeftY) - offset,
    Math.max(topLeftX, topRightX, bottomRightX, bottomLeftX) + offset,
    Math.max(topLeftY, topRightY, bottomRightY, bottomLeftY) + offset,
  ] as Bounds;

  debugDrawBounds(extendedBounds);

  return extendedBounds;
};

// Gets the heading for the point by creating a bounding box around the rotated
// close fitting bounding box, then creating 4 search cones around the center of
// the external bbox.
const getHeadingForWorldPointFromElement = (
  element: ExcalidrawElement,
  point: Point,
): Heading => {
  const SEARCH_CODE_MULTIPLIER = 2;
  const bounds = extendedBoundingBoxForElement(element, MIN_SELF_BOX_OFFSET);
  const midPoint = getCenterWorldCoordsForBounds(bounds);
  const startTopLeft = scaleUp(
    [bounds[0], bounds[1]],
    midPoint,
    SEARCH_CODE_MULTIPLIER,
  );
  const startTopRight = scaleUp(
    [bounds[2], bounds[1]],
    midPoint,
    SEARCH_CODE_MULTIPLIER,
  );
  const startBottomLeft = scaleUp(
    [bounds[0], bounds[3]],
    midPoint,
    SEARCH_CODE_MULTIPLIER,
  );
  const startBottomRight = scaleUp(
    [bounds[2], bounds[3]],
    midPoint,
    SEARCH_CODE_MULTIPLIER,
  );

  return PointInTriangle(point, startTopLeft, startTopRight, midPoint)
    ? UP
    : PointInTriangle(point, startTopRight, startBottomRight, midPoint)
    ? RIGHT
    : PointInTriangle(point, startBottomRight, startBottomLeft, midPoint)
    ? DOWN
    : LEFT;
};

// Turn a bounding box into 4 clockwise wounding segments
const bboxToClockwiseWoundingSegments = (b: Bounds | null) =>
  b && [
    [[b[0], b[1]] as Point, [b[2], b[1]] as Point] as Segment,
    [[b[2], b[1]] as Point, [b[2], b[3]] as Point] as Segment,
    [[b[2], b[3]] as Point, [b[0], b[3]] as Point] as Segment,
    [[b[0], b[3]] as Point, [b[0], b[1]] as Point] as Segment,
  ];

const getElementForId = (
  arrow: ExcalidrawArrowElement,
  id: string,
): ExcalidrawElement | null => {
  const elementsMap = getElementsMap(arrow);
  return elementsMap?.get(id) ?? null;
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

const getCenterWorldCoordsForBounds = (bounds: Bounds): Point => [
  bounds[0] + (bounds[2] - bounds[0]) / 2,
  bounds[1] + (bounds[3] - bounds[1]) / 2,
];
