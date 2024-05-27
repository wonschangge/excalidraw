import {
  PointInTriangle,
  addVectors,
  arePointsEqual,
  distanceSq,
  doBoundsIntersect,
  dotProduct,
  isPointInsideBoundingBox,
  normalize,
  pointToVector,
  rotatePoint,
  rotateVector,
  scaleUp,
  scaleVector,
  segmentsIntersectAt,
  segmentsOverlap,
  toLocalSpace,
  toWorldSpace,
  vectorToHeading,
} from "../../math";
import Scene from "../../scene/Scene";
import type { LocalPoint, Point, Segment, Vector } from "../../types";
import { getHoveredElementForBinding } from "../binding";
import type { BoundingBox, Bounds } from "../bounds";
import type { ExcalidrawArrowElement, ExcalidrawElement } from "../types";
import {
  debugClear,
  debugDrawBounds,
  debugDrawPoint,
  debugDrawSegments,
  debugNewFrame,
} from "./debug";

const STEP_COUNT_LIMIT = 50;
const MIN_SELF_BOX_OFFSET = 30;
const MIN_DONGLE_SIZE = 30;
const DISAMBIGUATION_DISTANCE = 10;

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

  debugClear();

  const target = toWorldSpace(arrow, arrow.points[arrow.points.length - 1]);
  const firstPoint = toWorldSpace(arrow, arrow.points[0]);
  const [startBounds, endBounds] = getStartEndBounds(arrow, firstPoint, target);
  const [startHeading, endHeading] = getHeadingForStartEndElements(
    arrow,
    firstPoint,
    target,
    startBounds,
    endBounds,
  );

  let avoidBounds = [startBounds, endBounds].filter(
    (bb): bb is Bounds => bb !== null,
  );

  const points = [
    firstPoint,
    startHeading
      ? extendSegmentToBoundingBoxEdge(
          [firstPoint, addVectors(firstPoint, startHeading)],
          true,
          startBounds !== null ? [startBounds] : [],
        )
      : addVectors(
          firstPoint,
          scaleVector(
            vectorToHeading(pointToVector(target, firstPoint)),
            MIN_DONGLE_SIZE,
          ),
        ),
  ];
  const endPoints = [
    endHeading
      ? extendSegmentToBoundingBoxEdge(
          [addVectors(target, endHeading), target],
          false,
          endBounds !== null ? [endBounds] : [],
        )
      : addVectors(
          target,
          scaleVector(
            vectorToHeading(pointToVector(firstPoint, target)),
            MIN_DONGLE_SIZE,
          ),
        ),
    target,
  ];

  avoidBounds = avoidBounds.filter(
    (bbox) =>
      !(
        isPointInsideBoundingBox(points[1], bbox) ||
        isPointInsideBoundingBox(endPoints[0], bbox)
      ),
  );

  // return simplifyElbowArrowPoints(
  //   calculateSegment(points, endPoints, avoidBounds).map((point) =>
  //     toLocalSpace(arrow, point),
  //   ),
  // );
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

const kernel = (
  points: Point[],
  target: Point[],
  boundingBoxes: Bounds[],
): Point => {
  debugNewFrame();

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

  const startNextVector = normalize(pointToVector(next, start));
  if (dotProduct(startNextVector, startVector) === 1) {
    next = rightStartNormalDot === 0 ? [start[0], end[1]] : [end[0], start[1]];
  }

  const nextEndVector = normalize(pointToVector(end, next));
  const nextEndDot = dotProduct(nextEndVector, endVector);
  const alignedButNotRightThere =
    Math.abs(vectorToHeading(nextEndVector)[0]) === 1
      ? end[0] - next[0] !== 0 && end[1] - next[1] === 0
      : end[0] - next[0] === 0 && end[1] - next[1] !== 0;
  debugDrawPoint(next, "red");
  if (nextEndDot === -1 && alignedButNotRightThere) {
    next = addVectors(start, scaleVector(pointToVector(next, start), 0.5));
  }

  if (boundingBoxes.length > 0) {
    const newStartNextVector = normalize(pointToVector(next, start));
    next = resolveIntersections(
      points,
      next,
      boundingBoxes,
      end,
      dotProduct(endVector, newStartNextVector) === -1,
    );
  }
  debugDrawPoint(next);

  return next;
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

  // TODO: If this is > 1 it means the arrow is in an overlapping shape
  if (boundingBoxes.length > 0) {
    const minDist = boundingBoxes
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

    const intersection = boundingBoxes
      .map(bboxToClockwiseWoundingSegments) // TODO: This could be calcualted once in createRoute
      .flatMap((segments) =>
        segments!.map((segment) => segmentsIntersectAt(candidate, segment)),
      )
      .filter((x) => x !== null)[0]!;

    return intersection
      ? addVectors(
          intersection,
          scaleVector(
            segmentIsStart
              ? normalize(vector)
              : normalize(pointToVector(start, end)),
            1, // TODO figure out scaling
          ),
        )
      : segmentIsStart
      ? segment[0]
      : segment[1];
  }

  return segmentIsStart ? segment[0] : segment[1];
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
              Math.sqrt(distanceSq(start, p)),
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
      ) ?? [0, 0, 0]
  );
};

const resolveIntersections = (
  points: Point[],
  next: Point,
  boundingBoxes: Bounds[],
  target: Point,
  targetNextFacing: boolean,
): Point => {
  const start = points[points.length - 1];
  const [offsetAhead, offsetLeft, offsetRight] = getHitOffset(
    start,
    next,
    boundingBoxes,
  );

  if (
    !targetNextFacing &&
    offsetAhead < Infinity &&
    offsetAhead > 10 && // TODO: This is a stand-in for still checking the start bound, but the start bound only
    boundingBoxes[0] &&
    boundingBoxes[1] &&
    !doBoundsIntersect(boundingBoxes[0], boundingBoxes[1])
  ) {
    console.log("dsfsdfds");
    const heading = vectorToHeading(pointToVector(next, start));
    return addVectors(start, scaleVector(heading, offsetAhead - 1));
  }

  if (offsetLeft > 0 || offsetRight > 0) {
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
      getHitOffset(start, nextLeftCandidate, boundingBoxes)[1] > 0 ||
      segmentsOverlap(
        [points[points.length - 1], points[points.length - 2]],
        [points[points.length - 1], nextLeftCandidate],
      )
        ? null
        : nextLeftCandidate;
    const nextRight =
      getHitOffset(start, nextRightCandidate, boundingBoxes)[1] > 0 ||
      segmentsOverlap(
        [points[points.length - 1], points[points.length - 2]],
        [points[points.length - 1], nextRightCandidate],
      )
        ? null
        : nextRightCandidate;

    const nextLeftEndDistance = nextLeft
      ? distanceSq(nextLeft, target)
      : Infinity;
    const nextRightEndDistance = nextRight
      ? distanceSq(nextRight, target)
      : Infinity;

    if (nextLeftEndDistance < nextRightEndDistance) {
      return nextLeft ? nextLeft : nextRight ? nextRight : next;
    }

    return nextRight ? nextRight : nextLeft ? nextLeft : next;
  }

  return next;
};

const getStartEndBounds = (
  arrow: ExcalidrawArrowElement,
  startPoint: Point,
  endPoint: Point,
): [Bounds | null, Bounds | null] => {
  const scene = Scene.getScene(arrow);
  if (!scene) {
    return [null, null];
  }

  const elementsMap = scene.getNonDeletedElementsMap();

  const startEndElements = [
    arrow.startBinding
      ? elementsMap.get(arrow.startBinding.elementId) ?? null
      : getHoveredElementForBinding(
          { x: startPoint[0], y: startPoint[1] },
          scene.getNonDeletedElements(),
          elementsMap,
        ),
    arrow.endBinding
      ? elementsMap.get(arrow.endBinding.elementId) ?? null
      : getHoveredElementForBinding(
          { x: endPoint[0], y: endPoint[1] },
          scene.getNonDeletedElements(),
          elementsMap,
        ),
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

const getHeadingForStartEndElements = (
  arrow: ExcalidrawArrowElement,
  startPoint: Point,
  endPoint: Point,
  startBounds: Bounds | null,
  endBounds: Bounds | null,
): [Vector | null, Vector | null] => {
  const scene = Scene.getScene(arrow);
  if (scene) {
    const elements = scene.getNonDeletedElements();
    const elementsMap = scene.getNonDeletedElementsMap();
    const start =
      arrow.startBinding === null
        ? getHoveredElementForBinding(
            { x: startPoint[0], y: startPoint[1] },
            elements,
            elementsMap,
          )
        : elementsMap.get(arrow.startBinding.elementId) ?? null;
    const end =
      arrow.endBinding === null
        ? getHoveredElementForBinding(
            { x: endPoint[0], y: endPoint[1] },
            elements,
            elementsMap,
          )
        : elementsMap.get(arrow.endBinding.elementId) ?? null;

    return [
      start && getHeadingForWorldPointFromElement(start, startPoint),
      end && getHeadingForWorldPointFromElement(end, endPoint),
    ];
  }

  return [null, null];
};

// Gets the heading for the point by creating a bounding box around the rotated
// close fitting bounding box, then creating 4 search cones around the center of
// the external bbox.
const getHeadingForWorldPointFromElement = (
  element: ExcalidrawElement,
  point: Point,
): Heading => {
  const SEARCH_CONE_MULTIPLIER = 2;
  const bounds = extendedBoundingBoxForElement(element, MIN_SELF_BOX_OFFSET);
  const midPoint = getCenterWorldCoordsForBounds(bounds);
  const ROTATION = element.type === "diamond" ? Math.PI / 4 : 0;

  const topLeft = rotatePoint(
    scaleUp([bounds[0], bounds[1]], midPoint, SEARCH_CONE_MULTIPLIER),
    midPoint,
    ROTATION,
  );
  const topRight = rotatePoint(
    scaleUp([bounds[2], bounds[1]], midPoint, SEARCH_CONE_MULTIPLIER),
    midPoint,
    ROTATION,
  );
  const bottomLeft = rotatePoint(
    scaleUp([bounds[0], bounds[3]], midPoint, SEARCH_CONE_MULTIPLIER),
    midPoint,
    ROTATION,
  );
  const bottomRight = rotatePoint(
    scaleUp([bounds[2], bounds[3]], midPoint, SEARCH_CONE_MULTIPLIER),
    midPoint,
    ROTATION,
  );

  // debugDrawSegments(
  //   [
  //     [topLeft, topRight],
  //     [topRight, midPoint],
  //     [midPoint, topLeft],
  //   ],
  //   "red",
  // );

  if (element.type === "diamond") {
    return PointInTriangle(point, topLeft, topRight, midPoint)
      ? RIGHT
      : PointInTriangle(point, topRight, bottomRight, midPoint)
      ? RIGHT
      : PointInTriangle(point, bottomRight, bottomLeft, midPoint)
      ? LEFT
      : LEFT;
  }

  return PointInTriangle(point, topLeft, topRight, midPoint)
    ? UP
    : PointInTriangle(point, topRight, bottomRight, midPoint)
    ? RIGHT
    : PointInTriangle(point, bottomRight, bottomLeft, midPoint)
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

const getCenterWorldCoordsForBounds = (bounds: Bounds): Point => [
  bounds[0] + (bounds[2] - bounds[0]) / 2,
  bounds[1] + (bounds[3] - bounds[1]) / 2,
];

/// If last and current segments have the same heading, skip the middle point
const simplifyElbowArrowPoints = (points: Point[]): Point[] =>
  points
    .slice(2)
    .reduce(
      (result, point) =>
        arePointsEqual(
          vectorToHeading(
            pointToVector(result[result.length - 1], result[result.length - 2]),
          ),
          vectorToHeading(pointToVector(point, result[result.length - 1])),
        )
          ? [...result.slice(0, -1), point]
          : [...result, point],
      [points[0] ?? [0, 0], points[1] ?? [1, 0]],
    );
