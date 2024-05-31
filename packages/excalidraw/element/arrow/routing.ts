import {
  PointInTriangle,
  addVectors,
  arePointsEqual,
  distance2d,
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
import {
  distanceToBindableElement,
  getHoveredElementForBinding,
  maxBindingGap,
} from "../binding";
import { type BoundingBox, type Bounds } from "../bounds";
import type {
  ExcalidrawArrowElement,
  ExcalidrawBindableElement,
  ExcalidrawElement,
} from "../types";
import {
  debugClear,
  debugDrawBounds,
  debugDrawPoint,
  debugDrawSegments,
  debugNewFrame,
} from "./debug";

const STEP_COUNT_LIMIT = 50;
const MIN_DONGLE_SIZE = 30;
const DONGLE_EXTENSION_SIZE = 150;
const HITBOX_EXTENSION_SIZE = 30;

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
  const [startHeading, endHeading] = getHeadingForStartEndElements(
    arrow,
    firstPoint,
    target,
  );
  const [startDongleBounds, endDongleBounds] = getStartEndBounds(
    arrow,
    firstPoint,
    target,
    startHeading,
    endHeading,
  );
  const points = [
    firstPoint,
    startHeading
      ? extendSegmentToBoundingBoxEdge(
          [firstPoint, addVectors(firstPoint, startHeading)],
          true,
          startDongleBounds !== null ? [startDongleBounds] : [],
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
          endDongleBounds !== null ? [endDongleBounds] : [],
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
  debugDrawPoint(points[0], "green");
  debugDrawPoint(points[1], "green");
  const avoidBounds = getStartEndBounds(
    arrow,
    firstPoint,
    target,
    startHeading,
    endHeading,
  )
    .filter((bb): bb is Bounds => bb !== null)
    .map((bb) => {
      debugDrawBounds(bb, "green");
      return bb;
    })
    .filter(
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

// Calculates the points between a start segment and an end segment with elbows
const calculateSegment = (
  start: readonly Point[],
  end: Point[],
  boundingBoxes: Bounds[],
): Point[] => {
  const points: Point[] = Array.from(start);
  // Limit max step to avoid infinite loop
  for (let step = 0; step < STEP_COUNT_LIMIT; step++) {
    // If the last generated point (or the start dongle) is inside a
    // bounding box then disable that bbox to avoid hitting in from the
    // "inside"
    const externalBoundingBoxes = boundingBoxes.filter(
      (bbox) => !isPointInsideBoundingBox(points[points.length - 1], bbox),
    );
    const next = kernel(points, end, externalBoundingBoxes, step);

    if (arePointsEqual(end[0], next)) {
      // We have reached the end dongle with the last step
      break;
    }

    points.push(next);
  }

  if (points.length > STEP_COUNT_LIMIT) {
    console.error("Elbow arrow routing step count limit reached", points);
  }

  // As the last step, connect to the end dongle since we skipped the
  // `points.push(next)` for the last step
  return points.concat(end);
};

// Generates the coordinate for the next point given the previous points
// and bounding boxes and the target dongle.
const kernel = (
  points: Point[],
  target: Point[],
  boundingBoxes: Bounds[],
  step: number,
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

  // The point generation happens in 5 phases:
  //
  // Phase 1: If the end dongle is ahead of the start dongle, go ahead as
  //          far as possible. If behind, turn toward the end dongle b
  //          90 degrees and go as far as possible.
  let next: Point =
    rightStartNormalDot === 0 // Last segment from start is horizontal
      ? endAhead
        ? [end[0], start[1]]
        : [start[0], end[1]] // Turn up/down all the way to end
      : endAhead
      ? [start[0], end[1]]
      : [end[0], start[1]]; // Turn left/right all the way to end

  if (step === 0) {
    const intersections = boundingBoxes
      .map(bboxToClockwiseWoundingSegments)
      .flatMap((segments) =>
        segments!.map((segment) => {
          const p = segmentsIntersectAt([start, next], segment);

          return p
            ? [p, segment[0], segment[1]]
            : [
                [Infinity, Infinity],
                [0, 0],
                [0, 0],
              ];
        }),
      )
      .filter((x) => x != null)
      .filter((x) => x[0][0] < Infinity);

    if (intersections.length === 2) {
      console.log("AH!");
      // const [hitPoint, leftPoint, rightPoint] = intersections[0] as Point[];
      // const startNextVector = normalize(pointToVector(next, start));
      // const len = distanceSq(start, next);
      // const lenLeft = distanceSq(hitPoint, leftPoint);
      // const lenRight = distanceSq(hitPoint, rightPoint);
      // const leftVec = normalize(pointToVector(leftPoint, hitPoint));
      // if (dotProduct(leftVec, startNextVector) === 1 && lenRight < len) {
      //   const oppositeVector = rotateVector(startNextVector, Math.PI);
      //   next = addVectors(start, scaleVector(oppositeVector, lenRight));
      // }

      //debugDrawPoint(start, "orange");
    }
  }

  // Phase 3: Do not go forward again if the previous segment we generated
  //          is continuing in the same direction, except the first step,
  //          because the start dongle might not go far enough on it's own.
  if (step !== 0) {
    // Segment start is only a stub, so allow going forward
    const startNextVector = normalize(pointToVector(next, start));
    if (dotProduct(startNextVector, startVector) === 1) {
      next =
        rightStartNormalDot === 0 ? [start[0], end[1]] : [end[0], start[1]];
    }
  }

  // Phase 4: Check the heading of the segment to the next point and the end
  //          dongle, determine of the next segment vector and the end dongle
  //          vector are facing each other or not. Also determine if the next
  //          and end dongles are aligned in either the X or the Y axis. If
  //          they are directly in front of each other then we need to go left
  //          or right to avoid collision and make a loop around.
  const nextEndVector = normalize(pointToVector(end, next));
  const nextEndDot = dotProduct(nextEndVector, endVector);
  const alignedButNotRightThere =
    Math.abs(vectorToHeading(nextEndVector)[0]) === 1
      ? end[0] - next[0] !== 0 && end[1] - next[1] === 0
      : end[0] - next[0] === 0 && end[1] - next[1] !== 0;

  if (nextEndDot === -1 && alignedButNotRightThere) {
    next = addVectors(start, scaleVector(pointToVector(next, start), 0.5));
  }

  // Phase 5: The last step is to check against the bounding boxes to see if
  //          the next segment would cross the bbox borders. Generate a new
  //          point if there is collision.
  if (boundingBoxes.length > 0) {
    const startNextVector = normalize(pointToVector(next, start));
    next = resolveIntersections(
      points,
      next,
      boundingBoxes,
      end,
      dotProduct(endVector, startNextVector) === -1,
    );
  }

  //debugDrawPoint(next);

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
          segmentIsStart
            ? normalize(vector)
            : normalize(pointToVector(start, end)),
        )
      : segmentIsStart
      ? segment[0]
      : segment[1];
  }

  return segmentIsStart ? segment[0] : segment[1];
};

const getHitOffset = (start: Point, next: Point, boundingBoxes: Bounds[]) =>
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
    ) ?? [0, 0, 0];

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
    offsetAhead > DONGLE_EXTENSION_SIZE - HITBOX_EXTENSION_SIZE + 1 && // TODO: This is a stand-in for still checking the start bound, but the start bound only
    boundingBoxes[0] &&
    boundingBoxes[1] &&
    !doBoundsIntersect(boundingBoxes[0], boundingBoxes[1])
  ) {
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
      scaleVector(altNextLeftDirection, Math.min(offsetLeft, offsetRight) + 1),
    );
    const altNextRightDirection = rotateVector(
      normalize(pointToVector(next, start)),
      Math.PI / 2,
    );
    const nextRightCandidate = addVectors(
      start,
      scaleVector(altNextRightDirection, Math.min(offsetLeft, offsetRight) + 1),
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

    // nextLeft && debugDrawPoint(nextLeft, "red");
    // nextRight && debugDrawPoint(nextRight, "green");

    const nextLeftEndDistance = nextLeft
      ? Math.sqrt(distanceSq(nextLeft, target)) + offsetRight
      : Infinity;
    const nextRightEndDistance = nextRight
      ? Math.sqrt(distanceSq(nextRight, target)) + offsetLeft
      : Infinity;
    // console.log(offsetAhead, offsetLeft, offsetRight);
    // console.log(nextLeftEndDistance, nextRightEndDistance);
    if (nextLeftEndDistance < nextRightEndDistance) {
      return nextLeft ? nextLeft : nextRight ? nextRight : next;
    }

    return nextRight ? nextRight : nextLeft ? nextLeft : next;
  }

  return next;
};

const getCommonAABB = (a: Bounds, b: Bounds): Bounds => [
  Math.min(a[0], b[0]),
  Math.min(a[1], b[1]),
  Math.max(a[2], b[2]),
  Math.max(a[3], b[3]),
];

// Returns the adjusted axis-aligned "frame" with equal distance between
// start and end bound shapes if present.
const getStartEndBounds = (
  arrow: ExcalidrawArrowElement,
  startPoint: Point,
  endPoint: Point,
  startHeading: Vector | null,
  endHeading: Vector | null,
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

  const BIAS = 50;
  const startBoundingBox =
    startEndElements[0] &&
    (extendedBoundingBoxForElement(startEndElements[0], BIAS) as [
      number,
      number,
      number,
      number,
    ]);
  const endBoundingBox =
    startEndElements[1] &&
    (extendedBoundingBoxForElement(startEndElements[1], BIAS) as [
      number,
      number,
      number,
      number,
    ]);

  if (
    startBoundingBox &&
    endBoundingBox &&
    startEndElements[0] &&
    startEndElements[1]
  ) {
    const commonBbox = getCommonAABB(startBoundingBox, endBoundingBox);
    debugDrawBounds(commonBbox);
    const verticalDistance =
      commonBbox[3] -
      commonBbox[1] -
      2 * BIAS -
      (startEndElements[0].height + startEndElements[1].height);
    const horizontalDistance =
      commonBbox[2] -
      commonBbox[0] -
      2 * BIAS -
      (startEndElements[0].width + startEndElements[1].width);

    if (verticalDistance > 0) {
      // Not overlapping vertically
      if (
        startEndElements[0].y + startEndElements[0].height <
        startEndElements[1].y
      ) {
        // Start is higher than end
        const start = startHeading === DOWN ? MIN_DONGLE_SIZE : 0;
        const end = endHeading === UP ? MIN_DONGLE_SIZE : 0;
        const distance = (verticalDistance - start - end) / 2 - 1;
        startBoundingBox[3] = startBoundingBox[3] - BIAS + start + distance;
        endBoundingBox[1] = endBoundingBox[1] + BIAS - end - distance;
      } else {
        // Start is lower than end
        const start = startHeading === UP ? MIN_DONGLE_SIZE : 0;
        const end = endHeading === DOWN ? MIN_DONGLE_SIZE : 0;
        const distance = (verticalDistance - start - end) / 2 - 1;
        startBoundingBox[1] = startBoundingBox[1] + BIAS - start - distance;
        endBoundingBox[3] = endBoundingBox[3] - BIAS + end + distance;
      }
    }
    if (horizontalDistance > 0) {
      // Not overlapping horizontally
      if (
        startEndElements[0].x + startEndElements[0].width <
        startEndElements[1].x
      ) {
        // Start is to the left of end
        const start = startHeading === RIGHT ? MIN_DONGLE_SIZE : 0;
        const end = endHeading === LEFT ? MIN_DONGLE_SIZE : 0;
        const distance = (horizontalDistance - start - end) / 2 - 1;
        startBoundingBox[2] = startBoundingBox[2] - BIAS + start + distance;
        endBoundingBox[0] = endBoundingBox[0] + BIAS - end - distance;
      } else {
        // Start is to the right of end
        const start = startHeading === LEFT ? MIN_DONGLE_SIZE : 0;
        const end = endHeading === RIGHT ? MIN_DONGLE_SIZE : 0;
        const distance = (horizontalDistance - start - end) / 2 - 1;
        startBoundingBox[0] = startBoundingBox[0] + BIAS - start - distance;
        endBoundingBox[2] = endBoundingBox[2] - BIAS + end + distance;
      }
    }
  }

  // Finally we need to check somehow if the arrow endpoint is dragged out of
  // the binding area to disconnect arrowhead tracking from the bindable shape
  return [
    startEndElements[0] &&
    isPointInsideBoundingBox(
      startPoint,
      extendedBoundingBoxForElement(
        startEndElements[0],
        maxBindingGap(
          startEndElements[0],
          startEndElements[0].width,
          startEndElements[0].height,
        ), // This is the binding area size!
      ),
    )
      ? startBoundingBox
      : null,
    startEndElements[1] &&
    isPointInsideBoundingBox(
      endPoint,
      extendedBoundingBoxForElement(
        startEndElements[1],
        maxBindingGap(
          startEndElements[1],
          startEndElements[1].width,
          startEndElements[1].height,
        ), // This is the binding area size!
      ),
    )
      ? endBoundingBox
      : null,
  ];
};

const extendedBoundingBoxForElement = (
  element: ExcalidrawElement,
  offset: number,
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

  return extendedBounds;
};

const getHeadingForStartEndElements = (
  arrow: ExcalidrawArrowElement,
  startPoint: Point,
  endPoint: Point,
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
      start &&
        getHeadingForWorldPointFromElement(
          start,
          startPoint,
          maxBindingGap(start, start.width, start.height),
        ),
      end &&
        getHeadingForWorldPointFromElement(
          end,
          endPoint,
          maxBindingGap(end, end.width, end.height),
        ),
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
  offset: number,
): Heading | null => {
  if (
    !isPointInsideBoundingBox(
      point,
      extendedBoundingBoxForElement(element, offset),
    )
  ) {
    return null;
  }

  const SEARCH_CONE_MULTIPLIER = 2;
  const bounds = extendedBoundingBoxForElement(element, offset);
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

const distanceBetweenStartEndBinds = (arrow: ExcalidrawArrowElement) => {
  const scene = Scene.getScene(arrow);
  if (scene === null) {
    return null;
  }

  const elementsMap = scene.getNonDeletedElementsMap();
  const startEl = ((arrow.startBinding &&
    elementsMap.get(arrow.startBinding.elementId)) ??
    null) as ExcalidrawBindableElement | null;
  const endEl = ((arrow.endBinding &&
    elementsMap.get(arrow.endBinding.elementId)) ??
    null) as ExcalidrawBindableElement | null;

  return (
    startEl &&
    endEl &&
    Math.min(
      distanceToBindableElement(startEl, [endEl.x, endEl.y], elementsMap),
      distanceToBindableElement(
        startEl,
        [endEl.x + endEl.width, endEl.y],
        elementsMap,
      ),
      distanceToBindableElement(
        startEl,
        [endEl.x + endEl.width, endEl.y + endEl.height],
        elementsMap,
      ),
      distanceToBindableElement(
        startEl,
        [endEl.x, endEl.y + endEl.height],
        elementsMap,
      ),
    )
  );
};

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
