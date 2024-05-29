import {
  addVectors,
  arePointsEqual,
  distanceSq,
  doBoundsIntersect,
  dotProduct,
  isPointInsideBoundingBox,
  normalize,
  pointToVector,
  rotateVector,
  scaleVector,
  segmentsOverlap,
  toLocalSpace,
  toWorldSpace,
  vectorToHeading,
} from "../../math";
import Scene from "../../scene/Scene";
import type { LocalPoint, Point, Vector } from "../../types";
import type { Bounds } from "../bounds";
import type { ExcalidrawArrowElement } from "../types";
import { getHitOffset, getStartEndBounds } from "./common";
import {
  debugClear,
  debugDrawBounds,
  debugDrawPoint,
  debugNewFrame,
} from "./debug";
import {
  extendSegmentToBoundingBoxEdge,
  getHeadingForStartEndElements,
} from "./dongle";

const STEP_COUNT_LIMIT = 50;
const DONGLE_EXTENSION_SIZE = 50;
const HITBOX_EXTENSION_SIZE = 50;

export const calculateElbowArrowJointPoints = (
  arrow: ExcalidrawArrowElement,
): readonly LocalPoint[] => {
  if (arrow.points.length < 2) {
    // Arrow being created
    return arrow.points;
  }

  const scene = Scene.getScene(arrow);
  if (scene === null) {
    // The arrow is not on the scene??
    return arrow.points;
  }

  debugClear();

  const target = toWorldSpace(arrow, arrow.points[arrow.points.length - 1]);
  const firstPoint = toWorldSpace(arrow, arrow.points[0]);
  const [startHeading, endHeading] = getHeadingForStartEndElements(
    scene,
    arrow,
    firstPoint,
    target,
  );
  //console.log(startHeading, endHeading);
  const [startBounds, endBounds] = getStartEndBounds(
    arrow,
    firstPoint,
    target,
    startHeading,
    endHeading,
  );

  const points = [
    firstPoint,
    extendSegmentToBoundingBoxEdge(
      [firstPoint, addVectors(firstPoint, startHeading)],
      true,
      startBounds,
    ),
  ];
  const endPoints = [
    extendSegmentToBoundingBoxEdge(
      [addVectors(target, endHeading), target],
      false,
      endBounds,
    ),
    target,
  ];

  const avoidBounds = [startBounds, endBounds]
    .filter((bb): bb is Bounds => bb !== null)
    .filter(
      (bbox) =>
        !(
          isPointInsideBoundingBox(points[1], bbox) ||
          isPointInsideBoundingBox(endPoints[0], bbox)
        ),
    )
    .map((bbox) => {
      debugDrawBounds(bbox);
      return bbox;
    });

  // return simplifyElbowArrowPoints(
  //   calculateSegment(points, endPoints, avoidBounds).map((point) =>
  //     toLocalSpace(arrow, point),
  //   ),
  // );
  const p = calculateSegment(points, endPoints, avoidBounds).map((point) =>
    toLocalSpace(arrow, point),
  );

  p.forEach((x) => debugDrawPoint(x));

  return p;
};

// Calculates the poitns between a start segment and an end segment with elbows
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
  const startVectorIsHorizontal = rightStartNormalDot === 0;

  // The point generation happens in 4 phases:
  //
  // Phase 1: If the end dongle is ahead of the start dongle, go ahead as
  //          far as possible. If behind, turn toward the end dongle b
  //          90 degrees and go as far as possible.
  let next: Point = startVectorIsHorizontal
    ? endAhead
      ? [end[0], start[1]] // Go ahead
      : [start[0], end[1]] // Turn up/down all the way to end
    : endAhead
    ? [start[0], end[1]]
    : [end[0], start[1]]; // Turn left/right all the way to end

  // Phase 2: Do not go forward again if the previous segment we generated
  //          is continuing in the same direction, except the first step,
  //          because the start dongle might not go far enough on it's own.
  const startNextVector = normalize(pointToVector(next, start));
  if (step !== 0 && dotProduct(startNextVector, startVector) === 1) {
    next = rightStartNormalDot === 0 ? [start[0], end[1]] : [end[0], start[1]];
  }

  // Phase 3: Check the heading of the segment to the next point and the end
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

  // Phase 4: The last step is to check against the bounding boxes to see if
  //          the next segment would cross the bbox borders. Generate a new
  //          point if there is collision.
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
  //debugDrawPoint(next);

  return next;
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

    nextLeft && debugDrawPoint(nextLeft, "red");
    nextRight && debugDrawPoint(nextRight, "green");

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
