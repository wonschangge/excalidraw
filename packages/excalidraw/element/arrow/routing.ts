import {
  PointInTriangle,
  addVectors,
  arePointsEqual,
  distanceSq,
  dotProduct,
  isPointInsideBoundingBox,
  normalize,
  pointToVector,
  rotateVector,
  scaleUp,
  scaleVector,
  segmentsIntersectAt,
  toLocalSpace,
  toWorldSpace,
  vectorToHeading,
} from "../../math";
import Scene from "../../scene/Scene";
import { LocalPoint, Point, Segment, Vector } from "../../types";
import { Bounds, getElementBounds } from "../bounds";
import { ExcalidrawArrowElement, NonDeletedSceneElementsMap } from "../types";

const STEP_COUNT_LIMIT = 50;
const MIN_SELF_BOX_OFFSET = 30; // This will break self-avoidance for MIN_SELF_BOX_OFFSET close elements

export const calculateElbowArrowJointPoints = (
  arrow: ExcalidrawArrowElement,
): readonly LocalPoint[] => {
  if (arrow.points.length < 2) {
    // Arrow being created
    return arrow.points;
  }

  const target = toWorldSpace(arrow, arrow.points[arrow.points.length - 1]);
  const firstPoint = toWorldSpace(arrow, arrow.points[arrow.points.length - 2]);
  const avoidBounds = getStartEndBounds(arrow).filter(
    (bb): bb is Bounds => bb !== null,
  );

  const [startHeading, endHeading] = getNormalVectorsForStartEndElements(
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
    points.push(
      addVectors(dongle, scaleVector(startHeading, MIN_SELF_BOX_OFFSET)),
    );
  } else {
    const heading = vectorToHeading(pointToVector(firstPoint, target));
    points.push(
      addVectors(firstPoint, scaleVector(heading, MIN_SELF_BOX_OFFSET)),
    );
  }

  const endPoints = [];
  if (endHeading) {
    const dongle = extendSegmentToBoundingBoxEdge(
      [addVectors(target, endHeading), target],
      false,
      avoidBounds,
    );
    endPoints.push(
      addVectors(dongle, scaleVector(endHeading, MIN_SELF_BOX_OFFSET)),
    );
  } else {
    const heading = vectorToHeading(pointToVector(target, firstPoint));
    endPoints.push(
      addVectors(target, scaleVector(heading, -MIN_SELF_BOX_OFFSET)),
    );
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

    return containing
      .map(bboxToSegments) // TODO: This could be calcualted once in createRoute
      .flatMap((segments) =>
        segments!.map((segment) => segmentsIntersectAt(candidate, segment)),
      )
      .filter((x) => x !== null)[0]!;
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
  //const endNormal = rotateVector(endVector, Math.PI / 2);
  //const rightEndNormalDot = dotProduct([1, 0], endNormal);
  //const startNormalEndDot = dotProduct(startNormal, endVector);

  let next: Point =
    rightStartNormalDot === 0 // Last segment from start is horizontal
      ? [start[0], end[1]] // Turn up/down all the way to end
      : [end[0], start[1]]; // Turn left/right all the way to end

  next = resolveIntersections(start, next, boundingBoxes, startVector);

  const nextEndVector = normalize(pointToVector(end, next));
  const nextEndDot = dotProduct(nextEndVector, endVector);
  // const nextStartVector = normalize(pointToVector(start, next));
  // const nextStartDot = dotProduct(nextStartVector, startVector);
  const alignedButNotRightThere =
    (end[0] - next[0] === 0) !== (end[1] - next[1] === 0);

  if (nextEndDot === -1 && alignedButNotRightThere) {
    next =
      rightStartNormalDot === 0
        ? [start[0], end[1] + 40]
        : [end[0] + 40, start[1]];
  }

  return next;
};

const resolveIntersections = (
  start: Point,
  next: Point,
  boundingBoxes: Bounds[],
  startVector: Vector,
) => {
  const intersections = boundingBoxes
    .map(bboxToSegments)
    .flatMap((segments) =>
      segments!.map((segment) => segmentsIntersectAt([start, next], segment)),
    )
    .filter((x) => x != null);

  const intersection = intersections.sort(
    (a, b) => distanceSq(start, a!) - distanceSq(start, b!),
  )[0];

  return intersection && !arePointsEqual(intersection, start)
    ? addVectors(
        start,
        scaleVector(startVector, Math.sqrt(distanceSq(start, intersection))),
      )
    : next;
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
): [Vector | null, Vector | null] => {
  const [startBounds, endBounds] = getStartEndBounds(arrow);
  const startMidPoint: Point | null = startBounds && [
    startBounds[0] + (startBounds[2] - startBounds[0]) / 2,
    startBounds[1] + (startBounds[3] - startBounds[1]) / 2,
  ];
  const endMidPoint: Point | null = endBounds && [
    endBounds[0] + (endBounds[2] - endBounds[0]) / 2,
    endBounds[1] + (endBounds[3] - endBounds[1]) / 2,
  ];
  let startHeading: Vector | null = null;
  if (startBounds && startMidPoint) {
    const startTopLeft = scaleUp(
      [startBounds[0], startBounds[1]],
      startMidPoint,
    );
    const startTopRight = scaleUp(
      [startBounds[2], startBounds[1]],
      startMidPoint,
    );
    const startBottomLeft = scaleUp(
      [startBounds[0], startBounds[3]],
      startMidPoint,
    );
    const startBottomRight = scaleUp(
      [startBounds[2], startBounds[3]],
      startMidPoint,
    );
    startHeading = PointInTriangle(
      startPoint,
      startTopLeft,
      startTopRight,
      startMidPoint,
    )
      ? [0, -1]
      : PointInTriangle(
          startPoint,
          startTopRight,
          startBottomRight,
          startMidPoint,
        )
      ? [1, 0]
      : PointInTriangle(
          startPoint,
          startBottomRight,
          startBottomLeft,
          startMidPoint,
        )
      ? [0, 1]
      : [-1, 0];
  }

  let endHeading: Vector | null = null;
  if (endBounds && endMidPoint) {
    const endTopLeft = scaleUp([endBounds[0], endBounds[1]], endMidPoint);
    const endTopRight = scaleUp([endBounds[2], endBounds[1]], endMidPoint);
    const endBottomLeft = scaleUp([endBounds[0], endBounds[3]], endMidPoint);
    const endBottomRight = scaleUp([endBounds[2], endBounds[3]], endMidPoint);
    endHeading = PointInTriangle(
      endPoint,
      endTopLeft,
      endTopRight,
      endMidPoint!,
    )
      ? [0, -1]
      : PointInTriangle(endPoint, endTopRight, endBottomRight, endMidPoint)
      ? [1, 0]
      : PointInTriangle(endPoint, endBottomRight, endBottomLeft, endMidPoint)
      ? [0, 1]
      : [-1, 0];
  }

  return [startHeading, endHeading];
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

const bboxToSegments = (b: Bounds | null) =>
  b && [
    [[b[0], b[1]] as Point, [b[2], b[1]] as Point] as Segment,
    [[b[2], b[1]] as Point, [b[2], b[3]] as Point] as Segment,
    [[b[2], b[3]] as Point, [b[0], b[3]] as Point] as Segment,
    [[b[0], b[3]] as Point, [b[0], b[1]] as Point] as Segment,
  ];
