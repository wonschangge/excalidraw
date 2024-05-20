import { CaptureConsole } from "@sentry/integrations";
import {
  PointInTriangle,
  addVectors,
  arePointsEqual,
  distance2d,
  distanceSq,
  distanceSqOfPointFromSegment,
  dotProduct,
  isPointInsideBoundingBox,
  normalize,
  pointToVector,
  rotateVector,
  scaleUp,
  scaleVector,
  segmentsIntersectAt,
  subtractVectors,
  toLocalSpace,
  toWorldSpace,
  vectorToHeading,
} from "../../math";
import Scene from "../../scene/Scene";
import { LocalPoint, Point, Segment, Vector } from "../../types";
import { Bounds, getElementBounds } from "../bounds";
import { ExcalidrawArrowElement, NonDeletedSceneElementsMap } from "../types";
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
  console.log("-----");
  debugDrawClear();

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
    points.push(addVectors(dongle, scaleVector(startHeading, 20)));
  } else {
    const heading = vectorToHeading(pointToVector(firstPoint, target));
    points.push(addVectors(firstPoint, scaleVector(heading, 20)));
  }

  const endPoints = [];
  if (endHeading) {
    const dongle = extendSegmentToBoundingBoxEdge(
      [addVectors(target, endHeading), target],
      true,
      avoidBounds,
    );
    endPoints.push(addVectors(dongle, scaleVector(endHeading, 20)));
  } else {
    const heading = vectorToHeading(pointToVector(target, firstPoint));
    endPoints.push(addVectors(target, scaleVector(heading, -20)));
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
  for (let step = 0; step < 50; step++) {
    const next = kernel(points, end);
    //const next = avoidanceKernel(points, end, boundingBoxes);
    if (arePointsEqual(end[0], next)) {
      break;
    }
    points.push(next);
  }

  return points.concat(end);
};

const extendSegmentToBoundingBoxEdge = (
  segment: Segment,
  segmentIsStart: boolean,
  boundingBoxes: Bounds[],
) => {
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
              rightSegmentNormalDot > 0 ? minDist : -minDist,
            ]),
          ]
      : segmentIsHorizontal
      ? [
          end,
          subtractVectors(end, [rightSegmentDot > 0 ? -minDist : minDist, 0]),
        ]
      : [
          end,
          subtractVectors(end, [
            0,
            rightSegmentNormalDot > 0 ? minDist : -minDist,
          ]),
        ];

    const dists = containing
      .map(bboxToSegments) // TODO: This could be calcualted once in createRoute
      .flatMap((segments) =>
        segments!.map((segment) => segmentsIntersectAt(candidate, segment)),
      )
      .filter((x) => x !== null);
    const dongle = dists.sort(
      (a, b) => distanceSq(a!, start) - distanceSq(b!, start),
    )[0]!;

    return segmentIsStart ? dongle : dongle;
  }

  return segmentIsStart ? segment[0] : segment[1];
};

// const avoidanceKernel = (
//   points: Point[],
//   target: Point[],
//   boundingBoxes: Bounds[],
// ): Point => {
//   const start = points[points.length - 1];
//   const startHead = points[points.length - 2];
//   const end = target[0];
//   const startVector =
//     points.length < 2
//       ? ([1, 0] as Vector) // TODO: Fixed right start attachment
//       : normalize(pointToVector(start, startHead));
//   const endVector =
//     target.length < 2
//       ? ([-1, 0] as Vector) // TODO: Fixed left end attachment
//       : normalize(pointToVector(target[1], end));
//   const startNormal = rotateVector(startVector, Math.PI / 2);
//   const endNormal = rotateVector(endVector, Math.PI / 2);
//   const startEndDot = dotProduct(startVector, endVector);
//   const startNormalEndDot = dotProduct(startNormal, endVector);
//   const rightStartNormalDot = dotProduct([1, 0], startNormal);
//   const rightEndNormalDot = dotProduct([1, 0], endNormal);
//   const rightStartDot = dotProduct([1, 0], startVector);

//   const boundingBoxesContainingStart = boundingBoxes
//     .filter((bBox) => isPointInsideBoundingBox(start, bBox))
//     .map((bbox) => {
//       debugDrawSegments(bboxToSegments(bbox), "green");
//       return bbox;
//     });

//   // If this is > 1 it means the arrow is in an overlapping shape
//   if (boundingBoxesContainingStart.length > 0) {
//     if (rightStartNormalDot === 0) {
//       // Start vector is horizontal
//       const minDist = boundingBoxesContainingStart
//         .map((bbox) => bbox[2] - bbox[0])
//         .reduce((largest, value) => (value > largest ? value : largest), 0);
//       const candidate = [
//         start,
//         addVectors(start, [rightStartDot > 0 ? minDist : -minDist, 0]),
//       ] as Segment;
//       const dists = boundingBoxesContainingStart
//         .map(bboxToSegments) // TODO: This could be calcualted once in createRoute
//         .flatMap((segments) =>
//           segments.map((segment) => segmentsIntersectAt(candidate, segment)),
//         )
//         .filter((x) => x !== null);
//       return dists.sort(
//         (a, b) => distanceSq(a!, start) - distanceSq(b!, start),
//       )[0]!;
//     }
//     // Start vector is vertical
//     const minDist = boundingBoxesContainingStart
//       .map((bbox) => bbox[3] - bbox[1])
//       .reduce((largest, value) => (value > largest ? value : largest), 0);
//     const candidate = [
//       start,
//       addVectors(start, [0, rightStartDot > 0 ? minDist : -minDist]),
//     ] as Segment;
//     const dists = boundingBoxesContainingStart
//       .map(bboxToSegments) // TODO: This could be calcualted once in createRoute
//       .flatMap((segments) =>
//         segments.map((segment) => segmentsIntersectAt(candidate, segment)),
//       )
//       .filter((x) => x !== null);
//     return dists.sort(
//       (a, b) => distanceSq(a!, start) - distanceSq(b!, start),
//     )[0]!;
//   }

//   const next = kernel(points, target, []);

//   const boundingBoxesContainingNext = boundingBoxes
//     .filter((bBox) => isPointInsideBoundingBox(next, bBox))
//     .map((bbox) => {
//       debugDrawSegments(bboxToSegments(bbox), "red");
//       return bbox;
//     });

//   if (boundingBoxesContainingNext.length > 0) {
//     //const nextEndDot = dotProduct(pointToVector(next, startHead), endNormal);
//     if (rightEndNormalDot === 0) {
//       // End vector is horizontal
//       const minDist = boundingBoxesContainingNext
//         .map((bbox) => bbox[2] - bbox[0])
//         .reduce((largest, value) => (value > largest ? value : largest), 0);
//     }
//   }

//   return kernel(points, target, []);

//   // 1.a If start and end are colinear and pointing
//   //     to the same direction, just jump to the end.
// };

const kernel = (points: Point[], target: Point[]): Point => {
  const start = points[points.length - 1];
  const end = target[0];
  const startVector =
    points.length < 2
      ? ([0, 0] as Vector) // TODO: Fixed right start attachment
      : normalize(pointToVector(start, points[points.length - 2]));
  //  console.log(points.length);
  const endVector =
    target.length < 2
      ? ([0, 0] as Vector) // TODO: Fixed left end attachment
      : normalize(pointToVector(target[1], end));
  const startNormal = rotateVector(startVector, Math.PI / 2);
  const rightStartNormalDot = dotProduct([1, 0], startNormal);
  const startNormalEndDot = dotProduct(startNormal, endVector);
  console.log(startNormalEndDot);
  const next: Point =
    rightStartNormalDot === 0 // Last segment from start is horizontal
      ? startNormalEndDot === 1
        ? [end[0], start[1]]
        : [start[0], end[1]]
      : startNormalEndDot === 1
      ? [start[0], end[1]]
      : [end[0], start[1]];

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

/*

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

  // debugDrawSegments(startLineSegments);
  // debugDrawSegments(endLineSegments);

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

const directedSegmentsIntersectionPointWithObtuseAngle = (
  a: Readonly<Segment>,
  b: Readonly<Segment>,
): Point | null => {
  const aVector = pointToVector(a[1], a[0]);
  const bVector = pointToVector(b[1], b[0]);

  if (dotProduct(aVector, bVector) < 0) {
    debugDrawSegments(b, "red");
    return segmentsIntersectAt(a, b);
  }

  return null;
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


 */
