import {
  PointInTriangle,
  addVectors,
  dotProduct,
  isPointInsideBoundingBox,
  normalize,
  pointToVector,
  rotatePoint,
  rotateVector,
  scaleUp,
  scaleVector,
  segmentsIntersectAt,
  vectorToHeading,
} from "../../math";
import type Scene from "../../scene/Scene";
import type { Point, Segment } from "../../types";
import { getHoveredElementForBinding, maxBindingGap } from "../binding";
import type { Bounds } from "../bounds";
import type { ExcalidrawArrowElement, ExcalidrawElement } from "../types";
import type { Heading } from "./common";
import {
  DOWN,
  LEFT,
  RIGHT,
  UP,
  aabbForElement,
  bboxToClockwiseWoundingSegments,
} from "./common";

export const extendSegmentToBoundingBoxEdge = (
  segment: Segment,
  segmentIsStart: boolean,
  boundingBox: Bounds | null,
): Point => {
  const [start, end] = segment;
  const vector = pointToVector(end, start);
  const normal = rotateVector(vector, Math.PI / 2);
  const rightSegmentNormalDot = dotProduct([1, 0], normal);
  const segmentIsHorizontal = rightSegmentNormalDot === 0;
  const rightSegmentDot = dotProduct([1, 0], vector);

  if (boundingBox) {
    const minDist = segmentIsHorizontal
      ? boundingBox[2] - boundingBox[0]
      : boundingBox[3] - boundingBox[1];

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

    const intersection = bboxToClockwiseWoundingSegments(boundingBox)
      ?.map((segment) => segmentsIntersectAt(candidate, segment))
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

// The arrow dongle orientation is critical to determine a stable
// and correct route for the arrow. This function determines the
// start and end dongle vectors depending on whether the arrow
// is bound to shapes or where it is relative to it's adjacent end
export const getHeadingForStartEndElements = (
  scene: Scene,
  arrow: ExcalidrawArrowElement,
  startPoint: Point,
  endPoint: Point,
): [Heading, Heading] => {
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

  const startHeadingCandidate =
    start && getHeadingForWorldPointFromElement(start, startPoint);
  const endHeadingCandidate =
    end && getHeadingForWorldPointFromElement(end, endPoint);

  const v = [
    startHeadingCandidate !== null
      ? startHeadingCandidate
      : addVectors(
          startPoint,
          scaleVector(vectorToHeading(pointToVector(endPoint, startPoint)), 50),
        ),
    endHeadingCandidate !== null
      ? normalize(endHeadingCandidate)
      : addVectors(
          endPoint,
          scaleVector(vectorToHeading(pointToVector(startPoint, endPoint)), 50),
        ),
  ] as [Heading, Heading];

  return v;
};

// Gets the heading for the point by creating a bounding box around the rotated
// close fitting bounding box, then creating 4 search cones around the center of
// the external bbox.
const getHeadingForWorldPointFromElement = (
  element: ExcalidrawElement,
  point: Point,
): Heading | null => {
  const dist = maxBindingGap(element, element.width, element.height);
  const bounds = aabbForElement(element, Array(4).fill(dist));

  if (!isPointInsideBoundingBox(point, bounds)) {
    return null;
  }

  const SEARCH_CONE_MULTIPLIER = 2;
  const ROTATION = element.type === "diamond" ? Math.PI / 4 : 0;

  const midPoint = [
    bounds[0] + (bounds[2] - bounds[0]) / 2,
    bounds[1] + (bounds[3] - bounds[1]) / 2,
  ] as [number, number];
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
