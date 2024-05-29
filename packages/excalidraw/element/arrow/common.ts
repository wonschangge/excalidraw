import {
  arePointsEqual,
  distanceSq,
  isPointInsideBoundingBox,
  pointToVector,
  rotatePoint,
  segmentsIntersectAt,
  vectorToHeading,
} from "../../math";
import Scene from "../../scene/Scene";
import type { Point, Segment } from "../../types";
import {
  distanceToBindableElement,
  getHoveredElementForBinding,
  maxBindingGap,
} from "../binding";
import type { BoundingBox, Bounds } from "../bounds";
import type {
  ExcalidrawArrowElement,
  ExcalidrawBindableElement,
  ExcalidrawElement,
} from "../types";
import { debugDrawSegments } from "./debug";

export type Heading = [1, 0] | [-1, 0] | [0, 1] | [0, -1];
export const UP = [0, -1] as Heading;
export const RIGHT = [1, 0] as Heading;
export const DOWN = [0, 1] as Heading;
export const LEFT = [-1, 0] as Heading;

// Generates an axis-aligned bounding box which grows or shrinks
// to accomodate the bounding box of a rotated element. Additionally
// it will extended the generated AABB with the offset values provided.
export const aabbForElement = (
  element: ExcalidrawElement,
  offsets: number[] = [50, 50, 50, 50],
): Bounds => {
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
    Math.min(topLeftX, topRightX, bottomRightX, bottomLeftX) - offsets[0],
    Math.min(topLeftY, topRightY, bottomRightY, bottomLeftY) - offsets[1],
    Math.max(topLeftX, topRightX, bottomRightX, bottomLeftX) + offsets[2],
    Math.max(topLeftY, topRightY, bottomRightY, bottomLeftY) + offsets[3],
  ] as Bounds;

  return extendedBounds;
};

// Turn a bounding box into 4 clockwise wounding segments
export const bboxToClockwiseWoundingSegments = (b: Bounds | null) =>
  b && [
    [[b[0], b[1]] as Point, [b[2], b[1]] as Point] as Segment,
    [[b[2], b[1]] as Point, [b[2], b[3]] as Point] as Segment,
    [[b[2], b[3]] as Point, [b[0], b[3]] as Point] as Segment,
    [[b[0], b[3]] as Point, [b[0], b[1]] as Point] as Segment,
  ];

export const getHitOffset = (
  start: Point,
  next: Point,
  boundingBoxes: Bounds[],
) => {
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

export const distanceBetweenElements = (
  a: ExcalidrawBindableElement,
  b: ExcalidrawBindableElement,
) => {
  const scene = Scene.getScene(a);
  if (scene === null) {
    return null;
  }

  const elementsMap = scene.getNonDeletedElementsMap();

  return Math.min(
    distanceToBindableElement(a, [b.x, b.y], elementsMap),
    distanceToBindableElement(a, [b.x + b.width, b.y], elementsMap),
    distanceToBindableElement(a, [b.x + b.width, b.y + b.height], elementsMap),
    distanceToBindableElement(a, [b.x, b.y + b.height], elementsMap),
  );
};

/// If last and current segments have the same heading, skip the middle point
export const simplifyElbowArrowPoints = (points: Point[]): Point[] =>
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

export const pointIsInBindingAreaForElement = (
  el: ExcalidrawBindableElement | null,
  p: Point,
) =>
  (el &&
    isPointInsideBoundingBox(
      p,
      aabbForElement(el, Array(4).fill(maxBindingGap(el, el.width, el.height))),
    )) ??
  false;

export const getStartEndBounds = (
  arrow: ExcalidrawArrowElement,
  startPoint: Point,
  endPoint: Point,
  startHeading: Heading,
  endHeading: Heading,
): [Bounds | null, Bounds | null] => {
  const startEndElements = getStartEndElements(arrow, startPoint, endPoint);

  const dist =
    startEndElements[0] &&
    startEndElements[1] &&
    distanceBetweenElements(startEndElements[0], startEndElements[1]);

  const MAX_DIST = 50;
  const variableDist = Math.min(
    Math.max(MAX_DIST, (dist ?? MAX_DIST) / 2 - 1),
    10,
  );
  const startBoundingBox =
    startEndElements[0] &&
    aabbForElement(startEndElements[0], [
      startHeading === LEFT ? variableDist : MAX_DIST,
      startHeading === UP ? variableDist : MAX_DIST,
      startHeading === RIGHT ? variableDist : MAX_DIST,
      startHeading === DOWN ? variableDist : MAX_DIST,
    ]);
  const endBoundingBox =
    startEndElements[1] &&
    aabbForElement(startEndElements[1], [
      endHeading === LEFT ? variableDist : MAX_DIST,
      endHeading === UP ? variableDist : MAX_DIST,
      endHeading === RIGHT ? variableDist : MAX_DIST,
      endHeading === DOWN ? variableDist : MAX_DIST,
    ]);

  // Finally we need to check somehow if the arrow endpoint is dragged out of
  // the binding area to disconnect arrowhead tracking from the bindable shape
  return [
    pointIsInBindingAreaForElement(startEndElements[0], startPoint)
      ? startBoundingBox
      : null,
    pointIsInBindingAreaForElement(startEndElements[1], endPoint)
      ? endBoundingBox
      : null,
  ];
};

const getStartEndElements = (
  arrow: ExcalidrawArrowElement,
  startPoint: Point,
  endPoint: Point,
) => {
  const scene = Scene.getScene(arrow);
  if (!scene) {
    return [null, null];
  }

  const elementsMap = scene.getNonDeletedElementsMap();

  return [
    arrow.startBinding
      ? (elementsMap.get(
          arrow.startBinding.elementId,
        ) as ExcalidrawBindableElement) ?? null
      : getHoveredElementForBinding(
          { x: startPoint[0], y: startPoint[1] },
          scene.getNonDeletedElements(),
          elementsMap,
        ),
    arrow.endBinding
      ? (elementsMap.get(
          arrow.endBinding.elementId,
        ) as ExcalidrawBindableElement) ?? null
      : getHoveredElementForBinding(
          { x: endPoint[0], y: endPoint[1] },
          scene.getNonDeletedElements(),
          elementsMap,
        ),
  ];
};
