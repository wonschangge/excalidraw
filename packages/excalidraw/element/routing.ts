import { ExcalidrawArrowElement, NonDeletedSceneElementsMap } from "./types";
import { Point } from "../types";
import { arePointsEqual } from "../math";
import { mutateElement } from "./mutateElement";

// ========================================
// The main idea is to Ray March the arrow
// ========================================

/// Recalculates the points of the arrow, except the start point
export const routeArrow = (
  arrow: ExcalidrawArrowElement,
  firstPointIsStart: boolean,
  target: Point,
  elementsMap: NonDeletedSceneElementsMap,
) => {
  const points = [
    firstPointIsStart ? arrow.points[0] : arrow.points[arrow.points.length - 1],
  ];

  // Limit max step to avoid infinite loop
  for (let step = 0; step < 50; step++) {
    const next = kernel(points[step], target, elementsMap);
    if (arePointsEqual(next, target)) {
      break;
    }
    points.push(next);
  }

  points.push(target);
  mutateElement(arrow, { points });
};

const kernel = (
  origin: Point,
  target: Point,
  elementsMap: NonDeletedSceneElementsMap,
): Point => {
  switch (true) {
    case origin[0] < target[0] && origin[1] < target[1]:
      // Target is bottom right from us
      return [target[0], origin[1]];
    case origin[0] < target[0] && origin[1] > target[1]:
      // Target is top right from us
      return [origin[0], target[1]];
    case origin[0] > target[0] && origin[1] < target[1]:
      // Target is bottom left from us
      return [origin[0], target[1]];
    case origin[0] > target[0] && origin[1] > target[1]:
      // Target is top left from us
      return [target[0], origin[1]];
  }

  // Target is exactly in-line with us - just connect to target and finish
  return target;
};
