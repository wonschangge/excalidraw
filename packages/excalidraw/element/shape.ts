import {
  GeometricShape,
  getClosedCurveShape,
  getCurveShape,
  getEllipseShape,
  getFreedrawShape,
  getPolygonShape,
} from "../../utils/geometry/shape";
import { ShapeCache } from "../scene/ShapeCache";
import { getElementAbsoluteCoords } from "./bounds";
import { shouldTestInside } from "./collision";
import { ExcalidrawElement, NonDeletedSceneElementsMap } from "./types";

/**
 * get the pure geometric shape of an excalidraw element
 * which is then used for hit detection
 */
export const getElementShape = (
  element: ExcalidrawElement,
  elementsMap: NonDeletedSceneElementsMap,
): GeometricShape => {
  switch (element.type) {
    case "rectangle":
    case "diamond":
    case "frame":
    case "magicframe":
    case "embeddable":
    case "image":
    case "iframe":
    case "text":
    case "selection":
      return getPolygonShape(element);
    case "arrow":
    case "line": {
      const roughShape =
        ShapeCache.get(element)?.[0] ??
        ShapeCache.generateElementShape(element, null)[0];
      const [, , , , cx, cy] = getElementAbsoluteCoords(element, elementsMap);

      return shouldTestInside(element)
        ? getClosedCurveShape(
            element,
            roughShape,
            [element.x, element.y],
            element.angle,
            [cx, cy],
          )
        : getCurveShape(roughShape, [element.x, element.y], element.angle, [
            cx,
            cy,
          ]);
    }

    case "ellipse":
      return getEllipseShape(element);

    case "freedraw": {
      const [, , , , cx, cy] = getElementAbsoluteCoords(element, elementsMap);
      return getFreedrawShape(element, [cx, cy], shouldTestInside(element));
    }
  }
};
