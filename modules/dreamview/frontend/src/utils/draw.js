import * as THREE from 'three';
import ThreeLine2D from 'three-line-2d';
import ThreeLine2DBasicShader from 'three-line-2d/shaders/basic';
import { copyProperty } from './misc';

const _ = require('lodash');

const DELTA_Z_OFFSET = 0.04;
const Line = ThreeLine2D(THREE);
const BasicShader = ThreeLine2DBasicShader(THREE);
const textureLoader = new THREE.TextureLoader();

export function addOffsetZ(mesh, value) {
  if (value) {
    const zOffset = value * DELTA_Z_OFFSET;
    mesh.position.z += zOffset;
  }
}

export function drawImage(img, width, height, x = 0, y = 0, z = 0) {
  const material = new THREE.MeshBasicMaterial(
    {
      map: textureLoader.load(img),
      transparent: true,
      depthWrite: false,
    },
  );
  const mesh = new THREE.Mesh(new THREE.PlaneGeometry(width, height), material);
  mesh.material.side = THREE.DoubleSide;
  mesh.position.set(x, y, z);
  mesh.overdraw = true;

  return mesh;
}

export function drawDashedLineFromPoints(
  points, color = 0xff0000, linewidth = 1, dashSize = 4, gapSize = 2,
  zOffset = 0, opacity = 1, matrixAutoUpdate = true,
) {
  const path = new THREE.Path();
  const geometry = path.createGeometry(points);
  geometry.computeLineDistances();
  const material = new THREE.LineDashedMaterial({
    color,
    dashSize,
    linewidth,
    gapSize,
    transparent: true,
    opacity,
  });
  const mesh = new THREE.Line(geometry, material);
  addOffsetZ(mesh, zOffset);
  mesh.matrixAutoUpdate = matrixAutoUpdate;
  if (!matrixAutoUpdate) {
    mesh.updateMatrix();
  }
  return mesh;
}

export function drawCircle(radius, material, segments = 32) {
  const geometry = new THREE.CircleGeometry(radius, segments);
  const circleMesh = new THREE.Mesh(geometry, material);
  return circleMesh;
}

export function drawEllipse(aRadius, bRadius, material) {
  const path = new THREE.Shape();
  path.absellipse(0, 0, aRadius, bRadius, 0, Math.PI * 2, false, 0);
  const geometry = new THREE.ShapeBufferGeometry(path);
  const ellipse = new THREE.Mesh(geometry, material);
  return ellipse;
}

export function drawThickBandFromPoints(
  points, thickness = 0.5, color = 0xffffff, opacity = 1, zOffset = 0,
) {
  const geometry = Line(points.map((p) => [p.x, p.y]));
  const material = new THREE.ShaderMaterial(BasicShader({
    side: THREE.DoubleSide,
    diffuse: color,
    thickness,
    opacity,
    transparent: true,
  }));
  const mesh = new THREE.Mesh(geometry, material);
  addOffsetZ(mesh, zOffset);
  return mesh;
}

export function drawSegmentsFromPoints(
  points, color = 0xff0000, linewidth = 1, zOffset = 0,
  matrixAutoUpdate = true, transparent = false, opacity = 1,
) {
  const path = new THREE.Path();
  const geometry = path.createGeometry(points);
  const material = new THREE.LineBasicMaterial({
    color,
    linewidth,
    transparent,
    opacity,
  });
  const pathLine = new THREE.Line(geometry, material);
  addOffsetZ(pathLine, zOffset);
  pathLine.matrixAutoUpdate = matrixAutoUpdate;
  if (matrixAutoUpdate === false) {
    pathLine.updateMatrix();
  }
  return pathLine;
}

export function drawSolidPolygonFace(
  color = 0xff0000, zOffset = 0,
  matrixAutoUpdate = true, transparent = true, opacity = 0.8,
) {
  const geometry = new THREE.PlaneGeometry(1, 1);
  const material = new THREE.MeshBasicMaterial({
    color,
    side: THREE.DoubleSide,
    transparent,
    opacity,
  });
  const rect = new THREE.Mesh(geometry, material);
  addOffsetZ(rect, zOffset);
  rect.matrixAutoUpdate = matrixAutoUpdate;
  if (matrixAutoUpdate === false) {
    rect.updateMatrix();
  }
  return rect;
}

function addOutlineToObject(object, objectGeometry, color, thickness = 1, opacity = 1) {
  const outline = new THREE.LineSegments(
    new THREE.EdgesGeometry(objectGeometry),
    new THREE.LineBasicMaterial({
      color,
      transparent: true,
      opacity,
      shadowSide: THREE.DoubleSide,
      depthTest: false,
      linewidth: thickness,
    }),
  );
  object.add(outline);
}

export function drawSolidBox(dimension, color, linewidth) {
  const geometry = new THREE.CubeGeometry(dimension.x, dimension.y, dimension.z);
  const material = new THREE.MeshBasicMaterial({
    color,
    transparent: true,
    opacity: 0.8,
  });
  const box = new THREE.Mesh(geometry, material);
  addOutlineToObject(box, geometry, color, linewidth);
  return box;
}

export function drawBox(dimension, color, linewidth) {
  const geometry = new THREE.CubeGeometry(dimension.x, dimension.y, dimension.z);
  const material = new THREE.MeshBasicMaterial({ color });
  const cube = new THREE.Mesh(geometry, material);
  const box = new THREE.BoxHelper(cube);
  box.material.linewidth = linewidth;
  return box;
}

export function drawDashedBox(dimension, color, linewidth, dashSize = 0.01, gapSize = 0.02) {
  let geometry = new THREE.CubeGeometry(dimension.x, dimension.y, dimension.z);
  geometry = new THREE.EdgesGeometry(geometry);
  geometry = new THREE.Geometry().fromBufferGeometry(geometry);
  geometry.computeLineDistances();
  const material = new THREE.LineDashedMaterial({
    color,
    linewidth,
    dashSize,
    gapSize,
  });
  const cube = new THREE.LineSegments(geometry, material);
  return cube;
}

export function drawArrow(length, linewidth, conelength, conewidth, color, thickBand = false) {
  const end = new THREE.Vector3(0, length, 0);
  const begin = new THREE.Vector3(0, 0, 0);
  const left = new THREE.Vector3(conewidth / 2, length - conelength, 0);
  const right = new THREE.Vector3(-conewidth / 2, length - conelength, 0);

  const arrow = (thickBand)
    ? drawThickBandFromPoints([begin, end, left, right, end], 0.3, color)
    : drawSegmentsFromPoints([begin, end, left, end, right], color, linewidth, 1);
  return arrow;
}

export function getShapeGeometryFromPoints(points, bezierCurve = false) {
  const shape = new THREE.Shape();
  if (bezierCurve) {
    shape.moveTo(points[0].x, points[0].y);
    for (let i = 1; i < points.length - 2; i += 1) {
      shape.bezierCurveTo(points[i].x, points[i].y,
        points[i + 1].x, points[i + 1].y,
        points[i + 2].x, points[i + 2].y);
    }
    shape.bezierCurveTo(_.takeRight(points, 2).concat(
      [{ x: points[0].x, y: points[0].y }],
    ));
    shape.bezierCurveTo(_.takeRight(points, 1).concat(
      [{ x: points[0].x, y: points[0].y },
        { x: points[1].x, y: points[1].y }],
    ));
  } else {
    shape.fromPoints(points);
  }
  return new THREE.ShapeGeometry(shape);
}

export function drawShapeFromPoints(points,
  material = new THREE.MeshBasicMaterial({ color: 0xff0000 }),
  bezierCurve = false, order = 0, matrixAutoUpdate = true) {
  const geometry = getShapeGeometryFromPoints(points, bezierCurve);
  const mesh = new THREE.Mesh(geometry, material);
  addOffsetZ(mesh, order);
  mesh.matrixAutoUpdate = matrixAutoUpdate;
  if (!matrixAutoUpdate) {
    mesh.updateMatrix();
  }
  return mesh;
}

export function disposeMeshGroup(mesh) {
  if (!mesh) {
    return;
  }

  mesh.traverse((child) => {
    if (child.geometry !== undefined) {
      child.geometry.dispose();
      child.material.dispose();
    }
  });
}

export function disposeMesh(mesh) {
  if (!mesh) {
    return;
  }

  mesh.geometry.dispose();
  mesh.material.dispose();
}

export function changeMaterial(mesh, color = 0xff0000, linewidth = 2,
  transparent = false, opacity = 1) {
  if (!mesh) {
    return;
  }
  mesh.material.dispose();
  mesh.material = new THREE.LineBasicMaterial({
    color,
    linewidth,
    transparent,
    opacity,
  });
}

export function drawRoutingPointArrow(origin, color, heading, length = 3) {
  const position = new THREE.Vector3(origin.x, origin.y, 0);
  const arrowMesh = drawArrow(length, 3, 0.5, 0.5, color, true);
  arrowMesh.rotation.set(0, 0, -Math.PI / 2);
  copyProperty(arrowMesh.position, position);
  arrowMesh.rotation.set(0, 0, -(Math.PI / 2 - heading));
  return arrowMesh;
}
