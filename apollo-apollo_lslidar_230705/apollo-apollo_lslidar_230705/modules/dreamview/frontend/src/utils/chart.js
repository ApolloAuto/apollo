const defaultColors = [
  'rgba(241, 113, 112, 0.5)', // red
  'rgba(254, 208, 114, 0.5)', // yellow
  'rgba(162, 212, 113, 0.5)', // lime green
  'rgba(113, 226, 208, 0.5)', // turquoise
  'rgba(113, 208, 255, 0.5)', // aqua
  'rgba(179, 164, 238, 0.5)', // lavender
];

let currentIdx = 0;

function pickColor() {
  return defaultColors[currentIdx++ % defaultColors.length];
}

function isEqual(point1, point2) {
  if (!point1 || !point2) {
    return false;
  }
  return point1.x === point2.x && point1.y === point2.y;
}

function parseString(valueInString) {
  let value = null;
  try {
    value = JSON.parse(valueInString);
  } catch (error) {
    console.warn(`Failed to parse string ${valueInString}.`,
      'Set its value without parsing.');
    value = valueInString;
  }

  return value;
}

function parseDatasetProperties(propertiesInString) {
  const properties = {};

  for (const key in propertiesInString) {
    properties[key] = parseString(propertiesInString[key]);
  }

  // If color is not set, assign one.
  if (!properties.color) {
    properties.color = pickColor();
  }

  return properties;
}

function parseDataset(lines, polygons, cars) {
  const data = {};
  const properties = {};

  if (lines) {
    data.lines = {};
    properties.lines = {};
    lines.forEach((dataset) => {
      const label = JSON.stringify(dataset.label);
      properties.lines[label] = parseDatasetProperties(dataset.properties);
      properties.lines[label].hideLabelInLegend = dataset.hideLabelInLegend;
      data.lines[label] = dataset.point;
    });
  }

  if (polygons) {
    data.polygons = {};
    properties.polygons = {};
    polygons.forEach((dataset) => {
      const length = dataset.point.length;
      if (length === 0) {
        return;
      }

      const label = JSON.stringify(dataset.label);
      data.polygons[label] = dataset.point;
      if (!isEqual(dataset.point[0], dataset.point[length - 1])) {
        // close the loop by adding the first point to the end
        data.polygons[label].push(dataset.point[0]);
      }

      // There're default properties for polygons, no need to set it if not specify
      if (dataset.properties) {
        properties.polygons[label] = parseDatasetProperties(dataset.properties);
        properties.polygons[label].hideLabelInLegend = dataset.hideLabelInLegend;
      }
    });
  }

  if (cars) {
    data.cars = {};
    properties.cars = {};
    cars.forEach((dataset) => {
      const label = JSON.stringify(dataset.label);
      properties.cars[label] = {
        color: parseString(dataset.color),
        hideLabelInLegend: dataset.hideLabelInLegend,
      };
      data.cars[label] = {
        x: dataset.x,
        y: dataset.y,
        heading: dataset.heading,
      };
    });
  }

  return { data, properties };
}

function parseChartDataFromProtoBuf(protobuf) {
  const legendDisplay = typeof protobuf.options.legendDisplay === 'boolean'
    ? protobuf.options.legendDisplay
    : true;
  const options = {
    legend: {
      display: legendDisplay,
    },
    axes: {
      x: protobuf.options.x,
      y: protobuf.options.y,
    },
    syncXYWindowSize: protobuf.options.syncXyWindowSize,
    aspectRatio: protobuf.options.aspectRatio,
  };

  const { properties, data } = parseDataset(protobuf.line, protobuf.polygon, protobuf.car);

  return {
    title: protobuf.title,
    options,
    properties,
    data,
  };
}

export { parseChartDataFromProtoBuf };
