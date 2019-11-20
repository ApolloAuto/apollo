import React from "react";
import ReactDOM from "react-dom";
import Chart from "chart.js";
import _ from "lodash";

import STORE from "store";

const defaultPolygonProperties = {
    color: 'rgba(255, 0, 0, 0.8)', // red
    borderWidth: 2,
    pointRadius: 0,
    fill: false,
    showLine: true,
    showText: true,
    cubicInterpolationMode: 'monotone',
    lineTension: 0,
};

Chart.plugins.register({
    afterDatasetsDraw: function(chart, easing) {
        const fontSize = 15;
        const fontStyle = 'normal';
        const fontFamily = 'Helvetica Neue';
        chart.config.data.datasets.forEach((dataset, index) => {
            if (dataset.showText) {
                const meta = chart.getDatasetMeta(index);
                const element = meta.data[Math.floor(meta.data.length / 2)];
                chart.ctx.font = Chart.helpers.fontString(fontSize, fontStyle, fontFamily);
                chart.ctx.fillStyle = dataset.borderColor;
                chart.ctx.textAlign = 'center';
                chart.ctx.textBaseline = 'middle';

                const padding = 1;
                const position = element.tooltipPosition();
                chart.ctx.fillText(dataset.text,
                    position.x, position.y - (fontSize / 2) - padding);
            } else if (dataset.specialMarker === 'car') {
                chart.ctx.save();

                const meta = chart.getDatasetMeta(index);
                const rotation = chart.data.datasets[index].data[0].heading || 0;

                const xAxis = chart.scales['x-axis-0'];
                const yAxis = chart.scales['y-axis-0'];
                const pixelPerUnit = {
                    x: xAxis.width / (xAxis.max - xAxis.min),
                    y: yAxis.height / (yAxis.max - yAxis.min),
                };
                const dx = Math.cos(rotation) > 0 ? 1 : -1;
                const dy = Math.tan(rotation) * dx;
                const xInPixels = dx * pixelPerUnit.x;
                const yInPixels = dy * pixelPerUnit.y;
                const rotationInPixels = Math.atan2(yInPixels, xInPixels);

                const element = meta.data[0];
                const position = element.tooltipPosition();
                chart.ctx.font = Chart.helpers.fontString(20, fontStyle, fontFamily);
                chart.ctx.translate(position.x, position.y);
                chart.ctx.rotate(-rotationInPixels); // ChartJS's rotation is clockwise
                chart.ctx.fillStyle = dataset.borderColor;
                chart.ctx.fillText('\u27A1︎\uFE0E' /* ➡ */, 0, 0);

                chart.ctx.restore();
            }
        });
    },
});

Chart.defaults.global.defaultFontColor = '#FFFFFF';

function updateTickWindow(scale, windowSize, midValue) {
    const mid = midValue || Math.floor((scale.max + scale.min) / 2);
    scale.max = mid + windowSize / 2;
    scale.min = mid - windowSize / 2;
}

function syncXYWindowSize(scale) {
    function isValidValue(value) {
        return value !== null && value !== undefined && !isNaN(value) && isFinite(value);
    }
    function IDMatches(meta) {
        return scale.isHorizontal() ? meta.xAxisID === scale.id : meta.yAxisID === scale.id;
    }

    // calculate the range for both x and y
    const min = {
        x: null,
        y: null,
    };
    const max = {
        x: null,
        y: null,
    };
    const chart = scale.chart;
    const datasets = chart.data.datasets;
    Chart.helpers.each(datasets, function(dataset, datasetIndex) {
        const meta = chart.getDatasetMeta(datasetIndex);
        if (chart.isDatasetVisible(datasetIndex) && IDMatches(meta)) {
            Chart.helpers.each(dataset.data, function(rawValue, index) {
                if (!isValidValue(rawValue.x) ||
                    !isValidValue(rawValue.y) ||
                    meta.data[index].hidden) {
                    return;
                }

                if (min.x === null || rawValue.x < min.x) {
                    min.x = rawValue.x;
                }
                if (max.x === null || rawValue.x > max.x) {
                    max.x = rawValue.x;
                }
                if (min.y === null || rawValue.y < min.y) {
                    min.y = rawValue.y;
                }
                if (max.y === null || rawValue.y > max.y) {
                    max.y = rawValue.y;
                }
            });
        }
    });

    // set min/max based on the larger range
    if (isValidValue(min.x) && isValidValue(min.y) &&
        isValidValue(max.x) && isValidValue(max.y)) {
        const max_diff = Math.max(max.x - min.x, max.y - min.y);
        const mid = scale.isHorizontal()
                        ? Math.floor((max.x + min.x) / 2)
                        : Math.floor((max.y + min.y) / 2);
        scale.max = mid + max_diff / 2;
        scale.min = mid - max_diff / 2;
    }
}

export default class ScatterGraph extends React.Component {
    initializeCanvas(title, options) {
        this.name2idx = {};
        const chartOptions = {
            title: {
                display: (title && title.length > 0),
                text: title
            },
            legend: {
                display: options.legend.display,
                labels: {
                    filter: (legendItem, data) => {
                        const hideLabel = _.get(data,
                            `datasets[${legendItem.datasetIndex}].hideLabelInLegend`, false);

                        return !hideLabel;
                    }
                }
            },
            tooltips: {
                enable: true,
                mode: "nearest",
                intersect: false,
            },
            aspectRatio: options.aspectRatio,
        };

        if (options.axes) {
            if (!chartOptions.scales) {
                chartOptions.scales = {};
            }
            for (const axis in options.axes) {
                const name = axis + 'Axes';
                const setting = options.axes[axis];
                const axisOptions = {
                    id: `${axis}-axis-0`,
                    scaleLabel: {
                        display: !_.isEmpty(setting.labelString),
                        labelString: setting.labelString,
                    },
                    ticks: {
                        min: setting.min,
                        max: setting.max,
                        minRotation: 0,
                        maxRotation: 0,
                        stepSize: setting.stepSize,
                        // Overwrite chartjs tick formatter to keep maximum 4 decimals
                        // and avoid scientific notation
                        callback: function(tickValue, index, ticks) {
                            // If we have lots of ticks, don't use the ones
                            let delta = ticks.length > 3
                                ? ticks[2] - ticks[1]
                                : ticks[1] - ticks[0];

                            // If we have a number like 2.5 as the delta,
                            // figure out how many decimal places we need
                            if (Math.abs(delta) > 1) {
                                if (tickValue !== Math.floor(tickValue)) {
                                    // not an integer
                                    delta = tickValue - Math.floor(tickValue);
                                }
                            }

                            const logDelta = Math.log10(Math.abs(delta));
                            let tickString = '';

                            if (Math.abs(tickValue) >= 1e-4) {
                                let numDecimal = -1 * Math.floor(logDelta);
                                numDecimal = Math.max(Math.min(numDecimal, 4), 0);
                                tickString = tickValue.toFixed(numDecimal);
                            } else {
                                tickString = '0'; // never show decimal places for 0
                            }

                            return tickString;
                        },
                    },
                    gridLines: {
                        color: 'rgba(153, 153, 153, 0.5)',
                        zeroLineColor: 'rgba(153, 153, 153, 0.7)',
                    },
                };
                if (!chartOptions.scales[name]) {
                    chartOptions.scales[name] = [];
                }
                if (setting.windowSize) {
                    axisOptions.afterDataLimits = (scale) => {
                        updateTickWindow(scale, setting.windowSize, setting.midValue);
                    };
                } else if (options.syncXYWindowSize) {
                    axisOptions.afterDataLimits = syncXYWindowSize;
                }
                chartOptions.scales[name].push(axisOptions);
            }
        }

        const ctx = this.canvasElement.getContext('2d');
        this.chart = new Chart(ctx, { type: "scatter", options: chartOptions });
    }

    constructDataConfig(properties) {
        // basic properties
        const config = {
            label: null, // legend
            hideLabelInLegend: properties.hideLabelInLegend,
            showText: properties.showText,
            text: null, // text in the graph

            backgroundColor: properties.color,
            borderColor: properties.color,

            data: null
        };

        // additional properties
        for (const key in properties) {
            config[key] = properties[key];
        }

        return config;
    }

    updateData(idx, name, properties, data) {
        const datasets = this.chart.data.datasets;
        const config = this.constructDataConfig(properties);
        if (datasets[idx] === undefined) {
            datasets.push(config);
        } else if (datasets[idx].text !== name) {
            datasets[idx] = config;
        }

        datasets[idx].label = name;
        datasets[idx].text = name;
        datasets[idx].data = data;
    }

    updateCar(name, point, properties) {
        // draw heading arrow
        {
            const arrowName = name;
            if (this.name2idx[arrowName] === undefined) {
                this.name2idx[arrowName] = this.chart.data.datasets.length;
            }
            const idx = this.name2idx[arrowName];
            const arrowProperties = properties;
            arrowProperties.specialMarker = 'car';
            arrowProperties.borderWidth = 0;
            arrowProperties.pointRadius = 0;
            this.updateData(idx, arrowName, arrowProperties, [point]);
        }

        // draw ego-vehicle bounding box
        {
            const polygonName = name + '_car_bounding_box';
            if (this.name2idx[polygonName] === undefined) {
                this.name2idx[polygonName] = this.chart.data.datasets.length;
            }
            const idx2 = this.name2idx[polygonName];
            const polygon = STORE.hmi.calculateCarPolygonPoints(point.x, point.y, point.heading);
            const polygonProperties = {
                borderWidth: 1,
                pointRadius: 0,
                color: properties.color,
                showLine: true,
                fill: false,
                lineTension: 0,
                hideLabelInLegend: true,
            };
            this.updateData(idx2, polygonName, polygonProperties, polygon);
        }
    }

    updateChart(props) {
        if (!props.data || !props.properties) {
            return;
        }
        const datasets = props.data;

        // Draw cars
        for (const name in props.properties.cars) {
            const nameInString = JSON.stringify(name);
            const point = _.get(datasets, `cars[${nameInString}]`, {});
            const properties = _.get(props, `properties.cars[${nameInString}]`, {});
            this.updateCar(name, point, properties);
        }

        // Draw lines
        for (const name in props.properties.lines) {
            if (this.name2idx[name] === undefined) {
                this.name2idx[name] = this.chart.data.datasets.length;
            }
            const idx = this.name2idx[name];
            const nameInString = JSON.stringify(name);
            const properties = _.get(props, `properties.lines[${nameInString}]`, {});
            const points = _.get(datasets, `lines[${nameInString}]`, []);
            this.updateData(idx, name, properties, points);
        };

        // Draw polygons
        let idx = Object.keys(this.name2idx).length;
        if (datasets.polygons) {
            for (const name in datasets.polygons) {
                const nameInString = JSON.stringify(name);
                const points = _.get(datasets, `polygons[${nameInString}]`);
                if (!points) {
                    continue;
                }

                const properties =
                    _.get(props, `properties.polygons[${nameInString}]`, defaultPolygonProperties);

                this.updateData(idx, name, properties, points);
                idx++;
            }
        }

        // Remove un-used polygons data
        this.chart.data.datasets.splice(idx, this.chart.data.datasets.length - idx);

        // Update chart
        this.chart.update(0);
    }

    componentDidMount() {
        const { title, options } = this.props;
        this.initializeCanvas(title, options);
        this.updateChart(this.props);
    }

    componentWillUnmount() {
        this.chart.destroy();
    }

    componentWillReceiveProps(nextProps) {
        this.updateChart(nextProps);
    }

    render() {
        const { title, options, properties, data } = this.props;
        return (
            <div className="scatter-graph">
                <canvas ref = {(input) => {
                            this.canvasElement = input;
                        }}/>
            </div>
        );
    }
}

function generateScatterGraph(setting, lineDatasets, carDatasets, polygonsDatasets) {
    if (!lineDatasets) {
        console.error("Graph data not found:", setting.title);
        return null;
    }

    if (!setting || !setting.properties || !setting.options) {
        console.error("Graph setting not found:", setting.title);
        return null;
    }

    return (
        <ScatterGraph
            key={setting.title}
            title={setting.title}
            options={setting.options}
            properties={setting.properties}
            data={{lines: lineDatasets, cars: carDatasets, polygons: polygonsDatasets}} />
    );
}

export {
    generateScatterGraph
};
