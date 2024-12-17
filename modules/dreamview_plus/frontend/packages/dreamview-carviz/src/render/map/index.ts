import * as THREE from 'three';
import Lane from './lane';
import Junction from './junction';
import Grid from './grid';
import ClearArea from './clearArea';
import Crosswalk from './crosswalk';
import PncJunction from './pncJunction';
import Road from './road';
import YieldSignal from './yieldSignal';
import TrafficSignal from './trafficSignal';
import StopSign from './stopSign';
import SpeedBump from './speedBump';
import ParkingSpace from './parkingSpace';
import Area from './area';
import BarrierGate from './barrierGate';

class Map {
    private scene;

    public trafficSignal;

    public stopSign;

    public yieldSignal;

    public clearArea;

    public crosswalk;

    public lane;

    private text;

    public junction;

    public pncJunction;

    public road;

    public speedBump;

    public parkingSpace;

    public grid;

    public area;

    public barrierGate;

    private option;

    private coordinates;

    private colors;

    constructor(scene, text, option, coordinates, colors) {
        this.colors = colors;
        this.scene = scene;
        this.text = text;
        this.option = option;
        this.coordinates = coordinates;
        this.trafficSignal = new TrafficSignal(scene, coordinates);
        this.stopSign = new StopSign(scene, coordinates);
        this.yieldSignal = new YieldSignal(scene, coordinates, this.colors);
        this.clearArea = new ClearArea(scene, coordinates);
        this.crosswalk = new Crosswalk(scene, coordinates);
        this.lane = new Lane(scene, text, option, coordinates, this.colors);
        this.junction = new Junction(scene, coordinates);
        this.pncJunction = new PncJunction(scene, coordinates);
        this.road = new Road(scene, coordinates);
        this.speedBump = new SpeedBump(scene, coordinates);
        this.parkingSpace = new ParkingSpace(scene, text, option, coordinates);
        this.grid = new Grid(scene);
        this.area = new Area(scene, coordinates);
        this.barrierGate = new BarrierGate(scene, coordinates);
    }

    public update(mapData, removeOld = false) {
        if (removeOld) {
            this.dispose();
        }
        Object.keys(mapData).forEach((key) => {
            const data = mapData[key];
            const {
                crosswalk,
                clearArea,
                junction,
                pncJunction,
                lane,
                road,
                signal,
                stopSign,
                yieldSign,
                speedBump,
                parkingSpace,
                area,
                barrierGate,
            } = this.option.layerOption.Map;

            // 如果不是全清除，需要手动去依次清除
            if (!removeOld) {
                if (!mapData.lane || !lane) {
                    this.lane.dispose();
                }
                if (!mapData.junction || !junction) {
                    this.junction.dispose();
                }
                if (!mapData.crosswalk || !crosswalk) {
                    this.crosswalk.dispose();
                }
                if (!mapData.clearArea || !clearArea) {
                    this.clearArea.dispose();
                }
                if (!mapData.pncJunction || !pncJunction) {
                    this.pncJunction.dispose();
                }
                if (!mapData.road || !road) {
                    this.road.dispose();
                }
                if (!mapData.stopSign || !stopSign) {
                    this.stopSign.dispose();
                }

                if (!mapData.signal || !signal) {
                    this.trafficSignal.dispose();
                }
                if (!mapData.speedBump || !speedBump) {
                    this.speedBump.dispose();
                }

                if (!mapData.parkingSpace || !parkingSpace) {
                    this.parkingSpace.dispose();
                }
                if (!mapData.adArea || !area) {
                    this.area.dispose();
                }
                if (!mapData.barrierGate || !barrierGate) {
                    this.barrierGate.dispose();
                }
            }

            switch (key) {
                case 'lane':
                    if (lane) {
                        this.lane.drawLanes(data);
                    }
                    break;
                case 'junction':
                    if (junction) {
                        this.junction.drawJunctions(data);
                    }
                    break;
                case 'crosswalk':
                    if (crosswalk) {
                        this.crosswalk.drawCrosswalk(data);
                    }
                    break;
                case 'clearArea':
                    if (clearArea) {
                        this.clearArea.drawClearAreas(data);
                    }
                    break;
                case 'pncJunction':
                    if (pncJunction) {
                        this.pncJunction.drawPncJunctions(data);
                    }
                    break;
                case 'road':
                    if (road) {
                        this.road.drawRoads(data);
                    }
                    break;
                case 'yield':
                    if (yieldSign) {
                        this.yieldSignal.drawYieldSigns(data);
                    }
                    break;
                case 'signal':
                    if (signal) {
                        this.trafficSignal.drawTrafficSignals(data);
                    }
                    break;
                case 'stopSign':
                    if (stopSign) {
                        this.stopSign.drawStopSigns(data);
                    }
                    break;
                case 'speedBump':
                    if (speedBump) {
                        this.speedBump.drawSpeedBumps(data);
                    }
                    break;
                case 'parkingSpace':
                    if (parkingSpace) {
                        this.parkingSpace.drawParkingSpaces(data);
                    }
                    break;
                case 'adArea':
                    if (area) {
                        this.area.drawAreas(data);
                    }
                    break;
                case 'barrierGate':
                    if (barrierGate) {
                        this.barrierGate.drawBarrierGates(data);
                    }
                    break;
                default:
                    break;
            }
        });
        if (this.lane.currentLaneIds.length !== 0) {
            const { width, height, center } = this.lane;
            const size = Math.max(width, height);
            const position = { x: center.x, y: center.y, z: 0 };

            this.grid.drawGrid(
                {
                    size,
                    divisions: size / 5,
                    colorCenterLine: this.colors.gridColor,
                    colorGrid: this.colors.gridColor,
                },
                position,
            );
        }
    }

    public updateTrafficStatus(signals) {
        this.trafficSignal.updateTrafficStatus(signals);
    }

    public dispose() {
        this.trafficSignal.dispose();
        this.stopSign.dispose();
        this.yieldSignal.dispose();
        this.clearArea.dispose();
        this.crosswalk.dispose();
        this.lane.dispose();
        this.junction.dispose();
        this.pncJunction.dispose();
        this.parkingSpace.dispose();
        this.road.dispose();
        this.speedBump.dispose();
        this.grid.dispose();
        this.area.dispose();
        this.barrierGate.dispose();
    }
}
export default Map;
