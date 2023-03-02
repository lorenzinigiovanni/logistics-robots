import express from 'express';
import path from 'path';
import fs from 'fs/promises';
import multer from 'multer';
import { png2svg } from 'svg-png-converter';
import { XMLBuilder, XMLParser } from 'fast-xml-parser';
import os from 'os';

import { MapNode } from '../../entity/map/MapNode';
import { MapEdge } from '../../entity/map/MapEdge';
import { Room } from '../../entity/map/Room';
import { Map } from '../../entity/map/Map';
import { Settings } from '../../entity/settings/Settings';
import { Task } from '../../entity/task/Task';
import { generateName } from '../../tools/name-generator';
import { execShellCommand } from '../../tools/shell';
import { Plan } from '../../entity/task/Plan';
import { Robot } from '../../entity/robot/Robot';

const pythonDir = path.join(__dirname, '..', '..', '..', 'src', 'scripts');

export class MapController {

    static route(app: express.Application): void {

        app.route('/map')
            .post(multer().single('file'), async (req, res) => {
                const file = req.file;

                if (file == null) {
                    res.status(500).send();
                    return;
                }

                let python = '';
                if (os.type() === 'Windows_NT') {
                    python = path.join(pythonDir, 'venv', 'Scripts', 'python.exe');
                } else {
                    python = path.join(pythonDir, 'venv', 'bin', 'python');
                }

                const pyScript = path.join(pythonDir, 'map_decomposition', 'map_decomposition.py');
                const imagePath = path.join(pythonDir, 'map_decomposition', file.originalname);
                const jsonOutputPath = path.join(pythonDir, 'map_decomposition', 'output.json');

                await fs.writeFile(imagePath, file.buffer);

                const result = await png2svg({
                    optimize: true,
                    input: file.buffer,
                    color: 'white',
                });

                await Map.delete({});
                const map = new Map();
                map.svg = result.content;
                await map.save();

                const [settings] = await Settings.find();

                if (settings == null) {
                    res.status(500).send();
                    return;
                }

                const parameters = [
                    imagePath,
                    jsonOutputPath,
                    settings.meterPerPixel,
                    settings.discretizationDistance,
                    settings.doorSize,
                    settings.robotRadius,
                ];

                const cmd = python + ' ' + pyScript + ' ' + parameters.join(' ');
                try {
                    await execShellCommand(cmd);
                } catch (err) {
                    console.error(err);
                    res.status(500).send();
                    return;
                }

                let rawdata = '';

                try {
                    rawdata = await fs.readFile(jsonOutputPath, 'utf8');
                } catch (err) {
                    console.error(err);
                    res.status(500).send();
                    return;
                }

                const mapJson = JSON.parse(rawdata);
                const graphJson = mapJson.graph;
                const roomsJson = mapJson.rooms;

                // delete all nodes, edges and rooms
                await MapNode.delete({});
                await Room.delete({});
                await Task.delete({});

                // add nodes to db
                let nodes: MapNode[] = [];
                let i = 0;
                for (const nodeJson of graphJson) {
                    i += 1;
                    const node = MapNode.create({
                        x: nodeJson.x,
                        y: nodeJson.y,
                        value: i,
                    });
                    nodes.push(node);
                }

                await MapNode
                    .createQueryBuilder()
                    .insert()
                    .into(MapNode)
                    .values(nodes)
                    .execute();

                nodes = await MapNode.find();

                // add edges to db
                const edges: MapEdge[] = [];
                for (const nodeJson of graphJson) {
                    const node1 = nodes.find((n: MapNode) => n.x === nodeJson.x && n.y === nodeJson.y);

                    for (const neighbourJson of nodeJson.neighbours) {
                        const node2 = nodes.find((n: MapNode) => n.x === neighbourJson.x && n.y === neighbourJson.y);

                        const edge = MapEdge.create({
                            node1: node1,
                            node2: node2,
                        });
                        edges.push(edge);
                    }
                }

                await MapEdge
                    .createQueryBuilder()
                    .insert()
                    .into(MapEdge)
                    .values(edges)
                    .execute();

                // add rooms to db
                const rooms: Room[] = [];

                for (const roomJson of roomsJson) {
                    const node = nodes.find((n: MapNode) => n.x === roomJson.node.x && n.y === roomJson.node.y);
                    const polygon = JSON.stringify(roomJson.polygon);
                    const room = Room.create({
                        node: node,
                        polygon: polygon,
                        name: generateName(),
                    });
                    rooms.push(room);
                }

                await Room
                    .createQueryBuilder()
                    .insert()
                    .into(Room)
                    .values(rooms)
                    .execute();

                await fs.unlink(imagePath);
                await fs.unlink(jsonOutputPath);

                res.status(200).send();
            });

        app.route('/map/svgmap')
            .get(async (req, res) => {
                const [settings] = await Settings.find();

                if (settings == null) {
                    res.status(500).send();
                    return;
                }

                const meterPerPixel = settings.meterPerPixel;

                // map

                const [map] = await Map.find();

                if (map == null) {
                    res.status(500).send();
                    return;
                }

                const parser = new XMLParser({
                    ignoreAttributes: false,
                    attributeNamePrefix: '@',
                });

                const mapSvg = parser.parse(map.svg);

                const width = mapSvg.svg['@width'];
                const height = mapSvg.svg['@height'];

                delete mapSvg.svg['@width'];
                delete mapSvg.svg['@height'];

                mapSvg.svg['@viewBox'] = `0 0 ${width} ${height}`;

                // rooms

                const polylines = [];

                const rooms = await Room.find();

                for (const room of rooms) {
                    const polygon = JSON.parse(room.polygon);
                    const newPolygon = [];
                    for (const point of polygon) {
                        newPolygon.push(point.x / meterPerPixel + ',' + point.y / meterPerPixel);
                    }

                    polylines.push({
                        '@points': newPolygon.join(' '),
                        '@stroke-width': 0,
                        '@fill': '#3366ff',
                        '@id': `${room.ID}`,
                    });
                }

                mapSvg.svg.polyline = polylines;

                const builder = new XMLBuilder({
                    ignoreAttributes: false,
                    attributeNamePrefix: '@',
                });

                const svg = builder.build(mapSvg);

                res.status(200).send(svg);
            });

        app.route('/map/svgrobots')
            .get(async (req, res) => {
                const [settings] = await Settings.find();

                if (settings == null) {
                    res.status(500).send();
                    return;
                }

                const meterPerPixel = settings.meterPerPixel;

                const [map] = await Map.find();

                if (map == null) {
                    res.status(500).send();
                    return;
                }

                const parser = new XMLParser({
                    ignoreAttributes: false,
                    attributeNamePrefix: '@',
                });

                const mapSvg = parser.parse(map.svg);

                const width = mapSvg.svg['@width'];
                const height = mapSvg.svg['@height'];

                const robotSvg: any = {};
                robotSvg.svg = {};
                robotSvg.svg['@xmlns'] = 'http://www.w3.org/2000/svg';
                robotSvg.svg['@viewBox'] = `0 0 ${width} ${height}`;

                // plans

                const polylines = [];

                const plans = await Plan.createQueryBuilder('plan')
                    .innerJoinAndSelect('plan.robot', 'robot')
                    .innerJoinAndSelect('plan.planToNodes', 'planToNodes')
                    .innerJoinAndSelect('planToNodes.node', 'node')
                    .orderBy({ 'planToNodes.order': 'ASC' })
                    .getMany();

                for (const plan of plans) {
                    if (plan.planToNodes) {
                        plan.nodes = plan.planToNodes.map(planToNode => planToNode.node);
                        delete plan.planToNodes;
                    }
                }

                for (const plan of plans) {
                    if (plan.nodes && plan.robot) {
                        const points = plan.nodes.map(node => node.x / meterPerPixel + ',' + (height - node.y / meterPerPixel));
                        polylines.push({
                            '@points': points.join(' '),
                            '@stroke-width': 8,
                            '@stroke': robotColor(plan.robot.number, await Robot.count()),
                            '@fill': 'none',
                        });
                    }
                }

                robotSvg.svg.polyline = polylines;

                // robots

                const robots = await Robot.find();
                const icons = [];
                const size = 100;

                for (const robot of robots) {
                    const position = [robot.currentX, robot.currentY];
                    icons.push({
                        '@x': position[0] / meterPerPixel - size / 2,
                        '@y': height - position[1] / meterPerPixel - size / 2,
                        '@width': size,
                        '@height': size,
                        '@xmlns': 'http://www.w3.org/2000/svg',
                        '@viewBox': '0 0 32 32',
                        path: [
                            {
                                '@fill': robotColor(robot.number, await Robot.count(), 35),
                                '@d': 'M11.27 31a2.11 2.11 0 0 1-2.37-1.92c-.16-4-.42-16.18.58-24.55a4 4 0 0 1 4-3.53h4.93a8 8 0 0 1 8 7.73c.16 5.88.08 13.89-.25 19.4a2 2 0 0 1-2 1.87h-1.89A2.07 2.07 0 0 1 20 28v-4h-4v4a3 3 0 0 1-3 3Z',
                            },
                            {
                                '@fill': robotColor(robot.number, await Robot.count(), 35),
                                '@d': 'M9.13 8v.11C9 9.27 9 10.47 8.91 11.72c-.15 3.11-.2 8.54-.15 12.26H5.84a2 2 0 0 1-2-1.87 77.13 77.13 0 0 1 .06-9.76c.09-1 .21-1.88.37-2.73A2 2 0 0 1 6.24 8h2.85Z',
                            },
                            {
                                '@fill': robotColor(robot.number, await Robot.count()),
                                '@d': 'M9.13 8v.11C9 9.27 9 10.47 8.91 11.72V12h-2.8a2 2 0 0 0-2 1.64c-.16.85-.28 3.77-.37 4.73v.66c0-1.62 0-5.19.18-6.66.09-1 .21-1.88.37-2.73A2 2 0 0 1 6.24 8h2.85zm12.03 13H15.8c-1.8 0-3.39-3.17-3.89-4.89a34.61 34.61 0 0 1-1.05-13.34 1 1 0 0 1 .93-.77c1.67-.66 4.31-.64 7.13-.48a6.29 6.29 0 0 1 6.69 6.62l-.4 7.08c-.12 2.1-1.89 5.78-4.05 5.78z',
                            },
                            {
                                '@fill': '#4a646d',
                                '@d': 'M28.48 10.9a3.76 3.76 0 0 1-.8 2 3.62 3.62 0 0 1-2.58 1.35l-6.2.44h-.26a3.65 3.65 0 0 1-3.64-3.6V9a2 2 0 0 1 2-2h7.84a3.67 3.67 0 0 1 3.61 3.12 4.07 4.07 0 0 1 .03.78Z',
                            },
                            {
                                '@fill': '#93c8da',
                                '@d': 'M28.48 10.9a3.42 3.42 0 0 1-1.38.38l-6.2.44h-.26A3.65 3.65 0 0 1 17 8.09V7h7.84a3.67 3.67 0 0 1 3.61 3.12 4.07 4.07 0 0 1 .03.78Z',
                            },
                            {
                                '@d': 'M29.44 10A4.67 4.67 0 0 0 27 6.57 9.08 9.08 0 0 0 18.37 0h-4.93a5 5 0 0 0-4.95 4.41c-.11.86-.19 1.74-.27 2.59A1.43 1.43 0 0 0 8 7H6.24a3 3 0 0 0-3 2.46 65.54 65.54 0 0 0-.4 12.74 3 3 0 0 0 3 2.8h1.94c0 1.69.08 3.1.12 4.12A3.09 3.09 0 0 0 11.27 32H13a4 4 0 0 0 4-4v-3h2v3a3.08 3.08 0 0 0 3.27 3h1.85a3 3 0 0 0 3-2.81c.22-3.75.33-9.69.33-13.73a4.79 4.79 0 0 0 1-.9 4.7 4.7 0 0 0 .99-3.56ZM5.84 23a1 1 0 0 1-1-.94 62.15 62.15 0 0 1 .41-12.25 1 1 0 0 1 1-.81h1.81c-.29 4.22-.34 10.5-.31 14Zm19.28 5.07a1 1 0 0 1-1 .93h-1.85c-.76 0-1.27-.4-1.27-1v-3a1 1 0 0 0 0-2h-4a2 2 0 0 0-2 2v3a2 2 0 0 1-2 2h-1.73c-.82 0-1.35-.37-1.37-1-.16-3.95-.43-16 .57-24.39a3 3 0 0 1 3-2.65h4.93a7 7 0 0 1 6.3 4H17a3 3 0 0 0-3 3v2.09a4.64 4.64 0 0 0 4.64 4.64H19l6.2-.44h.27c-.04 3.75-.14 9.35-.35 12.82Zm1.78-15.76a2.63 2.63 0 0 1-1.87 1l-6.2.44a2.57 2.57 0 0 1-2-.7 2.61 2.61 0 0 1-.83-1.96V9a1 1 0 0 1 1-1h7.84a2.65 2.65 0 0 1 2.06 4.31Z',
                            },
                        ],
                        rect: [
                            {
                                '@width': '4.44',
                                '@height': '1.6',
                                '@x': '21.55',
                                '@y': '8.57',
                                '@fill': '#fff',
                                '@rx': '0.8',
                            },
                        ],
                    });
                }

                robotSvg.svg.svg = icons;

                const builder = new XMLBuilder({
                    ignoreAttributes: false,
                    attributeNamePrefix: '@',
                });

                const svg = builder.build(robotSvg);

                res.status(200).send(svg);
            });

        app.route('/map/rooms')
            .get(async (req, res) => {
                const rooms = await Room.createQueryBuilder('room')
                    .select(['room.ID', 'room.name'])
                    .getMany();

                res.status(200).send(rooms);
            });

        app.route('/map/rooms/:ID')
            .put(async (req, res) => {
                const room = await Room.findOneBy({ ID: req.params.ID });

                if (room == null) {
                    res.status(404).send();
                    return;
                }

                Room.update(room.ID, req.body);

                res.status(200).send();
            });

        app.route('/map/jsonforbenchmarks')
            .get(async (req, res) => {
                const [settings] = await Settings.find();

                if (settings == null) {
                    throw new Error('Settings not found');
                }

                const nodes = await MapNode.createQueryBuilder('node')
                    .orderBy('node.value', 'ASC')
                    .getMany();

                const edges = await MapEdge
                    .createQueryBuilder('edge')
                    .leftJoin('edge.node1', 'node1')
                    .addSelect(['node1.ID', 'node1.value'])
                    .leftJoin('edge.node2', 'node2')
                    .addSelect(['node2.ID', 'node2.value'])
                    .getMany();

                const rooms = await Room
                    .createQueryBuilder('room')
                    .leftJoinAndSelect('room.node', 'node')
                    .getMany();

                // generate nodes list
                const nodeList = Array.from(Array(nodes.length), () => Array(4).fill(0));
                for (let i = 0; i < nodes.length; i++) {
                    // value (starts from 1), ID (starts from 0), x, y
                    nodeList[i] = [nodes[i].value, nodes[i].value - 1, nodes[i].x, nodes[i].y];
                }

                // generate rooms list
                const roomList = Array.from(Array(rooms.length), () => Array(4).fill(0));
                for (let i = 0; i < rooms.length; i++) {
                    // value (starts from 1), ID (starts from 0), x, y
                    roomList[i] = [rooms[i].node.value, rooms[i].node.value - 1, rooms[i].node.x, rooms[i].node.y];
                }

                // generate adjacency list
                const adjacencyList = Array.from(Array(nodes.length), () => Array(nodes.length).fill(0));
                for (let i = 0; i < nodes.length; i++) {
                    adjacencyList[i][i] = 1;

                    const node = nodes.find((n: MapNode) => n.value === i + 1);

                    if (node == null) {
                        throw new Error('Node not found');
                    }

                    const filteredEdges = edges.filter((e: MapEdge) => e.node1.ID === node.ID);

                    for (const edge of filteredEdges) {
                        const j = edge.node2.value - 1;

                        adjacencyList[i][j] = 1;
                        adjacencyList[j][i] = 1;
                    }
                }

                const jsonList = {
                    'rooms': roomList,
                    'nodes': nodeList,
                    'connect': adjacencyList,
                    'MAPF': settings.MAPFalgorithm,
                    'SAPF': settings.SAPFalgorithm,
                    'costFunction': settings.costFunction,
                    'heuristic': settings.heuristic,
                };
                const json = JSON.stringify(jsonList);

                const inputFilePath = path.join(__dirname, '..', '..', '..', '..', 'benchmarks', 'template.json');
                await fs.writeFile(inputFilePath, json, 'utf8');

                res.status(200).send();
            });
    }

}

function robotColor(index: number, robotCount: number, l = 50): string {
    const hue = index / robotCount * 360;
    return `hsl(${hue}, 100%, ${l}%)`;
}
