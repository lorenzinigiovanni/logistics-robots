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

const pythonDir = path.join(__dirname, '..', '..', 'scripts');

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

        app.route('/map/svg')
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

                // plans

                const plans = await Plan.createQueryBuilder('plan')
                    .leftJoinAndSelect('plan.robot', 'robot')
                    .leftJoinAndSelect('plan.planToNodes', 'planToNodes')
                    .leftJoinAndSelect('planToNodes.node', 'node')
                    .orderBy({ 'planToNodes.order': 'ASC' })
                    .getMany();

                for (const plan of plans) {
                    if (plan.planToNodes) {
                        plan.nodes = plan.planToNodes.map(planToNode => planToNode.node);
                        delete plan.planToNodes;
                    }
                }

                for (const plan of plans) {
                    if (plan.nodes) {
                        const points = plan.nodes.map(node => node.x / meterPerPixel + ',' + (height - node.y / meterPerPixel));
                        polylines.push({
                            '@points': points.join(' '),
                            '@stroke-width': 8,
                            '@stroke': robotColor(plan.robot.number, await Robot.count()),
                            '@fill': 'none',
                        });
                    }
                }

                mapSvg.svg.polyline = polylines;

                const builder = new XMLBuilder({
                    ignoreAttributes: false,
                    attributeNamePrefix: '@',
                });

                const svg = builder.build(mapSvg);

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
    }

}

function robotColor(index: number, robotCount: number): string {
    const hue = index / robotCount * 360;
    return `hsl(${hue}, 100%, 50%)`;
}
