import express from 'express';
import path from 'path';
import fs from 'fs/promises';
import multer from 'multer';
import { png2svg } from 'svg-png-converter';
import { XMLBuilder, XMLParser } from 'fast-xml-parser';

import { MapNode } from '../../entity/map/MapNode';
import { MapEdge } from '../../entity/map/MapEdge';
import { Room } from '../../entity/map/Room';
import { Map } from '../../entity/map/Map';
import { Settings } from '../../entity/settings/Settings';
import { Task } from '../../entity/task/Task';
import { generateName } from '../../tools/name-generator';
import { execShellCommand } from '../../tools/shell';

const pythonDir = path.join(__dirname, '..', '..', '..', '..', 'scripts');
const python = path.join(pythonDir, 'venv', 'Scripts', 'python.exe');

export class MapController {

    static route(app: express.Application): void {

        app.route('/map')
            .post(multer().single('file'), async (req, res) => {
                const file = req.file;

                if (file == null) {
                    res.status(500).send();
                    return;
                }

                const pyScript = path.join(pythonDir, 'map_decomposition', 'map_decomposition.py');
                const image_path = path.join(pythonDir, 'map_decomposition', file.originalname);
                const json_output_path = path.join(pythonDir, 'map_decomposition', 'output.json');

                await fs.writeFile(image_path, file.buffer);

                const result = await png2svg({
                    optimize: true,
                    input: file.buffer,
                    color: 'white',
                });

                await Map.delete({});
                const map = new Map();
                map.svg = result.content;
                await map.save();

                const settings = await Settings.findOne();

                if (settings == null) {
                    res.status(500).send();
                    return;
                }

                const parameters = [
                    image_path,
                    json_output_path,
                    settings.meterPerPixel,
                    settings.discretizationDistance,
                    settings.doorSize,
                    settings.robotRadius,
                ];

                const cmd = python + ' ' + pyScript + ' ' + parameters.join(' ');
                await execShellCommand(cmd);

                let rawdata = '';

                try {
                    rawdata = await fs.readFile(json_output_path, 'utf8');
                } catch (err) {
                    console.error(err);
                    res.status(500).send();
                    return;
                }

                const map_json = JSON.parse(rawdata);
                const graph_json = map_json.graph;
                const rooms_json = map_json.rooms;

                // delete all nodes, edges and rooms
                await MapNode.delete({});
                await Task.delete({});

                // add nodes to db
                let nodes: MapNode[] = [];
                let i = 0;
                for (const node_json of graph_json) {
                    i += 1;
                    const node = MapNode.create({
                        x: node_json.x,
                        y: node_json.y,
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
                for (const node_json of graph_json) {
                    const node1 = nodes.find((n: MapNode) => n.x === node_json.x && n.y === node_json.y);

                    for (const neighbour_json of node_json.neighbours) {
                        const node2 = nodes.find((n: MapNode) => n.x === neighbour_json.x && n.y === neighbour_json.y);

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

                for (const room_json of rooms_json) {
                    const node = nodes.find((n: MapNode) => n.x === room_json.node.x && n.y === room_json.node.y);
                    const polygon = JSON.stringify(room_json.polygon);
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

                res.status(200).send();
            });

        app.route('/map/svg')
            .get(async (req, res) => {
                const map = await Map.findOne();

                if (map == null) {
                    res.status(500).send();
                    return;
                }

                const rooms = await Room.find();

                const settings = await Settings.findOne();

                if (settings == null) {
                    res.status(500).send();
                    return;
                }

                const meterPerPixel = settings.meterPerPixel;

                const parser = new XMLParser({
                    ignoreAttributes: false,
                    attributeNamePrefix: '@',
                });

                const builder = new XMLBuilder({
                    ignoreAttributes: false,
                    attributeNamePrefix: '@',
                });

                const mapSvg = parser.parse(map.svg);

                const width = mapSvg.svg['@width'];
                const height = mapSvg.svg['@height'];

                delete mapSvg.svg['@width'];
                delete mapSvg.svg['@height'];

                mapSvg.svg['@viewBox'] = `0 0 ${width} ${height}`;

                const polylines = [];

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
                const room = await Room.findById(req.params.ID);

                if (room == null) {
                    res.status(404).send();
                    return;
                }

                Room.update(room, req.body);

                res.status(200).send();
            });
    }

}
