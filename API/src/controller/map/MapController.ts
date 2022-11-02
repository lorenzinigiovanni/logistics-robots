import express from 'express';
import { exec } from 'child_process';
import path from 'path';
import fs from 'fs/promises';
import multer from 'multer';

import { MapNode } from '../../entity/map/MapNode';
import { MapEdge } from '../../entity/map/MapEdge';
import { Room } from '../../entity/map/Room';
import { Settings } from '../../entity/settings/Settings';
import { Task } from '../../entity/task/Task';

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
                let edges: MapEdge[] = [];
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
                    });
                    rooms.push(room);
                }

                await Room
                    .createQueryBuilder()
                    .insert()
                    .into(Room)
                    .values(rooms)
                    .execute();

                // generate adjacency list
                edges = await MapEdge
                    .createQueryBuilder('edge')
                    .leftJoin('edge.node1', 'node1')
                    .addSelect(['node1.ID', 'node1.value'])
                    .leftJoin('edge.node2', 'node2')
                    .addSelect(['node2.ID', 'node2.value'])
                    .getMany();

                const node_number = nodes.length;
                const adjacency_list = Array.from(Array(node_number), _ => Array(node_number).fill(0));

                for (i = 0; i < node_number; i++) {
                    adjacency_list[i][i] = 1;

                    const node = nodes.find((n: MapNode) => n.value === i + 1);
                    if (node) {
                        const filteredEdges = edges.filter((e: MapEdge) => e.node1.ID === node.ID);

                        for (const edge of filteredEdges) {
                            const j = edge.node2.value - 1;

                            adjacency_list[i][j] = 1;
                            adjacency_list[j][i] = 1;
                        }
                    }
                }

                // eslint-disable-next-line no-console
                console.log(adjacency_list);

                res.status(200).send();
            });

    }

}

/**
 * Executes a shell command and return it as a Promise.
 * @param cmd {string}
 * @return {Promise<string>}
 */
function execShellCommand(cmd: string): Promise<string> {
    return new Promise((resolve, reject) => {
        exec(cmd, (error, stdout, stderr) => {
            if (error) {
                console.warn(error);
            }
            resolve(stdout ? stdout : stderr);
        });
    });
}
