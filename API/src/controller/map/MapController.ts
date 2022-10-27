import express from 'express';
import { exec } from 'child_process';
import path from 'path';
import fs from 'fs/promises';
import multer from 'multer';

import { MapNode } from '../../entity/map/MapNode';
import { MapEdge } from '../../entity/map/MapEdge';
import { Room } from '../../entity/map/Room';
import { Settings } from '../../entity/settings/Settings';

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

                // delete all nodes, edges and rooms
                await MapEdge.delete({});
                await Room.delete({});
                await MapNode.delete({});

                // add nodes to db
                let nodes: any = [];
                let i = 0;
                for (const node_json of graph_json) {
                    i += 1;
                    nodes.push({
                        x: node_json.x,
                        y: node_json.y,
                        value: i,
                    });
                }

                await MapNode
                    .createQueryBuilder()
                    .insert()
                    .into(MapNode)
                    .values(nodes)
                    .execute();

                nodes = await MapNode.find();

                // add edges to db
                const edges: any = [];
                for (const node_json of graph_json) {
                    const node1 = nodes.find((n: any) => n.x === node_json.x && n.y === node_json.y);

                    for (const neighbour_json of node_json.neighbours) {
                        const node2 = nodes.find((n: any) => n.x === neighbour_json.x && n.y === neighbour_json.y);

                        edges.push({
                            node1: node1,
                            node2: node2,
                        });
                    }
                }

                await MapEdge
                    .createQueryBuilder()
                    .insert()
                    .into(MapEdge)
                    .values(edges)
                    .execute();

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
