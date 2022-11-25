import { MapNode } from '../entity/map/MapNode';

export function manhattanDistanceFromNodes(n1: MapNode, n2: MapNode): number {
    return manhattanDistance(n1.x, n1.y, n2.x, n2.y);
}

export function euclideanDistanceFromNodes(n1: MapNode, n2: MapNode): number {
    return euclideanDistance(n1.x, n1.y, n2.x, n2.y);
}

export function manhattanDistance(x1: number, y1: number, x2: number, y2: number): number {
    return Math.abs(x1 - x2) + Math.abs(y1 - y2);
}

export function euclideanDistance(x1: number, y1: number, x2: number, y2: number): number {
    return Math.sqrt((x1 - x2) ** 2 + (y1 - y2) ** 2);
}
