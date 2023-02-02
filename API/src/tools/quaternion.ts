export function quaternionToEuler(x: number, y: number, z: number, w: number): [number, number, number] {
    const sqw = w * w;
    const sqx = x * x;
    const sqy = y * y;
    const sqz = z * z;
    const yaw = Math.atan2(2.0 * (x * y + z * w), sqx - sqy - sqz + sqw);
    const pitch = Math.asin(-2.0 * (x * z - y * w));
    const roll = Math.atan2(2.0 * (y * z + x * w), -sqx - sqy + sqz + sqw);
    return [roll, pitch, yaw];
}

export function eulerToQuaternion(roll: number, pitch: number, yaw: number): [number, number, number, number] {
    const cr = Math.cos(roll / 2);
    const cp = Math.cos(pitch / 2);
    const cy = Math.cos(yaw / 2);
    const sr = Math.sin(roll / 2);
    const sp = Math.sin(pitch / 2);
    const sy = Math.sin(yaw / 2);

    const x = sr * cp * cy - cr * sp * sy;
    const y = cr * sp * cy + sr * cp * sy;
    const z = cr * cp * sy - sr * sp * cy;
    const w = cr * cp * cy + sr * sp * sy;

    return [x, y, z, w];
}
