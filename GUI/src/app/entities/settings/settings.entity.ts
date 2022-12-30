import { Expose } from "class-transformer";

export class Settings {

    @Expose()
    ID!: string;

    @Expose()
    MAPFalgorithm!: string;

    @Expose()
    SAPFalgorithm!: string;

    @Expose()
    costFunction!: string;

    @Expose()
    heuristic!: string;

    @Expose()
    robotRadius!: number;

    @Expose()
    discretizationDistance!: number;

    @Expose()
    doorSize!: number;

    @Expose()
    meterPerPixel!: number;

}
