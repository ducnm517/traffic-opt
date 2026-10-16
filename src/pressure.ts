/**
 * Original PQ-MP paper by Liu et al
 * https://arxiv.org/pdf/2406.19305
 */

export interface VehMovement {
    id: number;
    queue: number;                                    
    baseSatFlow: number;                        
    downstreamIndices: number[];        
    turningRatios: number[];                
    conflictingCrosswalks: number[];
}

export interface PedCrosswalk {
    id: number;
    queue: number;                                    
    satFlow: number;                                
    downstreamCrosswalkIdx: number; 
    pedTurningRatio: number;                
}

export interface AdmissiblePhase {
    id: number;                                         
    activeVehMask: Uint8Array;            
    activePedMask: Uint8Array;            
}

export function selectPQMPPhase(
    vehMovements: VehMovement[],
    pedCrosswalks: PedCrosswalk[],
    phases: AdmissiblePhase[],
    lambda: number = 0.006                 
): number {
    const numVeh = vehMovements.length;
    const numPed = pedCrosswalks.length;


    const vehWeights = new Float32Array(numVeh);
    for (let m = 0; m < numVeh; m++) {
        const mov = vehMovements[m];
        let downstreamExpected = 0.0;
        for (let k = 0; k < mov.downstreamIndices.length; k++) {
            const downIdx = mov.downstreamIndices[k];
            downstreamExpected += vehMovements[downIdx].queue * mov.turningRatios[k];
        }
        vehWeights[m] = mov.queue - downstreamExpected;
    }


    const pedWeights = new Float32Array(numPed);
    for (let c = 0; c < numPed; c++) {
        const cw = pedCrosswalks[c];
        const downQueue = cw.downstreamCrosswalkIdx >= 0
            ? pedCrosswalks[cw.downstreamCrosswalkIdx].queue
            : 0.0;
        pedWeights[c] = cw.queue - downQueue * cw.pedTurningRatio;
    }


    let bestPhaseId = phases[0].id;
    let maxPressure = -Infinity;

    for (let p = 0; p < phases.length; p++) {
        const phase = phases[p];
        let phasePressure = 0.0;
 
        for (let m = 0; m < numVeh; m++) {
            if (phase.activeVehMask[m] === 0) continue;
            const mov = vehMovements[m];

            let maxPedClearanceRatio = 0.0;
            for (let cIdx = 0; cIdx < mov.conflictingCrosswalks.length; cIdx++) {
                const cwId = mov.conflictingCrosswalks[cIdx];
                if (phase.activePedMask[cwId] === 1) {
                    const cw = pedCrosswalks[cwId];
                    const ratio = cw.queue / Math.max(cw.satFlow, 1e-5);
                    if (ratio > maxPedClearanceRatio) maxPedClearanceRatio = ratio;
                }
            }

            const effectiveSatFlow = mov.baseSatFlow * (1.0 - Math.min(1.0, maxPedClearanceRatio));
            phasePressure += vehWeights[m] * effectiveSatFlow;
        }

        // Pedestrian contribution weighted by lambda
        for (let c = 0; c < numPed; c++) {
            if (phase.activePedMask[c] === 0) continue;
            phasePressure += lambda * pedWeights[c] * pedCrosswalks[c].satFlow;
        }

        if (phasePressure > maxPressure) {
            maxPressure = phasePressure;
            bestPhaseId = phase.id;
        }
    }

    return bestPhaseId;
}