#!/usr/bin/env node

const fs = require('fs');
const path = require('path');

const typeMap = {
    CoilStatus: 'REG_COIL_STATUS',
    InputStatus: 'REG_INPUT_STATUS',
    InputRegister: 'REG_INPUT',
    HoldingRegister: 'REG_HOLDING',
};

const MODEL_MEGA = 0x01;
const MODEL_INVERTER = 0x02;

function modelToString(mega, inverter) {
    const flags = [];
    if (mega === '1') flags.push('MODEL_MEGA');
    if (inverter === '1') flags.push('MODEL_INVERTER');
    if (flags.length === 0) return '0';
    return flags.join(' | ');
}

function escapeString(str) {
    return str.replace(/\\/g, '\\\\').replace(/"/g, '\\"');
}

function convertCsvToHeader(inputFile, outputFile) {
    const content = fs.readFileSync(inputFile, 'utf8');
    const lines = content.split('\n');

    const output = [];
    output.push('// Auto-generated from CSV - do not edit manually');
    output.push(`// Generated: ${new Date().toISOString()}`);
    output.push('');

    let headerSkipped = false;
    let count = 0;

    for (const line of lines) {
        const trimmed = line.trim();
        if (!trimmed || trimmed.startsWith('#')) continue;
        if (!headerSkipped)
            if (trimmed.startsWith('name,')) {
                headerSkipped = true;
                continue;
            }
        const fields = parseCSVLine(trimmed);
        if (fields.length < 10) {
            console.error(`Skipping malformed line: ${trimmed}`);
            continue;
        }
        const [name, type, address, defacto, scale, mega, inverter, system, subsystem, ...descParts] = fields;
        const description = descParts.join(',');
        const regType = typeMap[type];
        if (!regType) {
            console.error(`Unknown type '${type}' for register '${name}'`);
            continue;
        }
        const modelStr = modelToString(mega, inverter);
        const cleanDesc = escapeString(description.replace(/^"|"$/g, '')); // Remove surrounding quotes
        output.push(`    {"${name}", ${regType}, ${address}, ${defacto}, ${scale}, ${modelStr}, "${system}", "${subsystem}", "${cleanDesc}"},`);
        count++;
    }
    fs.writeFileSync(outputFile, output.join('\n') + '\n');
    console.log(`Generated ${outputFile} with ${count} register definitions`);
}

function parseCSVLine(line) {
    const fields = [];
    let current = '';
    let inQuotes = false;
    for (let i = 0; i < line.length; i++) {
        const char = line[i];
        if (char === '"') inQuotes = !inQuotes;
        else if (char === ',' && !inQuotes) {
            fields.push(current);
            current = '';
        } else current += char;
    }
    fields.push(current);
    return fields;
}

const args = process.argv.slice(2);
if (args.length < 1) {
    console.log('Usage: node csv2header.js <input.csv> [output.h]');
    console.log('       Default output: common_thermia_registers.h');
    process.exit(1);
}
const inputFile = args[0];
const outputFile = args[1] || 'common_thermia_registers.h';
if (!fs.existsSync(inputFile)) {
    console.error(`Input file not found: ${inputFile}`);
    process.exit(1);
}
convertCsvToHeader(inputFile, outputFile);
