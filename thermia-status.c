// ------------------------------------------------------------------------------------------------------------------------
// thermia-status: what the Thermia is doing, at a glance - a status board, a line each: what it's running and what waits, the
// compressor, its services (heating, hot water, pool, the additional heater), alarms. A state first (in capitals when it's
// doing something), then the few values that matter; anything out of the ordinary added. -v: the detail under each.
// Read-only: modbus TCP, alongside anything else talking to it.
//
//   thermia-status [-v] [address [model]]      address: a name or an IP (default thermia.local); model: mega (default), inverter
//
// The pool's request: the pool return line sensor is faked (a resistance switched in by the pool's relay), so its reading
// is the request, not a temperature - under its set point less the hysteresis the pool asks for heat, at the set point not
// ------------------------------------------------------------------------------------------------------------------------

#include <netdb.h>
#include <stdarg.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/socket.h>
#include <time.h>

#include <arpa/inet.h>

#include "common_thermia.h"

// ------------------------------------------------------------------------------------------------------------------------
// ------------------------------------------------------------------------------------------------------------------------

#define NOT_FITTED 199.9 // an absent sensor reads 200.0
#define ALARMS_MAXIMUM 64
#define CURVE_POINTS 7 // the heat curve: supply set point by outdoor temperature

enum { DEMAND_DEFROST = 2, DEMAND_HOT_WATER = 3, DEMAND_HEATING = 4, DEMAND_POOL = 6, DEMAND_STANDBY = 98, DEMAND_NONE = 99, DEMAND_OFF = 100 };

typedef struct {
    bool ok;
    double value;
    int raw;
} reading_t;

typedef struct {
    reading_t control, gear, gears, requested, percent, rpm, ok, stopping, blocked, unable;
    reading_t feed_supply, feed_return, brine_in, brine_out, discharge, brine_pump, brine_speed, condenser_pump, condenser_speed, high, low, superheat;
} compressor_t;
typedef struct {
    reading_t enable, winter, pump, gear, gear_min, gear_max, supply, setpoint, integral, season_stop, room, room_setpoint, buffer, buffer_setpoint;
    reading_t curve_x[CURVE_POINTS], curve_y[CURVE_POINTS], curve_min, curve_max;
} heating_t;
typedef struct {
    reading_t enable, top, lower, weighted, start, stop, gear, gear_min, gear_max;
} water_t;
typedef struct {
    reading_t enable, regulation, valve, pump, external_stop, request, setpoint, hysteresis, charge, supply, mix, mix_min, mix_max, gear, gear_min, gear_max,
        shift;
} pool_t;
typedef struct {
    reading_t enable, active, demand, pool;
} extra_t;

typedef struct {
    reading_t running[3], queued[5], mode, comfort, outdoor, major, minor, micro;
    compressor_t compressor;
    heating_t heating;
    water_t water;
    pool_t pool;
    extra_t extra;
    reading_t hours_compressor, hours_water, hours_extra;
    reading_t alarm_sum;
    char alarm_classes[16];
    const char *alarms[ALARMS_MAXIMUM];
    int alarm_count;
} status_t;

static int g_failures = 0;

// ------------------------------------------------------------------------------------------------------------------------
// ------------------------------------------------------------------------------------------------------------------------

static reading_t read_int(const char *name) {
    reading_t r = {false, 0, 0};
    if (!thermia_register_supported(name))
        return r;
    int raw;
    if (!thermia_modbus_read_register_int(name, &raw)) {
        g_failures++;
        return r;
    }
    const int scale = thermia_register_scale(name);
    r.ok = true;
    r.raw = raw;
    r.value = scale > 1 ? (double)raw / scale : raw;
    return r;
}

static reading_t read_bit(const char *name) {
    reading_t r = {false, 0, 0};
    if (!thermia_register_supported(name))
        return r;
    bool value;
    if (!thermia_modbus_read_register_bit(name, &value)) {
        g_failures++;
        return r;
    }
    r.ok = true;
    r.raw = value ? 1 : 0;
    r.value = r.raw;
    return r;
}

// hours from a pair of registers (MSB * 65536 + LSB; the LSB unsigned)
static reading_t read_hours(const char *msb, const char *lsb) {
    const reading_t high = read_int(msb), low = read_int(lsb);
    reading_t r = {high.ok && low.ok, 0, 0};
    if (r.ok)
        r.value = (double)(high.raw & 0xFFFF) * 65536 + (low.raw & 0xFFFF);
    return r;
}

// every alarm the model has, those on: the classes by letter, the rest by description
static void read_alarms(status_t *s) {
    s->alarm_sum = read_bit("alarmHeatpumpSum");
    for (int i = 0; i < thermia_register_count(); i++) {
        const char *name = thermia_register_name(i);
        if (strncmp(name, "alarm", 5) != 0 || strcmp(name, "alarmHeatpumpSum") == 0 || !thermia_register_supported(name))
            continue;
        const reading_t alarm = read_bit(name);
        if (!alarm.ok || !alarm.raw)
            continue;
        if (strncmp(name, "alarmHeatpumpClass", 18) == 0) {
            const size_t length = strlen(s->alarm_classes);
            if (length < sizeof(s->alarm_classes) - 1)
                s->alarm_classes[length] = name[18];
        } else if (s->alarm_count < ALARMS_MAXIMUM)
            s->alarms[s->alarm_count++] = thermia_register_description(name);
    }
}

static void read_status(status_t *s) {
    memset(s, 0, sizeof(*s));
    s->running[0] = read_int("valueHeatpumpCurrentlyRunningFirstPrioritisedDemand");
    s->running[1] = read_int("valueHeatpumpCurrentlyRunningSecondPrioritisedDemand");
    s->running[2] = read_int("valueHeatpumpCurrentlyRunningThirdPrioritisedDemand");
    s->queued[0] = read_int("valueHeatpumpQueuedDemandFirstPriority");
    s->queued[1] = read_int("valueHeatpumpQueuedDemandSecondPriority");
    s->queued[2] = read_int("valueHeatpumpQueuedDemandThirdPriority");
    s->queued[3] = read_int("valueHeatpumpQueuedDemandFourthPriority");
    s->queued[4] = read_int("valueHeatpumpQueuedDemandFifthPriority");
    s->mode = read_int("setHeatpumpOperationalMode");
    s->comfort = read_int("valueHeatpumpComfortMode");
    s->outdoor = read_int("valueHeatpumpOutdoorTemperature");
    s->major = read_int("valueHeatpumpSoftwareVersionMajor");
    s->minor = read_int("valueHeatpumpSoftwareVersionMinor");
    s->micro = read_int("valueHeatpumpSoftwareVersionMicro");

    compressor_t *c = &s->compressor;
    c->control = read_bit("statusHeatpumpCompressorControl");
    c->gear = read_int("valueHeatpumpCompressorCurrentGear");
    c->gears = read_int("valueHeatpumpCompressorAvailableGears");
    c->requested = read_int("valueHeatpumpMaximumGearCurrentlyRequested");
    c->percent = read_int("valueHeatpumpCompressorSpeedPercent");
    c->rpm = read_int("valueHeatpumpCompressorSpeedRpm");
    c->ok = read_bit("statusHeatpumpOkToStart");
    c->stopping = read_bit("statusHeatpumpStopping");
    c->blocked = read_int("valueHeatpumpCompressorTemporarilyBlocked");
    c->unable = read_bit("statusHeatpumpCompressorUnableToSpeedUp");
    c->feed_supply = read_int("valueHeatpumpCondenserOutTemperature");
    c->feed_return = read_int("valueHeatpumpCondenserInTemperature");
    c->brine_in = read_int("valueHeatpumpBrineInTemperature");
    c->brine_out = read_int("valueHeatpumpBrineOutTemperature");
    c->discharge = read_int("valueHeatpumpDischargePipeTemperature");
    c->brine_pump = read_bit("statusHeatpumpBrinePumpOnOff");
    c->brine_speed = read_int("valueHeatpumpBrineCirculationPumpSpeed");
    c->condenser_pump = read_bit("statusHeatpumpCondenserPumpOnOff");
    c->condenser_speed = read_int("valueHeatpumpCondenserCirculationPumpSpeed");
    c->high = read_int("valueHeatpumpHighPressureSidePressure");
    c->low = read_int("valueHeatpumpLowPressureSidePressure");
    c->superheat = read_int("valueHeatpumpSuperheatTemperature");

    heating_t *h = &s->heating;
    h->enable = read_bit("enableHeatpumpHeat");
    h->winter = read_bit("statusHeatingSeasonWinterActive");
    h->pump = read_bit("statusHeatingSystemCirculationPumpControl");
    h->gear = read_int("valueHeatingDesiredGear");
    h->gear_min = read_int("setHeatpumpMinAllowedGearHeating");
    h->gear_max = read_int("setHeatpumpMaxAllowedGearHeating");
    h->supply = read_int("valueHeatingSystemSupplyLineTemperature");
    h->setpoint = read_int("valueHeatingSystemSupplyLineCalculatedSetPoint");
    h->integral = read_int("valueHeatingSeasonIntegralValue");
    h->season_stop = read_int("setHeatingSeasonStopTemperature");
    h->room = read_int("valueHeatingRoomTemperatureSensor");
    h->room_setpoint = read_int("setHeatingComfortWheelSetting");
    for (int i = 0; i < CURVE_POINTS; i++) {
        char name[64];
        snprintf(name, sizeof(name), "valueHeatingHeatCurveXCoordinate%d", i + 1);
        h->curve_x[i] = read_int(name);
        snprintf(name, sizeof(name), "setHeatingHeatCurveYCoordinate%d", i + 1);
        h->curve_y[i] = read_int(name);
    }
    h->curve_min = read_int("setHeatingMinLimitationSetPointCurveRadiator");
    h->curve_max = read_int("setHeatingMaxLimitationSetPointCurveRadiator");
    h->buffer = read_int("valueHeatingBufferTankTemperature");
    h->buffer_setpoint = read_int("valueHeatingBufferTankChargeSetPoint");

    water_t *w = &s->water;
    w->enable = read_bit("enableHeatpumpTapWater");
    w->top = read_int("valueTapwaterTopTemperature");
    w->lower = read_int("valueTapwaterLowerTemperature");
    w->weighted = read_int("valueTapwaterWeightedTemperature");
    w->start = read_int("setTapwaterStartTemperature");
    w->stop = read_int("setTapwaterStopTemperature");
    w->gear = read_int("valueTapwaterDesiredGear");
    w->gear_min = read_int("setHeatpumpMinAllowedGearTapWater");
    w->gear_max = read_int("setHeatpumpMaxAllowedGearTapWater");

    pool_t *p = &s->pool;
    p->enable = read_bit("enablePoolPool");
    p->regulation = read_bit("statusPoolRegulationControl");
    p->valve = read_bit("statusPoolDirectionalValvePosition");
    p->pump = read_bit("statusPoolCirculationPumpControl");
    p->external_stop = read_bit("statusPoolExternalStop");
    p->request = read_int("valuePoolReturnLineTemperature");
    p->setpoint = read_int("setPoolReturnTempFromPoolToHeatExchanger");
    p->hysteresis = read_int("setPoolHysteresis");
    p->charge = read_int("setPoolChargeSetPoint");
    p->supply = read_int("valuePoolSupplyLineTemperature");
    p->mix = read_int("valuePoolSupplyLineMixValvePosition");
    p->mix_min = read_int("setPoolMixValveLowestAllowedOpeningDegree");
    p->mix_max = read_int("setPoolMixValveHighestAllowedOpeningDegree");
    p->gear = read_int("valuePoolDesiredGear");
    p->gear_min = read_int("setHeatpumpMinAllowedGearPool");
    p->gear_max = read_int("setHeatpumpMaxAllowedGearPool");
    p->shift = read_int("setHeatpumpGearShiftDelayPool");

    extra_t *e = &s->extra;
    e->enable = read_bit("enableHeatpumpExternalAdditionalHeater");
    e->active = read_bit("statusHeatpumpExternalAdditionalHeaterActive");
    e->demand = read_int("valueHeatpumpExternalAdditionalHeaterCurrentDemand");
    e->pool = read_bit("enablePoolExternalAdditionalHeater");

    s->hours_compressor = read_hours("valueHeatpumpCompressorOperatingHoursMsb", "valueHeatpumpCompressorOperatingHoursLsb");
    s->hours_water = read_hours("valueTapwaterOperatingHoursMsb", "valueTapwaterOperatingHoursLsb");
    s->hours_extra = read_hours("valueHeatpumpExternalAdditionalHeaterOperatingHoursMsb", "valueHeatpumpExternalAdditionalHeaterOperatingHoursLsb");

    read_alarms(s);
}

// ------------------------------------------------------------------------------------------------------------------------
// ------------------------------------------------------------------------------------------------------------------------

// formatted text in a few rotating buffers: enough for one line's worth of arguments
static const char *fmt(const char *format, ...) {
    static char buffers[32][160];
    static int next = 0;
    char *buffer = buffers[next++ % 32];
    va_list args;
    va_start(args, format);
    vsnprintf(buffer, sizeof(buffers[0]), format, args);
    va_end(args);
    return buffer;
}

static bool on(reading_t r) { return r.ok && r.raw != 0; }
static bool off(reading_t r) { return r.ok && r.raw == 0; }
static bool fitted(reading_t r) { return r.ok && r.value < NOT_FITTED; }
static const char *temp(reading_t r) { return !r.ok ? "?" : r.value >= NOT_FITTED ? "-" : fmt("%.1f", r.value); }
static const char *number(reading_t r, const char *format) { return r.ok ? fmt(format, r.value) : "?"; }
static const char *onoff(reading_t r) { return !r.ok ? "?" : r.raw ? "on" : "off"; }
static const char *enabled(reading_t r) { return !r.ok ? "?" : r.raw ? "enabled" : "disabled"; }

// the protocol's demand codes (footnote *1), named as badtuna's journal names them
static const char *demand_name(int code) {
    switch (code) {
    case 1:
        return "manual";
    case DEMAND_DEFROST:
        return "defrost";
    case DEMAND_HOT_WATER:
        return "hot-water";
    case DEMAND_HEATING:
        return "heating";
    case 5:
        return "cool";
    case DEMAND_POOL:
        return "pool";
    case 7:
        return "anti-legionella";
    case DEMAND_STANDBY:
        return "standby";
    case DEMAND_NONE:
        return "none";
    case DEMAND_OFF:
        return "off";
    }
    return fmt("unknown (%d)", code);
}

static const char *upper(const char *text) {
    char *out = (char *)fmt("%s", text);
    for (char *c = out; *c; c++)
        if (*c >= 'a' && *c <= 'z')
            *c = (char)(*c - 'a' + 'A');
    return out;
}

// operational mode (footnote *2): on/auto is the usual
static const char *mode_name(reading_t r) {
    if (!r.ok)
        return "mode ?";
    switch (r.raw) {
    case 1:
        return "OFF";
    case 2:
        return "STANDBY";
    case 3:
        return "on/auto";
    }
    return fmt("MODE %d", r.raw);
}

// smart grid (footnote *4): normal is the usual, said only when it's not
static const char *comfort_name(reading_t r) {
    if (!r.ok || r.raw == 4)
        return NULL;
    switch (r.raw) {
    case 1:
        return "EVU (blocked by the grid)";
    case 5:
        return "COMFORT";
    case 6:
        return "BOOST";
    }
    return fmt("COMFORT MODE %d", r.raw);
}

static bool running(const status_t *s, int code) {
    for (int i = 0; i < 3; i++)
        if (s->running[i].ok && s->running[i].raw == code)
            return true;
    return false;
}
static bool queued(const status_t *s, int code) {
    for (int i = 0; i < 5; i++)
        if (s->queued[i].ok && s->queued[i].raw == code)
            return true;
    return false;
}

// a service's state: RUNNING (the Thermia's on it), WAITING (queued behind what it's on), idle, or disabled
static const char *service(const status_t *s, reading_t enable, int code) {
    return off(enable) ? "disabled" : running(s, code) ? "RUNNING" : queued(s, code) ? "WAITING" : "idle";
}

// ------------------------------------------------------------------------------------------------------------------------
// ------------------------------------------------------------------------------------------------------------------------

#define MORE "                      " // under a row's values

// a row: its label, its state, its values (the state padded only when values follow)
static void row(const char *label, const char *state, const char *values) {
    if (values != NULL && *values)
        printf("%-12s%-10s%s\n", label, state, values);
    else
        printf("%-12s%s\n", label, state);
}

// a line's values, joined: each added only if there's something to say
typedef struct {
    char text[512];
} line_t;
static void add(line_t *line, const char *text) {
    if (text != NULL && *text)
        snprintf(line->text + strlen(line->text), sizeof(line->text) - strlen(line->text), "%s%s", line->text[0] ? "  " : "", text);
}

static void show_header(const status_t *s, const char *address, const char *ip) {
    char now[16];
    const time_t t = time(NULL);
    strftime(now, sizeof(now), "%H:%M:%S", localtime(&t));
    line_t line = {""};
    add(&line, mode_name(s->mode));
    add(&line, comfort_name(s->comfort));
    add(&line, fmt("outdoor %s°C", temp(s->outdoor)));
    printf("thermia %s%s  %s  %s\n\n", address, strcmp(address, ip) != 0 ? fmt(" (%s)", ip) : "", now, line.text);
}

// what it's running (in capitals), and what waits behind it
static void show_now(const status_t *s) {
    line_t what = {""}, waiting = {""};
    for (int i = 0; i < 3; i++)
        if (i == 0 || (s->running[i].ok && s->running[i].raw != DEMAND_NONE))
            add(&what, !s->running[i].ok ? "?" : s->running[i].raw == DEMAND_NONE ? "idle" : upper(demand_name(s->running[i].raw)));
    for (int i = 0; i < 5; i++)
        if (s->queued[i].ok && s->queued[i].raw != DEMAND_NONE && !running(s, s->queued[i].raw))
            add(&waiting, demand_name(s->queued[i].raw));
    row("now", what.text, waiting.text[0] ? fmt("waiting: %s", waiting.text) : NULL);
}

static void show_compressor(const status_t *s, bool verbose) {
    const compressor_t *c = &s->compressor;
    const char *state = on(c->control) ? "RUNNING" : on(c->stopping) ? "STOPPING" : on(c->blocked) ? "BLOCKED" : off(c->ok) ? "NOT OK" : "off";
    line_t line = {""};
    if (on(c->control))
        add(&line, fmt("gear %s/%s  %s %s rpm", number(c->gear, "%.0f"), number(c->gears, "%.0f"), number(c->percent, "%.0f%%"), number(c->rpm, "%.0f")));
    add(&line, fmt("feed %s→%s°C", temp(c->feed_supply), temp(c->feed_return)));
    add(&line, fmt("brine %s→%s°C", temp(c->brine_in), temp(c->brine_out)));
    if (on(c->blocked) && on(c->control))
        add(&line, fmt("START RESTRICTION (%d)", c->blocked.raw));
    if (on(c->unable))
        add(&line, "UNABLE TO SPEED UP");
    row("compressor", state, line.text);
    if (verbose) {
        printf(MORE "ok to start %s, requested gear %s, discharge %s°C, superheat %s°C, pressure %s/%s bar\n",
               !c->ok.ok   ? "?"
               : c->ok.raw ? "yes"
                           : "no",
               number(c->requested, "%.0f"), temp(c->discharge), temp(c->superheat), number(c->high, "%.1f"), number(c->low, "%.1f"));
        printf(MORE "pumps: brine %s %s, condenser %s %s\n", onoff(c->brine_pump), number(c->brine_speed, "%.0f%%"), onoff(c->condenser_pump),
               number(c->condenser_speed, "%.0f%%"));
    }
}

// its two set points: the room's (the comfort wheel: the display's indoor temperature), measured only with a room sensor; and the
// supply's (the heat curve at the outdoor temperature, the wheel's shift in it), against the system supply sensor - or without one,
// the condenser out while heating is what the Thermia's on (otherwise that's the feed to another service)
static void show_heating(const status_t *s, bool verbose) {
    const heating_t *h = &s->heating;
    const bool heating = s->running[0].ok && s->running[0].raw == DEMAND_HEATING;
    const reading_t supply = fitted(h->supply) ? h->supply : heating ? s->compressor.feed_supply : (reading_t){false, 0, 0};
    line_t line = {""};
    add(&line, fitted(h->room) ? fmt("room %s°C (set %s)", temp(h->room), temp(h->room_setpoint)) : fmt("room set %s°C", temp(h->room_setpoint)));
    add(&line, supply.ok ? fmt("supply %s°C (set %s)", temp(supply), temp(h->setpoint)) : fmt("supply set %s°C", temp(h->setpoint)));
    add(&line, fmt("degree minutes %s", number(h->integral, "%.0f")));
    if (off(h->winter))
        add(&line, "summer");
    row("heating", service(s, h->enable, DEMAND_HEATING), line.text);
    if (verbose) {
        printf(MORE "circulation pump %s, desired gear %s (allowed %s-%s), %s, season stop over %s°C\n", onoff(h->pump), number(h->gear, "%.0f"),
               number(h->gear_min, "%.0f"), number(h->gear_max, "%.0f"),
               !h->winter.ok   ? "?"
               : h->winter.raw ? "winter"
                               : "summer",
               temp(h->season_stop));
        line_t curve = {""};
        for (int i = 0; i < CURVE_POINTS; i++)
            add(&curve, fmt("%s→%s", number(h->curve_x[i], "%g"), number(h->curve_y[i], "%g")));
        printf(MORE "heat curve (outdoor→supply) %s, limits %s-%s°C, room sensor %s\n", curve.text, number(h->curve_min, "%g"), number(h->curve_max, "%g"),
               fitted(h->room) ? "fitted" : "none");
        if (fitted(h->buffer))
            printf(MORE "buffer tank %s°C (charge to %s°C)\n", temp(h->buffer), temp(h->buffer_setpoint));
    }
}

static void show_water(const status_t *s, bool verbose) {
    const water_t *w = &s->water;
    row("hot water", service(s, w->enable, DEMAND_HOT_WATER), fmt("%s°C  (starts under %s, stops at %s)", temp(w->weighted), temp(w->start), temp(w->stop)));
    if (verbose)
        printf(MORE "top %s°C, lower %s°C, desired gear %s (allowed %s-%s)\n", temp(w->top), temp(w->lower), number(w->gear, "%.0f"),
               number(w->gear_min, "%.0f"), number(w->gear_max, "%.0f"));
}

// its request (the faked sensor): on under the set point less the hysteresis, off at the set point, held between
static void show_pool(const status_t *s, bool verbose) {
    const pool_t *p = &s->pool;
    line_t line = {""};
    if (p->request.ok && p->setpoint.ok && p->hysteresis.ok) {
        const double start = p->setpoint.value - p->hysteresis.value;
        add(&line, p->request.value < start                ? fmt("request on (%.1f < %.1f)", p->request.value, start)
                   : p->request.value >= p->setpoint.value ? fmt("request off (%.1f ≥ %.1f)", p->request.value, p->setpoint.value)
                                                           : fmt("request held (%.1f, %.1f-%.1f)", p->request.value, start, p->setpoint.value));
    } else
        add(&line, "request ?");
    if (running(s, DEMAND_POOL) && off(p->valve))
        add(&line, "VALVE AWAY");
    if (running(s, DEMAND_POOL) && off(p->pump))
        add(&line, "PUMP OFF");
    if (on(p->external_stop))
        add(&line, "EXTERNAL STOP");
    row("pool", service(s, p->enable, DEMAND_POOL), line.text);
    if (verbose) {
        printf(MORE "valve %s, circulation pump %s, regulation %s, external stop %s\n",
               !p->valve.ok   ? "?"
               : p->valve.raw ? "to the pool"
                              : "away",
               onoff(p->pump), onoff(p->regulation), onoff(p->external_stop));
        printf(MORE "desired gear %s (allowed %s-%s, shift delay %ss), charge set point %s°C, mix valve %s (allowed %s-%s)%s\n", number(p->gear, "%.0f"),
               number(p->gear_min, "%.0f"), number(p->gear_max, "%.0f"), number(p->shift, "%.0f"), temp(p->charge), number(p->mix, "%.0f%%"),
               number(p->mix_min, "%.0f"), number(p->mix_max, "%.0f"), fitted(p->supply) ? fmt(", supply %s°C", temp(p->supply)) : "");
    }
}

static void show_extra(const status_t *s, bool verbose) {
    const extra_t *e = &s->extra;
    row("extra heat", on(e->active) ? "ACTIVE" : off(e->enable) ? "disabled" : "idle", on(e->active) ? fmt("demand %s", number(e->demand, "%.0f%%")) : NULL);
    if (verbose)
        printf(MORE "external heater %s, for the pool %s\n", enabled(e->enable), enabled(e->pool));
}

// each alarm on, less its trailing " alarm"; ALARM when the sum is on, notice when only some (e.g. class C) are
static void show_alarms(const status_t *s, bool verbose) {
    line_t line = {""};
    for (int i = 0; i < s->alarm_count; i++) {
        const char *description = s->alarms[i];
        const size_t length = strlen(description);
        add(&line, length > 6 && strcmp(description + length - 6, " alarm") == 0 ? fmt("%.*s", (int)(length - 6), description) : description);
    }
    if (s->alarm_classes[0])
        add(&line, fmt("(class %s)", s->alarm_classes));
    row("alarms", on(s->alarm_sum) ? "ALARM" : s->alarm_count > 0 || s->alarm_classes[0] ? "notice" : "none", line.text);
    if (verbose) {
        printf("\n");
        row("hours",
            fmt("compressor %sh, hot water %sh, extra heat %sh", number(s->hours_compressor, "%.0f"), number(s->hours_water, "%.0f"),
                number(s->hours_extra, "%.0f")),
            NULL);
        row("software", fmt("%s.%s.%s", number(s->major, "%.0f"), number(s->minor, "%.0f"), number(s->micro, "%.0f")), NULL);
    }
}

// ------------------------------------------------------------------------------------------------------------------------
// ------------------------------------------------------------------------------------------------------------------------

static bool resolve(const char *host, char *ip, size_t size) {
    struct addrinfo *result;
    const int err = getaddrinfo(host, NULL, &(struct addrinfo){.ai_family = AF_INET, .ai_socktype = SOCK_STREAM}, &result);
    if (err != 0) {
        fprintf(stderr, "thermia-status: can't resolve '%s': %s\n", host, gai_strerror(err));
        return false;
    }
    const bool ok = inet_ntop(AF_INET, &((struct sockaddr_in *)result->ai_addr)->sin_addr, ip, size) != NULL;
    freeaddrinfo(result);
    return ok;
}

static int usage(const char *prog, int status) {
    printf("Usage: %s [-v] [address [model]]\n  -v       the detail under each line\n  address  a name or an IP (default thermia.local)\n  model    mega "
           "(default) or inverter\n",
           prog);
    return status;
}

int main(int argc, char *argv[]) {
    bool verbose = false;
    const char *args[2] = {NULL, NULL};
    int count = 0;
    for (int i = 1; i < argc; i++) {
        if (strcmp(argv[i], "-v") == 0)
            verbose = true;
        else if (strcmp(argv[i], "-h") == 0 || strcmp(argv[i], "--help") == 0)
            return usage(argv[0], 0);
        else if (argv[i][0] == '-' || count == 2)
            return usage(argv[0], 1);
        else
            args[count++] = argv[i];
    }
    const char *address = args[0] ? args[0] : "thermia.local", *model_name = args[1] ? args[1] : "mega";
    model_t model;
    if (strcmp(model_name, "mega") == 0)
        model = MODEL_MEGA;
    else if (strcmp(model_name, "inverter") == 0)
        model = MODEL_INVERTER;
    else {
        fprintf(stderr, "thermia-status: model unknown, must be 'mega' or 'inverter': %s\n", model_name);
        return 1;
    }

    char ip[64];
    if (!resolve(address, ip, sizeof(ip)))
        return 1;
    thermia_modbus_quiet(true);
    if (!thermia_modbus_open(ip, 502, model))
        return 1;
    static status_t status;
    read_status(&status);
    thermia_modbus_close();

    show_header(&status, address, ip);
    show_now(&status);
    show_compressor(&status, verbose);
    show_heating(&status, verbose);
    show_water(&status, verbose);
    show_pool(&status, verbose);
    show_extra(&status, verbose);
    show_alarms(&status, verbose);

    if (g_failures > 0) {
        fprintf(stderr, "\nthermia-status: %d read(s) failed (shown as ?)\n", g_failures);
        return 1;
    }
    return 0;
}

// ------------------------------------------------------------------------------------------------------------------------
// ------------------------------------------------------------------------------------------------------------------------
