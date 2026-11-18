// ------------------------------------------------------------------------------------------------------------------------
// thermia-caleon: the Thermia's heating driven from the Caleon (the underfloor heating's room controller), over MQTT (caleon2mqtt).
//
// No room sensor is fitted to the Thermia: it runs its heat curve (a supply set point by outdoor temperature), and its comfort wheel
// shifts the curve, +1 on the wheel = +3°C on the supply set point (22: the curve as the installer left it, for a house at 21). The
// Caleon controls each room (a valve per room) and knows two things the Thermia doesn't:
//   - the rooms' set points, by its mode: regular 21, comfort 25, eco and holiday 10, off 4.5  (caleon/room/<n>/ecsRcTset, retained)
//   - its flow temperature request: 30, plus 3 per degree the coldest room is under its set point  (ecsRcTflow on caleon/sensor, each minute)
// The wheel follows both:
//   wheel = 22 + (the highest set point - 21)              the house's target: an away mode turns the heating down, comfort up
//             + (the flow request - 30) / 3, at most +2    the coldest room: what the Caleon asks for, on top of the curve
// in 0.5 steps, from 10 (the lowest the Thermia takes). A set point change is written at once, a change from the flow request at most
// every HOLD. Without a known set point it assumes 21; without a recent flow request (STALE), nothing from it. Back to 22 when stopped
// (TERM, INT, HUP); killed hard, the wheel stays where it was until this runs again.
//
// Settings by environment (systemd: /etc/default/thermia-caleon), the defaults:
//   THERMIA_ADDRESS=192.168.0.106 THERMIA_MODEL=mega BROKER=localhost BROKER_PORT=1883 TOPIC=caleon STATE_TOPIC=thermia/caleon
//   WHEEL_BASE=22 SETPOINT_BASE=21 FLOW_BASE=30 FLOW_PER_STEP=3 SHIFT_MAX=2 WHEEL_MIN=10 WHEEL_MAX=30
//   INTERVAL=60 HOLD=900 STALE=600 STATUS=900 DRY_RUN= (set: look and log, never write)
// Its state, retained: <STATE_TOPIC>/state (JSON, at each look at the Thermia: every STATUS, and on every write).
// ------------------------------------------------------------------------------------------------------------------------

#include <cjson/cJSON.h>
#include <math.h>
#include <mosquitto.h>
#include <signal.h>
#include <stdarg.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <unistd.h>

#include "common_thermia.h"

// ------------------------------------------------------------------------------------------------------------------------
// ------------------------------------------------------------------------------------------------------------------------

#define ROOMS_MAXIMUM 16
#define SETTLE_SECONDS 5 // after connecting, for the retained messages (each room's set point) to arrive before the first look
#define WHEEL_REGISTER "setHeatingComfortWheelSetting"

typedef struct {
    bool known;
    double value;
    time_t ts;
} value_t;

typedef struct {
    value_t setpoint, temperature;
    char name[32], mode[16];
} room_t;

typedef struct {
    bool setpoints_known, flow_known;
    double setpoint_max, flow, shift_setpoint, shift_flow, target;
    int coldest, rooms; // the room furthest under its set point (-1: none known); how many rooms have a temperature
    double temperature_min, temperature_max, temperature_mean;
} plan_t;

typedef struct {
    bool ok;
    double wheel, supply_set, supply, outdoor;
    int demand;
} thermia_t;

static struct {
    const char *thermia_address, *broker, *topic, *state_topic;
    model_t thermia_model;
    int broker_port, interval, hold, stale, status;
    double wheel_base, setpoint_base, flow_base, flow_per_step, shift_max, wheel_min, wheel_max;
    bool dry_run;
} g_cfg;

static room_t g_rooms[ROOMS_MAXIMUM];
static value_t g_flow;
static bool g_setpoint_changed = false;
static volatile sig_atomic_t g_stop = 0;
static struct mosquitto *g_mosq = NULL;

// ------------------------------------------------------------------------------------------------------------------------
// ------------------------------------------------------------------------------------------------------------------------

__attribute__((format(printf, 1, 2))) static void log_line(const char *fmt, ...) {
    char when[32];
    const time_t now = time(NULL);
    strftime(when, sizeof(when), "%Y-%m-%d %H:%M:%S", localtime(&now));
    printf("%s  ", when);
    va_list ap;
    va_start(ap, fmt);
    vprintf(fmt, ap);
    va_end(ap);
    printf("\n");
}

static const char *env_string(const char *name, const char *fallback) {
    const char *v = getenv(name);
    return v != NULL && *v != '\0' ? v : fallback;
}
static double env_double(const char *name, double fallback) {
    const char *v = getenv(name);
    if (v == NULL || *v == '\0')
        return fallback;
    char *end;
    const double d = strtod(v, &end);
    if (*end != '\0') {
        fprintf(stderr, "%s: '%s' is not a number, using %g\n", name, v, fallback);
        return fallback;
    }
    return d;
}

static void configure(void) {
    g_cfg.thermia_address = env_string("THERMIA_ADDRESS", "192.168.0.106");
    g_cfg.thermia_model = strcmp(env_string("THERMIA_MODEL", "mega"), "inverter") == 0 ? MODEL_INVERTER : MODEL_MEGA;
    g_cfg.broker = env_string("BROKER", "localhost");
    g_cfg.broker_port = (int)env_double("BROKER_PORT", 1883);
    g_cfg.topic = env_string("TOPIC", "caleon");
    g_cfg.state_topic = env_string("STATE_TOPIC", "thermia/caleon");
    g_cfg.wheel_base = env_double("WHEEL_BASE", 22.0);
    g_cfg.setpoint_base = env_double("SETPOINT_BASE", 21.0);
    g_cfg.flow_base = env_double("FLOW_BASE", 30.0);
    g_cfg.flow_per_step = env_double("FLOW_PER_STEP", 3.0);
    g_cfg.shift_max = env_double("SHIFT_MAX", 2.0);
    g_cfg.wheel_min = env_double("WHEEL_MIN", 10.0);
    g_cfg.wheel_max = env_double("WHEEL_MAX", 30.0);
    g_cfg.interval = (int)env_double("INTERVAL", 60);
    g_cfg.hold = (int)env_double("HOLD", 900);
    g_cfg.stale = (int)env_double("STALE", 600);
    g_cfg.status = (int)env_double("STATUS", 900);
    g_cfg.dry_run = getenv("DRY_RUN") != NULL && *getenv("DRY_RUN") != '\0';
    if (g_cfg.flow_per_step <= 0)
        g_cfg.flow_per_step = 3.0;
    if (g_cfg.interval < 5)
        g_cfg.interval = 5;
}

static double clamp(double v, double lo, double hi) { return v < lo ? lo : v > hi ? hi : v; }
static double round_half(double v) { return floor(v * 2.0 + 0.5) / 2.0; }
static bool same(double a, double b) { return fabs(a - b) < 0.005; }

static const char *room_name(int index) {
    static char fallback[16];
    if (g_rooms[index].name[0] != '\0')
        return g_rooms[index].name;
    snprintf(fallback, sizeof(fallback), "room %d", index);
    return fallback;
}

static const char *demand_name(int demand) {
    switch (demand) {
    case 2:
        return "defrost";
    case 3:
        return "hot-water";
    case 4:
        return "heating";
    case 6:
        return "pool";
    case 7:
        return "legionella";
    case 98:
        return "standby";
    case 99:
        return "none";
    case 100:
        return "off";
    default:
        return "?";
    }
}

// ------------------------------------------------------------------------------------------------------------------------
// ------------------------------------------------------------------------------------------------------------------------

// the Caleon, from caleon2mqtt: {"function":"ecsRcTset","room_index":7,"room_name":"gröntrum","value":21,"ts":...}; the same messages on
// <topic>/sensor (live) and <topic>/room/<n>/<function> (retained: the latest for each room at (re)connection) - the newer one wins

static void update(value_t *v, double value, time_t ts) {
    if (v->known && ts < v->ts)
        return;
    v->known = true;
    v->value = value;
    v->ts = ts;
}

static void on_message(struct mosquitto *mosq, void *userdata, const struct mosquitto_message *msg) {
    (void)mosq;
    (void)userdata;
    if (msg->payloadlen <= 0)
        return;
    char *text = malloc((size_t)msg->payloadlen + 1);
    if (text == NULL)
        return;
    memcpy(text, msg->payload, (size_t)msg->payloadlen);
    text[msg->payloadlen] = '\0';
    cJSON *j = cJSON_Parse(text);
    free(text);
    if (j == NULL)
        return;

    const cJSON *function = cJSON_GetObjectItemCaseSensitive(j, "function");
    const cJSON *value = cJSON_GetObjectItemCaseSensitive(j, "value");
    const cJSON *ts = cJSON_GetObjectItemCaseSensitive(j, "ts");
    const cJSON *index = cJSON_GetObjectItemCaseSensitive(j, "room_index");
    const time_t when = cJSON_IsNumber(ts) ? (time_t)ts->valuedouble : time(NULL);
    if (cJSON_IsString(function) && cJSON_IsNumber(value)) {
        const char *f = function->valuestring;
        if (strcmp(f, "ecsRcTflow") == 0)
            update(&g_flow, value->valuedouble, when);
        else if (cJSON_IsNumber(index) && index->valueint >= 0 && index->valueint < ROOMS_MAXIMUM) {
            room_t *r = &g_rooms[index->valueint];
            const cJSON *name = cJSON_GetObjectItemCaseSensitive(j, "room_name");
            if (cJSON_IsString(name))
                snprintf(r->name, sizeof(r->name), "%s", name->valuestring);
            if (strcmp(f, "ecsRcTset") == 0) {
                const bool was_known = r->setpoint.known;
                const double was = r->setpoint.value;
                update(&r->setpoint, value->valuedouble, when);
                if (!was_known || !same(was, r->setpoint.value))
                    g_setpoint_changed = true;
            } else if (strcmp(f, "ecsRcTroom") == 0)
                update(&r->temperature, value->valuedouble, when);
            else if (strcmp(f, "ecsRcSwitch") == 0) {
                const cJSON *mode = cJSON_GetObjectItemCaseSensitive(j, "mode_name");
                if (cJSON_IsString(mode))
                    snprintf(r->mode, sizeof(r->mode), "%s", mode->valuestring);
            }
        }
    }
    cJSON_Delete(j);
}

static void on_connect(struct mosquitto *mosq, void *userdata, int rc) {
    (void)userdata;
    if (rc != 0) {
        log_line("mqtt: connection refused: %s", mosquitto_connack_string(rc));
        return;
    }
    char sensor[128], rooms[128];
    snprintf(sensor, sizeof(sensor), "%s/sensor", g_cfg.topic);
    snprintf(rooms, sizeof(rooms), "%s/room/+/+", g_cfg.topic);
    mosquitto_subscribe(mosq, NULL, sensor, 0);
    mosquitto_subscribe(mosq, NULL, rooms, 0);
    log_line("mqtt: connected to %s:%d, following %s and %s", g_cfg.broker, g_cfg.broker_port, sensor, rooms);
}

// ------------------------------------------------------------------------------------------------------------------------
// ------------------------------------------------------------------------------------------------------------------------

static plan_t make_plan(time_t now) {
    plan_t p = {.coldest = -1, .setpoint_max = g_cfg.setpoint_base};
    double coldest_by = 0, sum = 0;
    for (int i = 0; i < ROOMS_MAXIMUM; i++) {
        const room_t *r = &g_rooms[i];
        if (r->setpoint.known && (!p.setpoints_known || r->setpoint.value > p.setpoint_max)) {
            p.setpoint_max = r->setpoint.value;
            p.setpoints_known = true;
        }
        if (r->temperature.known) {
            const double t = r->temperature.value;
            p.temperature_min = p.rooms == 0 || t < p.temperature_min ? t : p.temperature_min;
            p.temperature_max = p.rooms == 0 || t > p.temperature_max ? t : p.temperature_max;
            sum += t;
            p.rooms++;
            if (r->setpoint.known && (p.coldest < 0 || r->setpoint.value - t > coldest_by)) {
                p.coldest = i;
                coldest_by = r->setpoint.value - t;
            }
        }
    }
    p.temperature_mean = p.rooms > 0 ? sum / p.rooms : 0;
    p.shift_setpoint = p.setpoints_known ? p.setpoint_max - g_cfg.setpoint_base : 0;
    p.flow_known = g_flow.known && now - g_flow.ts <= g_cfg.stale;
    p.flow = g_flow.value;
    p.shift_flow = p.flow_known ? clamp((p.flow - g_cfg.flow_base) / g_cfg.flow_per_step, 0, g_cfg.shift_max) : 0;
    p.target = clamp(g_cfg.wheel_base + round_half(p.shift_setpoint + p.shift_flow), g_cfg.wheel_min, g_cfg.wheel_max);
    return p;
}

static const char *describe_plan(const plan_t *p) {
    static char s[256];
    char coldest[96] = "";
    if (p->coldest >= 0)
        snprintf(coldest, sizeof(coldest), "  coldest %s %.1f (set %.1f)", room_name(p->coldest), g_rooms[p->coldest].temperature.value,
                 g_rooms[p->coldest].setpoint.value);
    char flow[64];
    if (p->flow_known)
        snprintf(flow, sizeof(flow), "flow request %.1f (%+.1f)", p->flow, p->shift_flow);
    else
        snprintf(flow, sizeof(flow), "flow request none recent (+0.0)");
    snprintf(s, sizeof(s), "set points %s %.1f (%+.1f), %s%s", p->setpoints_known ? "highest" : "unknown, assumed", p->setpoint_max, p->shift_setpoint, flow,
             coldest);
    return s;
}

// ------------------------------------------------------------------------------------------------------------------------
// ------------------------------------------------------------------------------------------------------------------------

// the Thermia: a connection for each look (open, read and maybe write, close) - modbus TCP, alongside anything else talking to it

static bool read_scaled(const char *name, double *value) {
    int raw;
    if (!thermia_modbus_read_register_int(name, &raw))
        return false;
    const int scale = thermia_register_scale(name);
    *value = scale > 1 ? (double)raw / scale : raw;
    return true;
}

static bool thermia_open(void) {
    thermia_modbus_quiet(true);
    return thermia_modbus_open(g_cfg.thermia_address, 502, g_cfg.thermia_model);
}

static thermia_t thermia_read(void) {
    thermia_t t = {0};
    double demand = 0;
    t.ok = read_scaled(WHEEL_REGISTER, &t.wheel);
    if (t.ok) {
        read_scaled("valueHeatingSystemSupplyLineCalculatedSetPoint", &t.supply_set);
        read_scaled("valueHeatpumpCondenserOutTemperature", &t.supply);
        read_scaled("valueHeatpumpOutdoorTemperature", &t.outdoor);
        read_scaled("valueHeatpumpCurrentlyRunningFirstPrioritisedDemand", &demand);
        t.demand = (int)demand;
    }
    return t;
}

static bool thermia_write_wheel(double wheel) {
    const int scale = thermia_register_scale(WHEEL_REGISTER);
    const int raw = (int)lround(wheel * (scale > 1 ? scale : 1));
    double back = NAN;
    if (!thermia_modbus_write_register_int(WHEEL_REGISTER, raw))
        return false;
    if (!read_scaled(WHEEL_REGISTER, &back) || !same(back, wheel)) {
        log_line("WARNING: wrote the wheel %.1f (raw %d), it reads back %.2f", wheel, raw, back);
        return false;
    }
    return true;
}

// ------------------------------------------------------------------------------------------------------------------------
// ------------------------------------------------------------------------------------------------------------------------

static void publish_state(const plan_t *p, const thermia_t *t, bool running) {
    cJSON *s = cJSON_CreateObject();
    cJSON_AddNumberToObject(s, "ts", (double)time(NULL));
    cJSON_AddBoolToObject(s, "running", running);
    cJSON_AddBoolToObject(s, "dry_run", g_cfg.dry_run);
    if (t->ok) {
        cJSON_AddNumberToObject(s, "wheel", t->wheel);
        cJSON_AddNumberToObject(s, "supply_set", t->supply_set);
        cJSON_AddNumberToObject(s, "supply", t->supply);
        cJSON_AddNumberToObject(s, "outdoor", t->outdoor);
        cJSON_AddStringToObject(s, "demand", demand_name(t->demand));
    }
    if (p != NULL) {
        cJSON_AddNumberToObject(s, "target", p->target);
        if (p->setpoints_known)
            cJSON_AddNumberToObject(s, "setpoint_max", p->setpoint_max);
        cJSON_AddNumberToObject(s, "shift_setpoint", p->shift_setpoint);
        if (p->flow_known)
            cJSON_AddNumberToObject(s, "flow_request", p->flow);
        cJSON_AddNumberToObject(s, "shift_flow", p->shift_flow);
        if (p->coldest >= 0) {
            cJSON *c = cJSON_AddObjectToObject(s, "coldest");
            cJSON_AddStringToObject(c, "room", room_name(p->coldest));
            cJSON_AddNumberToObject(c, "temperature", g_rooms[p->coldest].temperature.value);
            cJSON_AddNumberToObject(c, "setpoint", g_rooms[p->coldest].setpoint.value);
        }
        if (p->rooms > 0)
            cJSON_AddNumberToObject(s, "rooms_mean", round(p->temperature_mean * 100) / 100);
    }
    char *text = cJSON_PrintUnformatted(s);
    if (text != NULL) {
        char topic[160];
        snprintf(topic, sizeof(topic), "%s/state", g_cfg.state_topic);
        mosquitto_publish(g_mosq, NULL, topic, (int)strlen(text), text, 0, true);
        free(text);
    }
    cJSON_Delete(s);
}

static void log_status(const plan_t *p, const thermia_t *t) {
    char rooms[96] = "";
    if (p->rooms > 0)
        snprintf(rooms, sizeof(rooms), "  rooms %.1f-%.1f (mean %.2f)", p->temperature_min, p->temperature_max, p->temperature_mean);
    log_line("wheel %.1f (target %.1f)  %s%s  supply set %.1f  supply %.1f  %s  outdoor %.1f", t->wheel, p->target, describe_plan(p), rooms, t->supply_set,
             t->supply, demand_name(t->demand), t->outdoor);
}

// ------------------------------------------------------------------------------------------------------------------------
// ------------------------------------------------------------------------------------------------------------------------

static void on_signal(int sig) {
    (void)sig;
    g_stop = 1;
}

static void restore(void) {
    if (g_cfg.dry_run) {
        log_line("stopping (dry run: the wheel left as it is)");
        publish_state(NULL, &(thermia_t){0}, false);
        return;
    }
    if (thermia_open()) {
        const bool ok = thermia_write_wheel(g_cfg.wheel_base);
        const thermia_t t = thermia_read();
        thermia_modbus_close();
        log_line("stopping: the wheel back to %.1f%s", g_cfg.wheel_base, ok ? "" : " - FAILED");
        publish_state(NULL, &t, false);
    } else {
        log_line("stopping: the Thermia can't be reached - the wheel left as it is");
        publish_state(NULL, &(thermia_t){0}, false);
    }
}

int main(void) {
    setvbuf(stdout, NULL, _IOLBF, 0);
    configure();

    struct sigaction sa = {.sa_handler = on_signal};
    sigaction(SIGTERM, &sa, NULL);
    sigaction(SIGINT, &sa, NULL);
    sigaction(SIGHUP, &sa, NULL);

    mosquitto_lib_init();
    g_mosq = mosquitto_new(NULL, true, NULL);
    if (g_mosq == NULL) {
        fprintf(stderr, "mqtt: can't create a client\n");
        return 1;
    }
    mosquitto_connect_callback_set(g_mosq, on_connect);
    mosquitto_message_callback_set(g_mosq, on_message);

    log_line("thermia-caleon: the Thermia %s's wheel from %s on %s: %.1f + (highest set point - %.1f) + (flow request - %.1f) / %.1f (at most +%.1f), "
             "%.1f to %.1f; every %ds, flow changes at most every %ds%s",
             g_cfg.thermia_address, g_cfg.topic, g_cfg.broker, g_cfg.wheel_base, g_cfg.setpoint_base, g_cfg.flow_base, g_cfg.flow_per_step, g_cfg.shift_max,
             g_cfg.wheel_min, g_cfg.wheel_max, g_cfg.interval, g_cfg.hold, g_cfg.dry_run ? " (DRY RUN)" : "");

    bool connected = mosquitto_connect(g_mosq, g_cfg.broker, g_cfg.broker_port, 60) == MOSQ_ERR_SUCCESS;
    if (!connected)
        log_line("mqtt: can't connect to %s:%d, retrying", g_cfg.broker, g_cfg.broker_port);

    double wheel = NAN; // the wheel as last read or written
    time_t written_at = 0, next_look = time(NULL) + SETTLE_SECONDS, next_status = 0;
    while (!g_stop) {
        if (!connected) {
            sleep(2);
            connected = mosquitto_reconnect(g_mosq) == MOSQ_ERR_SUCCESS;
            continue;
        }
        if (mosquitto_loop(g_mosq, 1000, 1) != MOSQ_ERR_SUCCESS && !g_stop) {
            log_line("mqtt: connection lost, reconnecting");
            connected = false;
            continue;
        }
        const time_t now = time(NULL);
        if (now < next_look)
            continue;
        next_look = now + g_cfg.interval;

        const plan_t p = make_plan(now);
        const bool change = isnan(wheel) || !same(wheel, p.target);
        if (!change)
            g_setpoint_changed = false; // (a set point changed, the target didn't)
        const bool allowed = written_at == 0 || g_setpoint_changed || now - written_at >= g_cfg.hold;
        if (!(change && allowed) && now < next_status)
            continue;

        if (!thermia_open()) {
            log_line("the Thermia can't be reached, trying again in %ds", g_cfg.interval);
            continue;
        }
        thermia_t t = thermia_read();
        if (!t.ok) {
            thermia_modbus_close();
            log_line("the Thermia's wheel can't be read, trying again in %ds", g_cfg.interval);
            continue;
        }
        if (!g_cfg.dry_run && !isnan(wheel) && !same(wheel, t.wheel))
            log_line("the wheel was changed outside this: %.1f, not %.1f", t.wheel, wheel);
        const double current = g_cfg.dry_run && !isnan(wheel) ? wheel : t.wheel; // a dry run: the wheel as it would be
        wheel = current;
        if (allowed && !same(current, p.target)) {
            if (g_cfg.dry_run || thermia_write_wheel(p.target)) {
                log_line("%sthe wheel %.1f -> %.1f: %s", g_cfg.dry_run ? "dry run: " : "", current, p.target, describe_plan(&p));
                wheel = p.target;
                written_at = now;
                g_setpoint_changed = false;
                if (!g_cfg.dry_run)
                    t = thermia_read(); // the new supply set point
            }
        }
        thermia_modbus_close();
        log_status(&p, &t);
        publish_state(&p, &t, true);
        next_status = now + g_cfg.status;
    }

    restore();
    mosquitto_loop(g_mosq, 500, 1); // send the last state
    mosquitto_disconnect(g_mosq);
    mosquitto_destroy(g_mosq);
    mosquitto_lib_cleanup();
    return 0;
}
