/* Copyright (c) 2026 Tunuino. */
#include "tunuino/calibration.h"
#include <math.h>
#include <stdbool.h>
#include <string.h>

static const uint8_t magic[] = {'T','U','N','U','I','N','O','M'};
static int shape(unsigned kind, unsigned nx, unsigned ny) {
    return (kind == TUNUINO_MAP_VE || kind == TUNUINO_MAP_IGNITION)
        && nx >= 2 && nx <= TUNUINO_MAP_AXIS_MAX && ny >= 2 && ny <= TUNUINO_MAP_AXIS_MAX;
}
static int cell_ok(unsigned kind, int32_t cell) {
    return kind == TUNUINO_MAP_VE ? cell >= 0 && cell <= 300000 : cell >= -90000 && cell <= 90000;
}
static uint32_t get_u32(const uint8_t *p) {
    return ((uint32_t)p[0] << 24) | ((uint32_t)p[1] << 16) | ((uint32_t)p[2] << 8) | p[3];
}
static int32_t get_i32(const uint8_t *p) {
    uint32_t raw = get_u32(p);
    return (int32_t)(raw <= INT32_MAX ? (int64_t)raw : (int64_t)raw - INT64_C(4294967296));
}
static void put_u32(uint8_t *p, uint32_t value) {
    for (unsigned i = 0; i < 4; ++i) { p[i] = (uint8_t)(value >> (24u - i * 8u)); }
}

tunuino_map_status tunuino_map_validate(const tunuino_map *map) {
    if (map == NULL) { return TUNUINO_MAP_ARGUMENT; }
    if (!shape(map->kind, map->rpm_count, map->load_count)) { return TUNUINO_MAP_FORMAT; }
    for (unsigned i = 0; i < map->rpm_count; ++i) {
        if (map->rpm[i] < 0 || (i && map->rpm[i] <= map->rpm[i - 1])) { return TUNUINO_MAP_RANGE; }
    }
    for (unsigned j = 0; j < map->load_count; ++j) {
        if (map->load_milli_kpa[j] < 0 || (j && map->load_milli_kpa[j] <= map->load_milli_kpa[j - 1])) {
            return TUNUINO_MAP_RANGE;
        }
        for (unsigned i = 0; i < map->rpm_count; ++i) {
            if (!cell_ok(map->kind, map->cells[j][i])) { return TUNUINO_MAP_RANGE; }
        }
    }
    return TUNUINO_MAP_OK;
}

tunuino_map_status tunuino_map_encode(const tunuino_map *map, uint8_t *bytes, size_t capacity, size_t *written) {
    if (written != NULL) { *written = 0; }
    if (bytes == NULL || written == NULL) { return TUNUINO_MAP_ARGUMENT; }
    tunuino_map_status valid = tunuino_map_validate(map);
    if (valid != TUNUINO_MAP_OK) { return valid; }
    size_t length = 16u + 4u * (map->rpm_count + map->load_count + map->rpm_count * map->load_count);
    if (capacity < length) { return TUNUINO_MAP_CAPACITY; }
    memcpy(bytes, magic, sizeof(magic));
    bytes[8] = 1; bytes[9] = (uint8_t)map->kind; bytes[10] = map->rpm_count; bytes[11] = map->load_count;
    size_t offset = 12;
    for (unsigned i = 0; i < map->rpm_count; ++i, offset += 4) { put_u32(bytes + offset, (uint32_t)map->rpm[i]); }
    for (unsigned j = 0; j < map->load_count; ++j, offset += 4) { put_u32(bytes + offset, (uint32_t)map->load_milli_kpa[j]); }
    for (unsigned j = 0; j < map->load_count; ++j) {
        for (unsigned i = 0; i < map->rpm_count; ++i, offset += 4) { put_u32(bytes + offset, (uint32_t)map->cells[j][i]); }
    }
    put_u32(bytes + offset, tunuino_crc32(bytes, offset));
    *written = length;
    return TUNUINO_MAP_OK;
}

tunuino_map_status tunuino_map_decode(const uint8_t *bytes, size_t length, tunuino_map *map) {
    if (bytes == NULL || map == NULL) { return TUNUINO_MAP_ARGUMENT; }
    if (length < 48 || length > TUNUINO_MAP_FILE_MAX || memcmp(bytes, magic, sizeof(magic)) != 0
            || bytes[8] != 1 || !shape(bytes[9], bytes[10], bytes[11])) { return TUNUINO_MAP_FORMAT; }
    unsigned kind = bytes[9], nx = bytes[10], ny = bytes[11];
    if (length != 16u + 4u * (nx + ny + nx * ny)) { return TUNUINO_MAP_FORMAT; }
    if (tunuino_crc32(bytes, length - 4) != get_u32(bytes + length - 4)) { return TUNUINO_MAP_CRC; }
    /* Validate before committing anything into caller-owned output storage. */
    size_t offset = 12;
    for (unsigned axis = 0; axis < 2; ++axis) {
        unsigned count = axis == 0 ? nx : ny;
        int32_t previous = -1;
        for (unsigned i = 0; i < count; ++i, offset += 4) {
            int32_t value = get_i32(bytes + offset);
            if (value < 0 || value <= previous) { return TUNUINO_MAP_RANGE; }
            previous = value;
        }
    }
    for (unsigned i = 0; i < nx * ny; ++i, offset += 4) {
        if (!cell_ok(kind, get_i32(bytes + offset))) { return TUNUINO_MAP_RANGE; }
    }
    map->kind = (tunuino_map_kind)kind; map->rpm_count = (uint8_t)nx; map->load_count = (uint8_t)ny;
    offset = 12;
    for (unsigned i = 0; i < nx; ++i, offset += 4) { map->rpm[i] = get_i32(bytes + offset); }
    for (unsigned j = 0; j < ny; ++j, offset += 4) { map->load_milli_kpa[j] = get_i32(bytes + offset); }
    for (unsigned j = 0; j < ny; ++j) {
        for (unsigned i = 0; i < nx; ++i, offset += 4) { map->cells[j][i] = get_i32(bytes + offset); }
    }
    return TUNUINO_MAP_OK;
}

static unsigned bin(const int32_t *axis, unsigned count, double value) {
    unsigned i = 0;
    while (i + 2 < count && value > axis[i + 1]) { ++i; }
    return i;
}
tunuino_map_status tunuino_map_lookup(const tunuino_map *map, double rpm, double map_kpa, double *value) {
    if (value == NULL || !isfinite(rpm) || !isfinite(map_kpa) || rpm < 0 || map_kpa < 0) {
        return TUNUINO_MAP_ARGUMENT;
    }
    tunuino_map_status valid = tunuino_map_validate(map);
    if (valid != TUNUINO_MAP_OK) { return valid; }
    double xmin = map->rpm[0], xmax = map->rpm[map->rpm_count - 1];
    double ymin = map->load_milli_kpa[0] / 1000.0, ymax = map->load_milli_kpa[map->load_count - 1] / 1000.0;
    int clamped = rpm < xmin || rpm > xmax || map_kpa < ymin || map_kpa > ymax;
    double x = rpm < xmin ? xmin : rpm > xmax ? xmax : rpm;
    double y = map_kpa <= ymin ? map->load_milli_kpa[0]
        : map_kpa >= ymax ? map->load_milli_kpa[map->load_count - 1] : map_kpa * 1000.0;
    unsigned i = bin(map->rpm, map->rpm_count, x), j = bin(map->load_milli_kpa, map->load_count, y);
    double wx = (x - map->rpm[i]) / ((double)map->rpm[i + 1] - map->rpm[i]);
    double wy = (y - map->load_milli_kpa[j]) / ((double)map->load_milli_kpa[j + 1] - map->load_milli_kpa[j]);
    double low = map->cells[j][i] * (1 - wx) + map->cells[j][i + 1] * wx;
    double high = map->cells[j + 1][i] * (1 - wx) + map->cells[j + 1][i + 1] * wx;
    *value = (low * (1 - wy) + high * wy) / 1000.0;
    return clamped ? TUNUINO_MAP_CLAMPED : TUNUINO_MAP_OK;
}

/* Uniform subdivision of the largest remaining interval keeps every original
 * RPM and kPa axis knot. Values are in integer RPM or milli-kPa, respectively. */
static bool refine_axis_16(const int32_t original[8], int32_t refined[16]) {
    memcpy(refined,original,8*sizeof(int32_t));
    for(unsigned count=8;count<16;++count) {
        unsigned best=0;
        int64_t width=0;
        for(unsigned i=0;i<count-1;++i) {
            const int64_t gap=(int64_t)refined[i+1]-refined[i];
            if(gap>width) {width=gap;best=i;}
        }
        if(width<2)return false;
        const int32_t new_value=(int32_t)((int64_t)refined[best]+width/2);
        memmove(refined+best+2,refined+best+1,(count-best-1)*sizeof(int32_t));
        refined[best+1]=new_value;
    }
    return true;
}

tunuino_map_status tunuino_map_refine_16(const tunuino_map *source, tunuino_map *destination) {
    if(source==NULL||destination==NULL||source==destination)
        return TUNUINO_MAP_ARGUMENT;
    if(source->rpm_count!=8||source->load_count!=8||tunuino_map_validate(source)!=TUNUINO_MAP_OK)
        return TUNUINO_MAP_FORMAT;
    int32_t rpm[16],load[16];
    if(!refine_axis_16(source->rpm,rpm)||!refine_axis_16(source->load_milli_kpa,load))
        return TUNUINO_MAP_RANGE;
    /* At this point no further failure is possible for a validated input:
     * every new knot lies in the original bounding rectangle, and bilinear
     * interpolation of valid cells remains within that kind's value range. */
    destination->kind=source->kind;
    destination->rpm_count=destination->load_count=16;
    for(unsigned i=0;i<16;++i)destination->rpm[i]=rpm[i];
    for(unsigned i=0;i<16;++i)destination->load_milli_kpa[i]=load[i];
    for(unsigned row=0;row<16;++row)for(unsigned col=0;col<16;++col) {
        int exact_row=-1, exact_col=-1;
        for(unsigned k=0;k<8;++k) {
            if(source->load_milli_kpa[k]==load[row])exact_row=(int)k;
            if(source->rpm[k]==rpm[col])exact_col=(int)k;
        }
        if(exact_row>=0&&exact_col>=0) {
            destination->cells[row][col]=source->cells[exact_row][exact_col];
        } else {
            double value=0.0;
            (void)tunuino_map_lookup(source,(double)rpm[col],load[row]/1000.0,&value);
            destination->cells[row][col]=(int32_t)lround(value*1000.0);
        }
    }
    return TUNUINO_MAP_OK;
}
