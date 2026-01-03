const std = @import("std");
const geometry = @import("geometry.zig");

const Vec3 = geometry.Vec3;
const Aabb = geometry.Aabb;
const Ray = geometry.Ray;

const doNotOptimizeAway = std.mem.doNotOptimizeAway;

fn print(comptime fmt: []const u8, args: anytype) void {
    const stdout = std.fs.File.stdout();
    var buf: [4096]u8 = undefined;
    const slice = std.fmt.bufPrint(&buf, fmt, args) catch return;
    _ = stdout.write(slice) catch {};
}

fn formatTime(ns: f64, buf: []u8) []const u8 {
    if (ns < 1_000) {
        return std.fmt.bufPrint(buf, "{d:.2}ns", .{ns}) catch "?";
    } else if (ns < 1_000_000) {
        return std.fmt.bufPrint(buf, "{d:.2}us", .{ns / 1_000.0}) catch "?";
    } else if (ns < 1_000_000_000) {
        return std.fmt.bufPrint(buf, "{d:.2}ms", .{ns / 1_000_000.0}) catch "?";
    } else {
        return std.fmt.bufPrint(buf, "{d:.2}s", .{ns / 1_000_000_000.0}) catch "?";
    }
}

fn randomFloat(rng: std.Random) f32 {
    return rng.float(f32) * 20.0 - 10.0;
}

fn randomAabb(rng: std.Random) Aabb {
    const x1 = randomFloat(rng);
    const y1 = randomFloat(rng);
    const z1 = randomFloat(rng);
    const x2 = x1 + rng.float(f32) * 5.0 + 0.1;
    const y2 = y1 + rng.float(f32) * 5.0 + 0.1;
    const z2 = z1 + rng.float(f32) * 5.0 + 0.1;
    return .{
        .bmin = Vec3.fromArray(&.{ x1, y1, z1 }),
        .bmax = Vec3.fromArray(&.{ x2, y2, z2 }),
    };
}

fn randomRay(rng: std.Random) Ray {
    const ox = randomFloat(rng);
    const oy = randomFloat(rng);
    const oz = randomFloat(rng);
    var dx = randomFloat(rng);
    var dy = randomFloat(rng);
    var dz = randomFloat(rng);
    const len = @sqrt(dx * dx + dy * dy + dz * dz);
    if (len > 0.001) {
        dx /= len;
        dy /= len;
        dz /= len;
    } else {
        dx = 1.0;
        dy = 0.0;
        dz = 0.0;
    }
    return .{
        .origin = Vec3.fromArray(&.{ ox, oy, oz }),
        .direction = Vec3.fromArray(&.{ dx, dy, dz }),
    };
}

fn benchmarkCenter(iterations: usize, rng: std.Random, alloc: std.mem.Allocator) !f64 {
    const aabbs = try alloc.alloc(Aabb, iterations);
    defer alloc.free(aabbs);

    for (0..iterations) |i| {
        aabbs[i] = randomAabb(rng);
    }

    var acc: f32 = 0;
    const start = std.time.nanoTimestamp();
    for (0..iterations) |i| {
        const c = aabbs[i].center();
        acc += c.toArray()[0];
    }
    const end = std.time.nanoTimestamp();
    doNotOptimizeAway(acc);

    return @as(f64, @floatFromInt(end - start)) / @as(f64, @floatFromInt(iterations));
}

fn benchmarkOverlaps(iterations: usize, rng: std.Random, alloc: std.mem.Allocator) !f64 {
    const aabbs_a = try alloc.alloc(Aabb, iterations);
    defer alloc.free(aabbs_a);
    const aabbs_b = try alloc.alloc(Aabb, iterations);
    defer alloc.free(aabbs_b);

    for (0..iterations) |i| {
        aabbs_a[i] = randomAabb(rng);
        aabbs_b[i] = randomAabb(rng);
    }

    var acc: usize = 0;
    const start = std.time.nanoTimestamp();
    for (0..iterations) |i| {
        if (aabbs_a[i].overlaps(aabbs_b[i])) acc += 1;
    }
    const end = std.time.nanoTimestamp();
    doNotOptimizeAway(acc);

    return @as(f64, @floatFromInt(end - start)) / @as(f64, @floatFromInt(iterations));
}

fn benchmarkIntersectsRay(iterations: usize, rng: std.Random, alloc: std.mem.Allocator) !f64 {
    const aabbs = try alloc.alloc(Aabb, iterations);
    defer alloc.free(aabbs);
    const rays = try alloc.alloc(Ray, iterations);
    defer alloc.free(rays);

    for (0..iterations) |i| {
        aabbs[i] = randomAabb(rng);
        rays[i] = randomRay(rng);
    }

    var acc: usize = 0;
    const start = std.time.nanoTimestamp();
    for (0..iterations) |i| {
        if (aabbs[i].intersectsRay(rays[i])) acc += 1;
    }
    const end = std.time.nanoTimestamp();
    doNotOptimizeAway(acc);

    return @as(f64, @floatFromInt(end - start)) / @as(f64, @floatFromInt(iterations));
}

pub fn main() !void {
    const args = try std.process.argsAlloc(std.heap.page_allocator);
    defer std.process.argsFree(std.heap.page_allocator, args);

    var iterations: usize = 1_000_000;

    for (args) |arg| {
        if (std.mem.startsWith(u8, arg, "-N=")) {
            iterations = std.fmt.parseInt(usize, arg[3..], 10) catch 1_000_000;
        }
    }

    const ts: i128 = std.time.nanoTimestamp();
    var prng = std.Random.DefaultPrng.init(@truncate(@as(u128, @bitCast(ts))));
    const rng = prng.random();

    const alloc = std.heap.page_allocator;

    print("\n AABB Benchmark ({d} iterations, ReleaseFast)\n", .{iterations});
    print("{s}\n", .{"=" ** 50});
    print(" {s: <30} | {s: >15}\n", .{ "Operation", "Time/op" });
    print("{s}\n", .{"-" ** 50});

    var buf: [32]u8 = undefined;

    const center_ns = try benchmarkCenter(iterations, rng, alloc);
    print(" {s: <30} | {s: >15}\n", .{ "Aabb.center", formatTime(center_ns, &buf) });

    const overlaps_ns = try benchmarkOverlaps(iterations, rng, alloc);
    print(" {s: <30} | {s: >15}\n", .{ "Aabb.overlaps", formatTime(overlaps_ns, &buf) });

    const ray_ns = try benchmarkIntersectsRay(iterations, rng, alloc);
    print(" {s: <30} | {s: >15}\n", .{ "Aabb.intersectsRay", formatTime(ray_ns, &buf) });

    print("{s}\n\n", .{"=" ** 50});
}
