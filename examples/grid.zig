//! Runnable demo: find a path across a 2D grid with walls and print it.
//!
//!   zig build example

const std = @import("std");
const astar = @import("astar");

/// A 2D grid where `#` cells are walls and everything else is walkable.
/// Movement is 4-directional with uniform cost.
const Maze = struct {
    rows: []const []const u8,

    const Point = struct { x: i32, y: i32 };

    fn walkable(self: Maze, p: Point) bool {
        if (p.y < 0 or p.y >= self.rows.len) return false;
        const row = self.rows[@intCast(p.y)];
        if (p.x < 0 or p.x >= row.len) return false;
        return row[@intCast(p.x)] != '#';
    }

    /// Manhattan distance — admissible for 4-directional uniform-cost movement.
    pub fn heuristic(_: Maze, node: Point, goal: Point) u32 {
        return @abs(node.x - goal.x) + @abs(node.y - goal.y);
    }

    pub fn neighbors(self: Maze, node: Point, out: Search.Successors) !void {
        const deltas = [_]Point{
            .{ .x = 1, .y = 0 },
            .{ .x = -1, .y = 0 },
            .{ .x = 0, .y = 1 },
            .{ .x = 0, .y = -1 },
        };
        for (deltas) |d| {
            const next = Point{ .x = node.x + d.x, .y = node.y + d.y };
            if (self.walkable(next)) try out.add(next, 1);
        }
    }
};

const Search = astar.AStar(Maze.Point, u32, Maze);

pub fn main() !void {
    var gpa: std.heap.DebugAllocator(.{}) = .init;
    defer _ = gpa.deinit();
    const allocator = gpa.allocator();

    const maze = Maze{ .rows = &.{
        "S....#....",
        ".###.#.##.",
        ".#...#.#..",
        ".#.###.#.#",
        ".#.....#.#",
        ".#####.#.#",
        ".....#.#.#",
        "####.#.#.#",
        "...#...#.G",
        ".#.#.###..",
    } };

    const start = Maze.Point{ .x = 0, .y = 0 }; // 'S'
    const goal = Maze.Point{ .x = 9, .y = 8 }; // 'G'

    const path = try Search.findPath(allocator, maze, start, goal);
    defer if (path) |p| allocator.free(p);

    if (path == null) {
        std.debug.print("No path from start to goal.\n", .{});
        return;
    }

    // Mark the path cells with '*' and print the maze.
    var on_path = std.AutoHashMap(Maze.Point, void).init(allocator);
    defer on_path.deinit();
    for (path.?) |p| try on_path.put(p, {});

    std.debug.print("Path found in {d} steps:\n\n", .{path.?.len - 1});
    for (maze.rows, 0..) |row, y| {
        for (row, 0..) |cell, x| {
            const p = Maze.Point{ .x = @intCast(x), .y = @intCast(y) };
            if (cell == 'S' or cell == 'G') {
                std.debug.print("{c}", .{cell});
            } else if (on_path.contains(p)) {
                std.debug.print("*", .{});
            } else {
                std.debug.print("{c}", .{cell});
            }
        }
        std.debug.print("\n", .{});
    }
}
