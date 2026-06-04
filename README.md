# zig-astar

A small, generic implementation of the [A\* search algorithm](https://en.wikipedia.org/wiki/A*_search_algorithm) for Zig.

A\* finds a least-cost path between two nodes in a weighted graph. This library
is generic over the node, cost, and graph types: you describe your graph by
implementing two small methods and the library does the search. It works for
grids, mazes, road networks, puzzle state spaces, or any graph you can
enumerate.

- Single file, no dependencies.
- Generic over node type, cost type (integers or floats), and graph.
- Returns the optimal path when your heuristic is admissible.
- Allocator-aware: you control where memory comes from.

Tested with **Zig 0.16.0**.

## Usage

Specialize `AStar` for your node type, cost type, and a *context* that
describes the graph:

```zig
const std = @import("std");
const astar = @import("astar");

// Nodes must be usable as AutoHashMap keys (no pointers/slices).
const Point = struct { x: i32, y: i32 };

const Grid = struct {
    width: i32,
    height: i32,

    // Estimated remaining cost to the goal.
    // Must never overestimate (be "admissible") for an optimal result.
    pub fn heuristic(_: Grid, node: Point, goal: Point) u32 {
        return @abs(node.x - goal.x) + @abs(node.y - goal.y); // Manhattan distance
    }

    // Report each reachable neighbor and the cost to step there.
    pub fn neighbors(self: Grid, node: Point, out: Search.Successors) !void {
        const steps = [_]Point{
            .{ .x = 1, .y = 0 },  .{ .x = -1, .y = 0 },
            .{ .x = 0, .y = 1 },  .{ .x = 0, .y = -1 },
        };
        for (steps) |d| {
            const n = Point{ .x = node.x + d.x, .y = node.y + d.y };
            if (n.x >= 0 and n.y >= 0 and n.x < self.width and n.y < self.height) {
                try out.add(n, 1); // uniform cost of 1 per step
            }
        }
    }
};

// AStar(Node, Cost, Context)
const Search = astar.AStar(Point, u32, Grid);

pub fn main() !void {
    var gpa: std.heap.DebugAllocator(.{}) = .init;
    defer _ = gpa.deinit();
    const allocator = gpa.allocator();

    const grid = Grid{ .width = 10, .height = 10 };
    const path = try Search.findPath(allocator, grid, .{ .x = 0, .y = 0 }, .{ .x = 9, .y = 9 });
    defer if (path) |p| allocator.free(p);

    if (path) |p| {
        std.debug.print("path of {d} nodes\n", .{p.len});
    } else {
        std.debug.print("no path\n", .{});
    }
}
```

`findPath` returns a freshly allocated slice of nodes from `start` to `goal`
(inclusive), or `null` if no path exists. **You own the slice and must free it.**


## Installation

Fetch the package into your `build.zig.zon` (replace the tag with the version
or commit you want):

```sh
zig fetch --save git+https://github.com/keenanjohnson/zig-astar#v0.1.0
```

Then wire the module into your `build.zig`:

```zig
const astar = b.dependency("zig_astar", .{
    .target = target,
    .optimize = optimize,
});
exe.root_module.addImport("astar", astar.module("astar"));
```

Now `@import("astar")` is available in your code.

## Usage

Specialize `AStar` for your node type, cost type, and a *context* that
describes the graph:

```zig
const std = @import("std");
const astar = @import("astar");

// Nodes must be usable as AutoHashMap keys (no pointers/slices).
const Point = struct { x: i32, y: i32 };

const Grid = struct {
    width: i32,
    height: i32,

    // Estimated remaining cost to the goal.
    // Must never overestimate (be "admissible") for an optimal result.
    pub fn heuristic(_: Grid, node: Point, goal: Point) u32 {
        return @abs(node.x - goal.x) + @abs(node.y - goal.y); // Manhattan distance
    }

    // Report each reachable neighbor and the cost to step there.
    pub fn neighbors(self: Grid, node: Point, out: Search.Successors) !void {
        const steps = [_]Point{
            .{ .x = 1, .y = 0 },  .{ .x = -1, .y = 0 },
            .{ .x = 0, .y = 1 },  .{ .x = 0, .y = -1 },
        };
        for (steps) |d| {
            const n = Point{ .x = node.x + d.x, .y = node.y + d.y };
            if (n.x >= 0 and n.y >= 0 and n.x < self.width and n.y < self.height) {
                try out.add(n, 1); // uniform cost of 1 per step
            }
        }
    }
};

// AStar(Node, Cost, Context)
const Search = astar.AStar(Point, u32, Grid);

pub fn main() !void {
    var gpa: std.heap.DebugAllocator(.{}) = .init;
    defer _ = gpa.deinit();
    const allocator = gpa.allocator();

    const grid = Grid{ .width = 10, .height = 10 };
    const path = try Search.findPath(allocator, grid, .{ .x = 0, .y = 0 }, .{ .x = 9, .y = 9 });
    defer if (path) |p| allocator.free(p);

    if (path) |p| {
        std.debug.print("path of {d} nodes\n", .{p.len});
    } else {
        std.debug.print("no path\n", .{});
    }
}
```

`findPath` returns a freshly allocated slice of nodes from `start` to `goal`
(inclusive), or `null` if no path exists. **You own the slice and must free it.**


## Running the example

A complete maze-solving demo lives in [`examples/grid.zig`](examples/grid.zig):

```sh
zig build example
```

## Running the tests

```sh
zig build test
```

## License

[Apache-2.0](LICENSE)
