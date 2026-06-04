# zig-astar

![maze](https://github.com/keenanjohnson/zig-astar-example/blob/main/demo.gif)

*Full example maze from https://github.com/keenanjohnson/zig-astar-example/.*

A\* pathfinding for Zig, in a single file with no dependencies.

Built and tested against Zig 0.16.0.

## Install

```sh
zig fetch --save git+https://github.com/keenanjohnson/zig-astar#v0.1.0
```

Then add the module in your `build.zig`:

```zig
const astar = b.dependency("zig_astar", .{
    .target = target,
    .optimize = optimize,
});
exe.root_module.addImport("astar", astar.module("astar"));
```

and `@import("astar")` from your code.

## Example

Here's pathfinding on a plain grid. `AStar(Node, Cost, Context)` takes your node
type, the number type for costs, and a context struct that describes the graph:

```zig
const std = @import("std");
const astar = @import("astar");

const Point = struct { x: i32, y: i32 };

const Grid = struct {
    width: i32,
    height: i32,

    // Estimated cost from node to goal. Must be non-negative and zero at the
    // goal; don't overestimate, or the path may not be optimal. Manhattan
    // distance works for 4-way movement.
    pub fn heuristic(_: Grid, node: Point, goal: Point) u32 {
        return @abs(node.x - goal.x) + @abs(node.y - goal.y);
    }

    // List the neighbors of a node by calling out.add(neighbor, cost).
    pub fn neighbors(self: Grid, node: Point, out: Search.Successors) !void {
        const steps = [_]Point{
            .{ .x = 1, .y = 0 }, .{ .x = -1, .y = 0 },
            .{ .x = 0, .y = 1 }, .{ .x = 0, .y = -1 },
        };
        for (steps) |d| {
            const n = Point{ .x = node.x + d.x, .y = node.y + d.y };
            if (n.x >= 0 and n.y >= 0 and n.x < self.width and n.y < self.height) {
                try out.add(n, 1);
            }
        }
    }
};

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

`findPath` hands back a slice of nodes from start to goal (or `null` if there's
no path). The slice is yours to free.

A couple of things to keep in mind: the node type has to work as an
`std.AutoHashMap` key, so stick to integers or structs of integers — no
pointers or slices. The cost type can be any number, integer or float. And if
you return `0` from `heuristic`, you get Dijkstra's algorithm.

Costs accumulate along the path, so pick a `Cost` type wide enough for your
worst-case total — `g + heuristic` is computed with ordinary arithmetic, which
overflows (a panic in Debug and `ReleaseSafe` builds) if the running total
exceeds the type's range.

## Running it

There's a maze-solving demo in [examples/grid.zig](examples/grid.zig):

```sh
zig build example
```

And the tests:

```sh
zig build test
```

## License

[Apache-2.0](LICENSE)
