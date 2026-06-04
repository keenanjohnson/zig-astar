//! A generic, allocator-aware implementation of the A* search algorithm.
//!
//! A* finds a least-cost path between a `start` and a `goal` node in a weighted
//! graph. The graph is described implicitly by a user-supplied *context* type,
//! so the same algorithm works for grids, road networks, state-space search, or
//! anything else that can enumerate neighbors and estimate a remaining cost.
//!
//! See `AStar` for the entry point and the README for a worked example.

const std = @import("std");

/// Build an A* searcher specialized for a given `Node`, `Cost`, and `Context`.
///
/// - `Node` is the type identifying a vertex in the graph. It must be usable as
///   an `std.AutoHashMap` key (i.e. it cannot contain pointers or slices); plain
///   integers and structs of integers work well (e.g. `struct { x: i32, y: i32 }`).
/// - `Cost` is the numeric type used for edge weights and distances. Any type
///   that supports `+` and ordering works — typically `u32`, `usize`, or `f32`.
/// - `Context` describes the graph. It must expose two methods:
///
///   ```zig
///   /// Estimated remaining cost from `node` to `goal`.
///   /// Must be admissible (never overestimate) for A* to return optimal paths.
///   pub fn heuristic(self: Context, node: Node, goal: Node) Cost
///
///   /// Report every node reachable from `node` and the cost of getting there
///   /// by calling `out.add(neighbor, edge_cost)` once per neighbor.
///   pub fn neighbors(self: Context, node: Node, out: Successors) !void
///   ```
pub fn AStar(comptime Node: type, comptime Cost: type, comptime Context: type) type {
    return struct {
        const Self = @This();

        /// An outgoing edge: a reachable `node` and the `cost` to traverse to it.
        pub const Neighbor = struct {
            node: Node,
            cost: Cost,
        };

        /// Passed to `Context.neighbors`; call `add` once per reachable node.
        pub const Successors = struct {
            list: *std.ArrayList(Neighbor),
            allocator: std.mem.Allocator,

            /// Record an edge from the current node to `node` with weight `cost`.
            pub fn add(self: Successors, node: Node, cost: Cost) !void {
                try self.list.append(self.allocator, .{ .node = node, .cost = cost });
            }
        };

        const Entry = struct {
            node: Node,
            g: Cost, // best known cost from start to this node when enqueued
            f: Cost, // g + heuristic; the priority key
        };

        fn compareF(_: void, a: Entry, b: Entry) std.math.Order {
            return std.math.order(a.f, b.f);
        }

        const Queue = std.PriorityQueue(Entry, void, compareF);

        /// Find a least-cost path from `start` to `goal`.
        ///
        /// On success returns a freshly allocated slice of nodes beginning with
        /// `start` and ending with `goal` (a single-element slice when they are
        /// equal). Returns `null` when no path exists. The caller owns the
        /// returned slice and must free it with `allocator`.
        pub fn findPath(
            allocator: std.mem.Allocator,
            context: Context,
            start: Node,
            goal: Node,
        ) !?[]Node {
            // Best known cost from start to each discovered node.
            var g_score = std.AutoHashMap(Node, Cost).init(allocator);
            defer g_score.deinit();
            // For each node, the node we arrived from on the best path so far.
            var came_from = std.AutoHashMap(Node, Node).init(allocator);
            defer came_from.deinit();
            // Frontier ordered by f = g + heuristic.
            var open: Queue = .empty;
            defer open.deinit(allocator);
            // Scratch buffer reused for each node's neighbors.
            var scratch: std.ArrayList(Neighbor) = .empty;
            defer scratch.deinit(allocator);

            try g_score.put(start, 0);
            try open.push(allocator, .{
                .node = start,
                .g = 0,
                .f = context.heuristic(start, goal),
            });

            while (open.pop()) |current| {
                if (std.meta.eql(current.node, goal)) {
                    return try reconstruct(allocator, &came_from, start, goal);
                }

                // The queue may hold stale entries for a node that was later
                // reached more cheaply. Skip them.
                const best = g_score.get(current.node) orelse continue;
                if (current.g > best) continue;

                scratch.clearRetainingCapacity();
                try context.neighbors(current.node, .{ .list = &scratch, .allocator = allocator });

                for (scratch.items) |edge| {
                    const tentative = current.g + edge.cost;
                    const known = g_score.get(edge.node);
                    if (known == null or tentative < known.?) {
                        try g_score.put(edge.node, tentative);
                        try came_from.put(edge.node, current.node);
                        try open.push(allocator, .{
                            .node = edge.node,
                            .g = tentative,
                            .f = tentative + context.heuristic(edge.node, goal),
                        });
                    }
                }
            }

            return null;
        }

        fn reconstruct(
            allocator: std.mem.Allocator,
            came_from: *std.AutoHashMap(Node, Node),
            start: Node,
            goal: Node,
        ) ![]Node {
            var path: std.ArrayList(Node) = .empty;
            errdefer path.deinit(allocator);

            var current = goal;
            try path.append(allocator, current);
            while (!std.meta.eql(current, start)) {
                current = came_from.get(current).?;
                try path.append(allocator, current);
            }
            std.mem.reverse(Node, path.items);
            return path.toOwnedSlice(allocator);
        }
    };
}

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

const testing = std.testing;

/// A small 2D grid used by the tests. `0` cells are walkable, `1` cells are
/// walls. Movement is 4-directional with uniform cost; the heuristic is the
/// Manhattan distance, which is admissible for this movement model.
const GridContext = struct {
    cells: []const u8,
    width: i32,
    height: i32,

    const Point = struct { x: i32, y: i32 };

    fn walkable(self: GridContext, p: Point) bool {
        if (p.x < 0 or p.y < 0 or p.x >= self.width or p.y >= self.height) return false;
        return self.cells[@intCast(p.y * self.width + p.x)] == 0;
    }

    pub fn heuristic(_: GridContext, node: Point, goal: Point) u32 {
        return @abs(node.x - goal.x) + @abs(node.y - goal.y);
    }

    pub fn neighbors(self: GridContext, node: Point, out: Grid.Successors) !void {
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

const Grid = AStar(GridContext.Point, u32, GridContext);

test "straight-line path on an empty grid" {
    const ctx = GridContext{
        .cells = &[_]u8{0} ** 9,
        .width = 3,
        .height = 3,
    };
    const path = try Grid.findPath(testing.allocator, ctx, .{ .x = 0, .y = 0 }, .{ .x = 2, .y = 0 });
    try testing.expect(path != null);
    defer testing.allocator.free(path.?);
    // Optimal Manhattan distance is 2, so the path has 3 nodes.
    try testing.expectEqual(@as(usize, 3), path.?.len);
    try testing.expectEqual(GridContext.Point{ .x = 0, .y = 0 }, path.?[0]);
    try testing.expectEqual(GridContext.Point{ .x = 2, .y = 0 }, path.?[path.?.len - 1]);
}

test "path routes around a wall" {
    // A vertical wall down the middle with a single gap at the bottom row.
    //   . # .
    //   . # .
    //   . . .
    const ctx = GridContext{
        .cells = &[_]u8{
            0, 1, 0,
            0, 1, 0,
            0, 0, 0,
        },
        .width = 3,
        .height = 3,
    };
    const path = try Grid.findPath(testing.allocator, ctx, .{ .x = 0, .y = 0 }, .{ .x = 2, .y = 0 });
    try testing.expect(path != null);
    defer testing.allocator.free(path.?);
    // Must detour down and around: (0,0)->(0,1)->(0,2)->(1,2)->(2,2)->(2,1)->(2,0) = 7 nodes.
    try testing.expectEqual(@as(usize, 7), path.?.len);
}

test "no path when goal is walled off" {
    // Center column is a full wall: the right side is unreachable.
    const ctx = GridContext{
        .cells = &[_]u8{
            0, 1, 0,
            0, 1, 0,
            0, 1, 0,
        },
        .width = 3,
        .height = 3,
    };
    const path = try Grid.findPath(testing.allocator, ctx, .{ .x = 0, .y = 0 }, .{ .x = 2, .y = 2 });
    try testing.expectEqual(@as(?[]GridContext.Point, null), path);
}

test "start equals goal yields a single-node path" {
    const ctx = GridContext{
        .cells = &[_]u8{0} ** 9,
        .width = 3,
        .height = 3,
    };
    const path = try Grid.findPath(testing.allocator, ctx, .{ .x = 1, .y = 1 }, .{ .x = 1, .y = 1 });
    try testing.expect(path != null);
    defer testing.allocator.free(path.?);
    try testing.expectEqual(@as(usize, 1), path.?.len);
    try testing.expectEqual(GridContext.Point{ .x = 1, .y = 1 }, path.?[0]);
}

test "returned path is contiguous and starts/ends correctly" {
    const ctx = GridContext{
        .cells = &[_]u8{0} ** 25,
        .width = 5,
        .height = 5,
    };
    const start = GridContext.Point{ .x = 0, .y = 0 };
    const goal = GridContext.Point{ .x = 4, .y = 3 };
    const path = try Grid.findPath(testing.allocator, ctx, start, goal);
    try testing.expect(path != null);
    defer testing.allocator.free(path.?);

    try testing.expectEqual(start, path.?[0]);
    try testing.expectEqual(goal, path.?[path.?.len - 1]);
    // Every consecutive pair must be exactly one grid step apart.
    for (path.?[1..], path.?[0 .. path.?.len - 1]) |b, a| {
        const step = @abs(a.x - b.x) + @abs(a.y - b.y);
        try testing.expectEqual(@as(u32, 1), step);
    }
}

test "weighted edges pick the cheaper route" {
    // Two-node-wide choice modeled directly: nodes are u8 ids, the searcher
    // proves it prefers the lower total cost even when more hops are involved.
    const Weighted = struct {
        pub fn heuristic(_: @This(), _: u8, _: u8) u32 {
            return 0; // zero heuristic -> behaves like Dijkstra, still optimal
        }
        pub fn neighbors(_: @This(), node: u8, out: AStar(u8, u32, @This()).Successors) !void {
            switch (node) {
                // Direct but expensive edge A->D costs 10.
                // Cheap detour A->B->C->D costs 1+1+1 = 3.
                'A' => {
                    try out.add('D', 10);
                    try out.add('B', 1);
                },
                'B' => try out.add('C', 1),
                'C' => try out.add('D', 1),
                else => {},
            }
        }
    };
    const Search = AStar(u8, u32, Weighted);
    const path = try Search.findPath(testing.allocator, .{}, 'A', 'D');
    try testing.expect(path != null);
    defer testing.allocator.free(path.?);
    // Cheapest route is the 4-node detour, not the direct edge.
    try testing.expectEqualSlices(u8, "ABCD", path.?);
}
