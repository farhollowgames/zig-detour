//! The integer variant: the navigation mesh and its queries on whole
//! millimetres, with no floating-point step, so every machine gives the
//! same bytes for the same call.
//!
//! The files mirror the float port's, name for name. `doc/integer-design.md`
//! gives the units, the bounds that keep every product inside its type, and
//! how the variant is checked against the float port.

const std = @import("std");

pub const common = @import("integer/common.zig");
pub const nav_mesh = @import("integer/nav_mesh.zig");
pub const node = @import("integer/node.zig");
pub const nav_mesh_query = @import("integer/nav_mesh_query.zig");
pub const convert = @import("integer/convert.zig");

pub const Fraction = common.Fraction;

pub const PolyRef = nav_mesh.PolyRef;
pub const TileRef = nav_mesh.TileRef;
pub const NavMesh = nav_mesh.NavMesh;
pub const NavMeshParams = nav_mesh.NavMeshParams;
pub const LinkOrder = nav_mesh.LinkOrder;
pub const MeshTile = nav_mesh.MeshTile;
pub const MeshHeader = nav_mesh.MeshHeader;

pub const NavMeshQuery = nav_mesh_query.NavMeshQuery;
pub const QueryFilter = nav_mesh_query.QueryFilter;
pub const RaycastHit = nav_mesh_query.RaycastHit;
pub const PolyQuery = nav_mesh_query.PolyQuery;
pub const Node = node.Node;
pub const NodePool = node.NodePool;
pub const NodeQueue = node.NodeQueue;

pub const tileFromFloat = convert.tileFromFloat;

test {
    std.testing.refAllDecls(common);
    std.testing.refAllDecls(nav_mesh);
    std.testing.refAllDecls(node);
    std.testing.refAllDecls(nav_mesh_query);
    std.testing.refAllDecls(convert);
    std.testing.refAllDecls(Fraction);
    std.testing.refAllDecls(NavMesh);
    std.testing.refAllDecls(MeshTile);
    std.testing.refAllDecls(NavMeshQuery);
    std.testing.refAllDecls(QueryFilter);
    std.testing.refAllDecls(RaycastHit);
    std.testing.refAllDecls(NodePool);
    std.testing.refAllDecls(NodeQueue);
}
