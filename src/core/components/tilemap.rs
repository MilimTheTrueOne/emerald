use std::collections::HashMap;

use crate::*;

#[cfg(feature = "physics")]
use rapier2d::prelude::{ConvexPolygon, ShapeType};

#[derive(Clone, Copy, Debug, Hash, PartialEq, Eq)]
pub struct TilesheetId(pub(crate) usize, pub(crate) usize);
impl TilesheetId {
    /// Corresponds to a tile in a tilesheet, starting from the bottom-left corner.
    pub fn new(x: usize, y: usize) -> Self {
        Self(x, y)
    }
}

#[derive(Clone)]
pub struct Tilemap {
    pub(crate) width: usize,
    pub(crate) height: usize,
    pub(crate) tilesheet: TextureKey,
    pub(crate) tile_size: Vector2<usize>,
    pub(crate) tiles: Vec<Option<TilesheetId>>,
    pub z_index: f32,
    pub visible: bool,

    #[cfg(feature = "physics")]
    pub(crate) tilesheet_colliders: HashMap<TilesheetId, ConvexPolygon>,
}
impl Tilemap {
    pub fn new(
        tilesheet: TextureKey,
        tile_size: Vector2<usize>,
        width: usize,
        height: usize,
    ) -> Self {
        let mut tiles = Vec::with_capacity(width * height);

        for _ in 0..(width * height) {
            tiles.push(None);
        }

        Tilemap {
            tile_size,
            tilesheet,
            height,
            width,
            tiles,
            z_index: 0.0,
            visible: true,

            #[cfg(feature = "physics")]
            tilesheet_colliders: HashMap::new(),
        }
    }

    pub fn get_tile(&self, x: usize, y: usize) -> Result<Option<TilesheetId>, EmeraldError> {
        let tile_index = self.get_index(x, y)?;

        if let Some(tile) = self.tiles.get(tile_index) {
            let tile = tile.map(|id| id);

            return Ok(tile);
        }

        let err_msg = format!(
            "Position {:?} does not exist. Tilemap size is {}",
            (x, y),
            self.size()
        );

        Err(EmeraldError::new(err_msg))
    }

    pub fn set_tile(
        &mut self,
        x: usize,
        y: usize,
        new_tile: Option<TilesheetId>,
    ) -> Result<(), EmeraldError> {
        let tile_index = self.get_index(x, y)?;

        if let Some(tile_id) = self.tiles.get_mut(tile_index) {
            *tile_id = new_tile;

            return Ok(());
        }

        let err_msg = format!(
            "Position {:?} does not exist. Tilemap size is {}",
            (x, y),
            self.size()
        );

        Err(EmeraldError::new(err_msg))
    }

    pub fn size(&self) -> Vector2<usize> {
        Vector2::new(self.width, self.height)
    }

    pub fn width(&self) -> usize {
        self.width
    }

    pub fn height(&self) -> usize {
        self.height
    }

    pub fn set_tilesheet(&mut self, tilesheet: TextureKey) {
        self.tilesheet = tilesheet
    }

    pub(crate) fn get_index(&self, x: usize, y: usize) -> Result<usize, EmeraldError> {
        get_tile_index(x, y, self.width, self.height)
    }

    /// Sets a collider for the given tile on the Tilesheet for this Tilemap.
    /// Does not bake the tilemap. You must re-bake the tilemap for it to take effect.
    #[cfg(feature = "physics")]
    pub fn set_tilesheet_collider_polygon(
        &mut self,
        tilesheet_id: TilesheetId,
        convex_polygon: ConvexPolygon,
    ) -> Result<(), EmeraldError> {
        self.tilesheet_colliders
            .insert(tilesheet_id, convex_polygon);

        Ok(())
    }

    /// Borrows a collider for the given tile on the Tilesheet for this Tilemap.
    #[cfg(feature = "physics")]
    pub fn get_tilesheet_collider_polygon(
        &self,
        tilesheet_id: TilesheetId,
    ) -> Result<Option<&ConvexPolygon>, EmeraldError> {
        Ok(self.tilesheet_colliders.get(&tilesheet_id))
    }

    /// Sets a collider for the given tile on the Tilesheet for this Tilemap.
    /// Does not bake the tilemap. You must re-bake the tilemap for it to take effect.
    #[cfg(feature = "physics")]
    pub fn remove_tilesheet_collider_polygon(
        &mut self,
        tilesheet_id: TilesheetId,
    ) -> Result<Option<ConvexPolygon>, EmeraldError> {
        Ok(self.tilesheet_colliders.remove(&tilesheet_id))
    }
}

/// Get the tile index of a grid from the bottom-left corner.
pub(crate) fn get_tile_index(
    x: usize,
    y: usize,
    width: usize,
    height: usize,
) -> Result<usize, EmeraldError> {
    if x >= width {
        return Err(EmeraldError::new(format!(
            "Given x: {} is outside the width of {}",
            x, width
        )));
    }

    if y >= height {
        return Err(EmeraldError::new(format!(
            "Given y: {} is outside the height of {}",
            y, height
        )));
    }

    Ok((y * width) + x)
}

#[cfg(feature = "physics")]
mod physics {
    use geo_clipper::*;
    use geo_types::{Coordinate, LineString, Polygon};
    use nalgebra::{Point, Point2, Vector2};
    use rapier2d::prelude::{Collider, ColliderBuilder, RigidBodyBuilder};

    use crate::{tilemap::Tilemap, EmeraldError, Entity, World};

    /// Unions all possible colliders on the tiles in the map, then bakes it all into one Static Body on the Tilemap Entity.
    /// Warning: Destroys the current body for this entity.
    pub fn bake_tilemap_entity(
        tilemap_entity: Entity,
        world: &mut World,
    ) -> Result<(), EmeraldError> {
        world.physics().remove_body(tilemap_entity);
        let mut polygons;

        match world.get_mut::<Tilemap>(tilemap_entity) {
            Err(_) => {
                return Err(EmeraldError::new(format!(
                    "Entity {:?} does not contain a Tilemap, cannot bake it.",
                    tilemap_entity
                )))
            }
            Ok(tilemap) => {
                polygons = get_all_tile_collider_polygons(&tilemap);
            }
        }

        let rbh = world
            .physics()
            .build_body(tilemap_entity, RigidBodyBuilder::new_static())?;

        if polygons.len() == 0 {
            return Ok(());
        }

        let mut final_polygons = vec![polygons.remove(0)];
        bake_polygons(&mut final_polygons, polygons);

        for polygon in final_polygons {
            if let Some(collider_builder) = get_collider_builder_from_polygon(polygon) {
                world.physics().build_collider(rbh, collider_builder);
            }
        }

        Ok(())
    }

    fn get_collider_builder_from_polygon(polygon: Polygon<f64>) -> Option<ColliderBuilder> {
        ColliderBuilder::convex_polyline(
            polygon
                .exterior()
                .points()
                .map(|point| Point2::new(point.x() as f32, point.y() as f32))
                .collect::<Vec<Point2<f32>>>(),
        )
    }

    /// Bake all other polygons into the final polygon list.
    fn bake_polygons(
        final_polygons: &mut Vec<Polygon<f64>>,
        mut other_polygons: Vec<Polygon<f64>>,
    ) {
        while other_polygons.len() > 0 {
            let other_polygon = other_polygons.remove(0);
            bake_polygon(final_polygons, other_polygon);
        }
    }

    /// Bake other polygon into final polygon list.
    fn bake_polygon(final_polygons: &mut Vec<Polygon<f64>>, mut other_polygon: Polygon<f64>) {
        let mut i = 0;
        let mut merge_result = None;

        for final_polygon in final_polygons.iter_mut() {
            let mut result = final_polygon.union(&mut other_polygon, 1.0);

            if result.0.len() == 1 {
                merge_result = Some(result.0.remove(0));
                break;
            }

            i += 1;
        }

        if let Some(new_polygon) = merge_result {
            final_polygons.remove(i);
            final_polygons.insert(i, new_polygon);
        }
    }

    /// Gets all of the colliders of tile instances, then maps them into a Vec of (x, y) points
    fn get_all_tile_collider_polygons(tilemap: &Tilemap) -> Vec<Polygon<f64>> {
        let mut polygons = Vec::new();

        for x in 0..tilemap.width() {
            for y in 0..tilemap.height() {
                if let Ok(Some(tilesheet_id)) = tilemap.get_tile(x, y) {
                    if let Ok(Some(polygon)) = tilemap.get_tilesheet_collider_polygon(tilesheet_id)
                    {
                        let real_points = polygon.points();

                        let coordinates = real_points
                            .iter()
                            .map(|real_points| Coordinate {
                                x: real_points[0] as f64,
                                y: real_points[1] as f64,
                            })
                            .collect::<Vec<Coordinate<f64>>>();

                        let polygon = Polygon::new(LineString::new(coordinates), vec![]);

                        polygons.push(polygon);
                    }
                }
            }
        }

        polygons
    }
}
