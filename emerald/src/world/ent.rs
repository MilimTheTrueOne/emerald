use hecs::Entity;
use serde::{Deserialize, Serialize};

use crate::{AssetLoader, EmeraldError, Transform, World};

use self::{ent_sprite_loader::load_ent_sprite, ent_transform_loader::load_ent_transform};
#[cfg(feature = "aseprite")]
pub(crate) mod ent_aseprite_loader;

pub(crate) mod ent_rigid_body_loader;
pub(crate) mod ent_sprite_loader;
pub(crate) mod ent_transform_loader;

const SPRITE_SCHEMA_KEY: &str = "sprite";
const RIGID_BODY_SCHEMA_KEY: &str = "rigid_body";
const ASEPRITE_SCHEMA_KEY: &str = "aseprite";
const TRANSFORM_SCHEMA_KEY: &str = "transform";

#[derive(Default)]
pub struct EntLoadConfig<'a> {
    pub transform: Transform,
    pub custom_component_loader: Option<
        &'a dyn Fn(
            &mut AssetLoader<'_>,
            Entity,
            &mut World,
            toml::Value,
            String,
        ) -> Result<(), EmeraldError>,
    >,
}

pub(crate) fn load_ent(
    loader: &mut AssetLoader<'_>,
    world: &mut World,
    toml: &mut crate::toml::Value,
    config: EntLoadConfig<'_>,
) -> Result<Entity, EmeraldError> {
    let entity = world.spawn((config.transform,));

    if let Some(table) = toml.as_table_mut() {
        let table_keys = table
            .keys()
            .into_iter()
            .map(|key| key.clone())
            .collect::<Vec<String>>();
        for key in table_keys {
            match key.as_str() {
                TRANSFORM_SCHEMA_KEY => {
                    if let Some(transform_value) = table.remove(TRANSFORM_SCHEMA_KEY) {
                        load_ent_transform(loader, entity, world, &transform_value)?;
                    }
                }
                SPRITE_SCHEMA_KEY => {
                    if let Some(sprite_value) = table.remove(SPRITE_SCHEMA_KEY) {
                        load_ent_sprite(loader, entity, world, &sprite_value)?;
                    }
                }
                RIGID_BODY_SCHEMA_KEY => {
                    if let Some(rigid_body_value) = table.remove(RIGID_BODY_SCHEMA_KEY) {
                        ent_rigid_body_loader::load_ent_rigid_body(
                            loader,
                            entity,
                            world,
                            &rigid_body_value,
                        )?;
                    }
                }
                ASEPRITE_SCHEMA_KEY => {
                    #[cfg(feature = "aseprite")]
                    {
                        if let Some(aseprite_value) = table.remove(ASEPRITE_SCHEMA_KEY) {
                            ent_aseprite_loader::load_ent_aseprite(
                                loader,
                                entity,
                                world,
                                &aseprite_value,
                            )?;
                        }
                    }
                }
                _ => {
                    if let Some(custom_component_loader) = config.custom_component_loader {
                        if let Some(value) = table.remove(&key) {
                            custom_component_loader(loader, entity, world, value, key)?;
                        }
                    }
                }
            }
        }
    }

    Ok(entity)
}

pub(crate) fn load_ent_from_toml(
    loader: &mut AssetLoader<'_>,
    world: &mut World,
    toml: String,
    config: EntLoadConfig<'_>,
) -> Result<Entity, EmeraldError> {
    let mut value = toml.parse::<toml::Value>()?;
    load_ent(loader, world, &mut value, config)
}

#[derive(Deserialize, Serialize)]
pub(crate) struct Vec2f32Schema {
    pub x: f32,
    pub y: f32,
}