mod cloth;
mod grid;

use bevy::{
    prelude::*,
    render::{mesh::Indices, pipeline::PrimitiveTopology},
};
use bevy_mod_picking::*;
use nalgebra::{geometry::Point3, Vector3};
use rand::Rng;

use cloth::Cloth;

type I = usize;
type F = f32;
type P = Point3<F>;
type V = Vector3<F>;


/* TIME-STEP SIZE */
const DT: F = 0.05;
const DT_SQ: F = DT * DT;

const IMAGE_PATH: &str = "/home/phaqlow/projects/cloth_sim/assets/texture.png";

fn main() {
    App::build()
        .add_plugins(DefaultPlugins)
        .add_plugin(PickingPlugin)
        .add_startup_system(setup.system())
        .add_system(step.system())
        .add_system(interact.system())
        .run();
}

/* SIMULATE SINGLE STEP */
fn step(mut meshes: ResMut<Assets<Mesh>>, mut cloth: Mut<Cloth>) {
    // add gravity (negative y direction)
    cloth.add_force(V::new(0., -0.2, 0.));

    // add some random crosswind
    cloth.add_force(V::new(0., 0., rand::thread_rng().gen_range(-0.1, 0.1)));

    // simulate single step
    cloth.step();

    // update mesh for displaying based on the simulated step
    let mesh = meshes.get_mut(&cloth.mesh_handle).unwrap();
    cloth.update_mesh(mesh);
}

/* BOILERPLATE CODE FOR UI INITIALIZATION AND INTERACTION */

fn interact(mbi: Res<Input<MouseButton>>, (mut cloth, entity): (Mut<Cloth>, &PickableMesh)) {
    let lmb = mbi.pressed(MouseButton::Left);
    let rmb = mbi.pressed(MouseButton::Right);
    if !lmb && !rmb {
        return;
    }

    if let Some(it) = entity.intersection(&Group::default()).unwrap() {
        let p = it.position();
        let p = P::new(p.x(), p.y(), p.z());
        cloth.set_fixed(p, lmb);
    }
}

fn setup(
    mut commands: Commands,
    meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
    asset_server: Res<AssetServer>,
) {
    let (cloth, mesh_handle) = Cloth::new(10., 12., meshes);

    commands
        .spawn(PbrComponents {
            mesh: mesh_handle,
            material: materials.add(asset_server.load(IMAGE_PATH).into()),
            ..Default::default()
        })
        .with(cloth)
        .with(PickableMesh::default())
        .spawn(LightComponents {
            transform: Transform::from_translation(Vec3::new(6., -6., 15.)),
            light: Light {
                color: Color::rgb(2., 2., 2.),
                ..Default::default()
            },
            ..Default::default()
        })
        .spawn(Camera3dComponents {
            transform: Transform::from_translation(Vec3::new(-5., 0., 0.))
                .looking_at(Vec3::new(2., -6., 20.), Vec3::unit_y()),
            ..Default::default()
        })
        .with(PickSource::default());
}
