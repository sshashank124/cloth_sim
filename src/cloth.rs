use std::cmp::Ordering::Equal;

use lazysort::SortedBy;
use nalgebra::zero;
// use nalgebra::geometry::Isometry3;
// use ncollide3d::{query::{closest_points, ClosestPoints}, shape::{Ball, Triangle}};
use rand::Rng;

use crate::{
    grid::{Grid, GridIdx},
    *,
};

// NUMBER OF STEPS IN ITERATIVE CONSTRAINT SOLVING
const CONSTRAINTS_ITER: I = 10;

// ENERGY DAMPING TO APPLY TO SYSTEM WHEN PERFORMING VERLET POSITION INTEGRATION
const DAMPING: F = 0.995;

// GRID RESOLUTION OF THE CLOTH: SUBDIVISIONS x SUBDIVISIONS
const SUBDIVISIONS: I = 30;

// THRESHOLD FOR COLLISION CHECKING
const EPSILON: F = 0.3;

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Particle {
    pub p: P,
    old_p: P,
    a: V,
    m: F,
    fixed: bool,
}

impl Particle {
    fn new(x: F, y: F, z: F) -> Self {
        let p = P::new(x, y, z);
        Self {
            p,
            old_p: p,
            a: zero(),
            m: 1.,
            fixed: false,
        }
    }

    fn add_force(&mut self, f: V) {
        self.a += f / self.m;
    }
    fn offset(&mut self, v: V) {
        if !self.fixed {
            self.p += v;
        }
    }

    fn step(&mut self) {
        if !self.fixed {
            let tmp = self.p;
            /* VERLET POSITION INTEGRATION */
            self.p += DAMPING * (self.p - self.old_p) + self.a * DT_SQ;
            self.old_p = tmp;
            self.a = zero();
        }
    }
}

struct Constraint {
    p1: GridIdx,
    p2: GridIdx,
    d: F,
}

impl Constraint {
    fn new(p1: GridIdx, p2: GridIdx, particles: &Grid<Particle>) -> Self {
        Self {
            p1,
            p2,
            d: (particles[p1].p - particles[p2].p).norm(),
        }
    }
}

pub struct Cloth {
    pub particles: Grid<Particle>,
    constraints: Vec<Constraint>,
    pub mesh_handle: Handle<Mesh>,
}

impl Cloth {
    pub fn new(width: F, height: F, mut meshes: ResMut<Assets<Mesh>>) -> (Self, Handle<Mesh>) {
        let parts: Vec<Particle> = (0..SUBDIVISIONS)
            .flat_map(|y| {
                (0..SUBDIVISIONS).map(move |x| {
                    Particle::new(
                        width * (x as F / SUBDIVISIONS as F),
                        -height * (y as F / SUBDIVISIONS as F),
                        (0.5 * height * (y as F / SUBDIVISIONS as F))
                            + rand::thread_rng().gen_range(20., 20.1),
                    )
                })
            })
            .collect();
        let mut particles = Grid::new(parts, SUBDIVISIONS);

        /* SET 4 CORNERS TO BE FIXED AT SIMULATION START */
        particles[(0, 0)].fixed = true;
        particles[(1, 0)].fixed = true;
        particles[(SUBDIVISIONS - 2, 0)].fixed = true;
        particles[(SUBDIVISIONS - 1, 0)].fixed = true;
        particles[(0, SUBDIVISIONS - 1)].fixed = true;
        particles[(1, SUBDIVISIONS - 1)].fixed = true;
        particles[(SUBDIVISIONS - 2, SUBDIVISIONS - 1)].fixed = true;
        particles[(SUBDIVISIONS - 1, SUBDIVISIONS - 1)].fixed = true;

        /* CREATE CONSTRAINTS (AKA SPRINGS IN THE MASS-SPRING SYSTEM) */
        let mut cs = vec![];
        for y in 0..SUBDIVISIONS {
            for x in 0..SUBDIVISIONS {
                /* STRUCTURAL SPRINGS */
                if x < SUBDIVISIONS - 1 {
                    cs.push(Constraint::new((x, y), (x + 1, y), &particles));
                }
                if y < SUBDIVISIONS - 1 {
                    cs.push(Constraint::new((x, y), (x, y + 1), &particles));
                }
                /* SHEAR SPRINGS */
                if x < SUBDIVISIONS - 1 && y < SUBDIVISIONS - 1 {
                    cs.push(Constraint::new((x, y), (x + 1, y + 1), &particles));
                    cs.push(Constraint::new((x + 1, y), (x, y + 1), &particles));
                }
                /* FLEXION SPRINGS */
                if x < SUBDIVISIONS - 2 {
                    cs.push(Constraint::new((x, y), (x + 2, y), &particles));
                }
                if y < SUBDIVISIONS - 2 {
                    cs.push(Constraint::new((x, y), (x, y + 2), &particles));
                }
            }
        }

        // PREPARE MESH
        let handle = meshes.add(Mesh::new(PrimitiveTopology::TriangleList));
        let mesh = meshes.get_mut(&handle).unwrap();

        let flatten = |x, y| (y * SUBDIVISIONS + x) as u32;
        let indices: Vec<u32> = (0..SUBDIVISIONS - 1)
            .flat_map(|y| {
                (0..SUBDIVISIONS - 1).flat_map(move |x| {
                    vec![
                        flatten(x, y),
                        flatten(x + 1, y),
                        flatten(x, y + 1),
                        flatten(x, y + 1),
                        flatten(x + 1, y),
                        flatten(x, y),
                        flatten(x + 1, y),
                        flatten(x + 1, y + 1),
                        flatten(x, y + 1),
                        flatten(x, y + 1),
                        flatten(x + 1, y + 1),
                        flatten(x + 1, y),
                    ]
                })
            })
            .collect();
        mesh.set_indices(Some(Indices::U32(indices)));

        let normals = vec![[0., 0., -1.]; SUBDIVISIONS * SUBDIVISIONS];
        mesh.set_attribute(Mesh::ATTRIBUTE_NORMAL, normals.into());

        let uv: Vec<[F; 2]> = (0..SUBDIVISIONS)
            .flat_map(|y| {
                (0..SUBDIVISIONS).map(move |x| {
                    [
                        x as F / (SUBDIVISIONS - 1) as F,
                        y as F / (SUBDIVISIONS - 1) as F,
                    ]
                })
            })
            .collect();
        mesh.set_attribute(Mesh::ATTRIBUTE_UV_0, uv.into());

        let cloth = Cloth {
            particles,
            constraints: cs,
            mesh_handle: handle.clone(),
        };
        cloth.update_mesh(mesh);

        (cloth, handle)
    }

    pub fn add_force(&mut self, force: V) {
        self.particles.iter_mut().for_each(|p| p.add_force(force));
    }

    pub fn step(&mut self) {
        /* ITERATIVELY RESOLVE SPRING CONSTRAINTS */
        for _ in 0..CONSTRAINTS_ITER {
            for constraint in &self.constraints {
                let p12 = self.particles[constraint.p2].p - self.particles[constraint.p1].p;
                let d = p12.norm();
                let f_c = DT * (d - constraint.d) * (p12 / d);
                self.particles[constraint.p1].offset(f_c);
                self.particles[constraint.p2].offset(-f_c);
            }
        }

        /* RESOLVE EXTERNAL FORCES ON THE PARTICLES */
        self.particles.iter_mut().for_each(Particle::step);

        let mut mods = vec![];

        /* EXPENSIVE POINT-FACE COLLISION CHECKING */
        /* SHOULD UPDATE EPSILON TO BE 0.1 FOR THIS MODE */
        /* ALSO WORKS CLOSE TO REAL-TIME WITH <20x20 GRID */
        /* UNCOMMENT THE 2 IMPORT STATEMENTS AT THE TOP FOR THIS MODE */
        // for (i, p) in self.particles.iter().enumerate() {
        //     for (p1, p2, p3) in (0..SUBDIVISIONS - 1)
        //         .flat_map(|y| {
        //             (0..SUBDIVISIONS - 1).flat_map(|x| {
        //                 vec![(self.particles[(x, y)].p.clone(),
        //                       self.particles[(x + 1, y)].p.clone(),
        //                       self.particles[(x, y + 1)].p.clone()),
        //                      (self.particles[(x, y + 1)].p.clone(),
        //                       self.particles[(x + 1, y)].p.clone(),
        //                       self.particles[(x, y)].p.clone()),
        //                 ]
        //             }).collect::<Vec<_>>()
        //         }) {
        //         let t = Triangle::new(p1, p2, p3);
        //         let res = closest_points(&Isometry3::translation(p.p.x, p.p.y, p.p.z), &Ball::new(EPSILON),
        //                                  &Isometry3::identity(), &t, EPSILON);

        //         if let ClosestPoints::WithinMargin(pa, pb) = res {
        //             let diff = pb - pa;
        //             let d = diff.norm();
        //             let ratio = EPSILON / d;
        //             let delta = diff * (1. - ratio);
        //             mods.push((i, delta));
        //         }
        //     }
        // }

        /* OR */
        /* CHEAP POINT-POINT COLLISION CHECKING */
        /* SHOULD UPDATE EPSILON TO BE 0.3 FOR THIS MODE */
        /* WORKS WELL IN REALTIME EVEN UP TO 50+ BY 50+ GRID */
        for (i1, p1) in self.particles.iter().enumerate() {
            for (i2, p2) in self.particles.iter().skip(i1).enumerate() {
                let diff = p2.p - p1.p;
                let d = diff.norm();
                if p1 != p2 && d < EPSILON {
                    let ratio = EPSILON / d;
                    let delta = diff * (1. - ratio);
                    mods.push((i1, delta));
                    mods.push((i2, -delta));
                }
            }
        }

        /* APPLY IMPULSE RESPONSES FOR ABOVE COMPUTED COLLISION CHECKS */
        for (idx, delta) in mods {
            self.particles.data[idx].offset(delta);
        }
    }

    /* SET 8 NEAREST POINTS TO SELECTED REGION TO BE FIXED IN SPACE */
    /* USES QUICKSORT LAZYSORING TO AVOID UNNECESSARY SORTING */
    pub fn set_fixed(&mut self, p: P, fixed: bool) {
        self.particles
            .iter_mut()
            .map(|v| ((v.p - p).norm_squared(), v))
            .sorted_by(|(a, _), (b, _)| a.partial_cmp(b).unwrap_or(Equal))
            .take(8)
            .for_each(|(_, v)| v.fixed = fixed);
    }

    pub fn update_mesh(&self, mesh: &mut Mesh) {
        let positions = self
            .particles
            .iter()
            .map(|p| [p.p.x, p.p.y, p.p.z])
            .collect::<Vec<_>>();
        mesh.set_attribute(Mesh::ATTRIBUTE_POSITION, positions.into());
    }
}
