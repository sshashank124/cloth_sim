use std::ops::{Deref, DerefMut, Index, IndexMut};

use crate::I;

/* CONVENIENCE CLASS FOR WORKING WITH A FLATTENED 2-D GRID */

pub type GridIdx = (I, I);

pub struct Grid<T> {
    pub data: Vec<T>,
    width: I,
}

impl<T> Grid<T> {
    pub fn new(data: Vec<T>, width: I) -> Self { Self { data, width } }
}

impl<T> Deref for Grid<T> {
    type Target = Vec<T>;
    fn deref(&self) -> &Self::Target { &self.data }

}

impl<T> DerefMut for Grid<T> {
    fn deref_mut(&mut self) -> &mut Self::Target { &mut self.data }

}

impl<T> Index<GridIdx> for Grid<T> {
    type Output = T;

    fn index(&self, idx: GridIdx) -> &Self::Output {
        let pos = idx.1 * self.width + idx.0;
        &self.data[pos]
    }
}

impl<T> IndexMut<GridIdx> for Grid<T> {
    fn index_mut(&mut self, idx: GridIdx) -> &mut Self::Output {
        let pos = idx.1 * self.width + idx.0;
        &mut self.data[pos]
    }
}
