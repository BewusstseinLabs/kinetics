// Copyright 2024 Bewusstsein Labs

use core::time;
use std::{
    fmt::Debug,
    ops::{ Deref, Mul, Div, AddAssign }
};

use linear_algebra::vector::Vector;

use kinematics::{ body::Body, Assert, IsTrue };

#[derive( Clone, Default, Debug, PartialEq )]
pub struct Force<T, const DIM: usize>( Vector<T, DIM> )
where
    T: 'static + Default + Copy + Debug;

impl<T, const DIM: usize> Force<T, DIM>
where
    T: 'static + Default + Copy + Debug
{
    pub fn new( force: Vector<T, DIM> ) -> Self {
        Self( force )
    }

    pub fn x( &self ) -> &T where Assert<{ DIM >= 1 }>: IsTrue { &self.0[0] }
    pub fn y( &self ) -> &T where Assert<{ DIM >= 2 }>: IsTrue { &self.0[1] }
    pub fn z( &self ) -> &T where Assert<{ DIM >= 3 }>: IsTrue { &self.0[2] }

    pub fn apply<const ORD: usize>( &self, body: &mut Body<T, DIM, ORD>, time_step: T )
    where
        Assert<{ ORD >= 1 }>: IsTrue,
        T: 'static + Default + Copy + Debug + Mul<Output = T> + Div<Output = T> + AddAssign,
        [(); ORD + 1]:
    {
        for i in 0..DIM {
            let acceleration = self.0[i] / *body.mass(); // a = F / m
            body.spatial_velocity_mut()[i] += time_step * acceleration; // v = v + a * time_step
        }
    }
}

impl<T, const DIM: usize> Deref for Force<T, DIM>
where
    T: 'static + Default + Copy + Debug
{
    type Target = Vector<T, DIM>;

    fn deref( &self ) -> &Self::Target {
        &self.0
    }
}

pub type Force1D<T> = Force<T, 1>;
pub type Force2D<T> = Force<T, 2>;
pub type Force3D<T> = Force<T, 3>;
pub type Force4D<T> = Force<T, 4>;
