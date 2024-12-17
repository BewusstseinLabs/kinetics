// Copyright 2024 Bewusstsein Labs

use core::time;
use std::{
    fmt::Debug,
    ops::{ Deref, DerefMut, Sub, Mul, Div, AddAssign }
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
    pub fn new( force: [ T; DIM ] ) -> Self {
        Self( Vector::from( force ) )
    }

    pub fn x( &self ) -> &T where Assert<{ DIM >= 1 }>: IsTrue { &self.0[0] }
    pub fn y( &self ) -> &T where Assert<{ DIM >= 2 }>: IsTrue { &self.0[1] }
    pub fn z( &self ) -> &T where Assert<{ DIM >= 3 }>: IsTrue { &self.0[2] }

    pub fn apply_1st_ord( &self, body: &mut Body<T, DIM, 1>, time_step: T )
    where
        T: 'static + Default + Copy + Debug + Sub<Output = T> + Mul<Output = T> + Div<Output = T> + AddAssign,
    {
        let acceleration = self.0 / *body.mass(); // a = F / m
        *body.spatial_velocity_mut() += acceleration * time_step; // v = v + a * time_step
    }

    pub fn apply_2nd_ord( &self, body: &mut Body<T, DIM, 2> )
    where
        T: 'static + Default + Copy + Debug + Sub<Output = T> + Mul<Output = T> + Div<Output = T> + AddAssign,
    {
        let acceleration = self.0 / *body.mass(); // a = F / m
        *body.spatial_acceleration_mut() = acceleration;
    }

    pub fn apply<const ORD: usize>( &self, body: &mut Body<T, DIM, ORD>, time_step: T )
    where
        Assert<{ ORD >= 2 }>: IsTrue,
        Assert<{ ORD >= 3 }>: IsTrue,
        T: 'static + Default + Copy + Debug + Sub<Output = T> + Mul<Output = T> + Div<Output = T> + AddAssign,
        [(); ORD + 1]:
    {
        let acceleration = self.0 / *body.mass(); // a = F / m
        body.update_spatial_acceleration( acceleration, time_step );
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

impl <T, const DIM: usize> DerefMut for Force<T, DIM>
where
    T: 'static + Default + Copy + Debug
{
    fn deref_mut( &mut self ) -> &mut Self::Target {
        &mut self.0
    }
}

impl<T, const DIM: usize> From<Vector<T, DIM>> for Force<T, DIM>
where
    T: 'static + Default + Copy + Debug
{
    fn from( force: Vector<T, DIM> ) -> Self {
        Self( force )
    }
}

pub type Force1D<T> = Force<T, 1>;
pub type Force2D<T> = Force<T, 2>;
pub type Force3D<T> = Force<T, 3>;
pub type Force4D<T> = Force<T, 4>;

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_force() {
        let force = Force1D::<f32>::default();
        assert_eq!( *force.x(), 0.0 );
    }
}
