// Copyright 2024 Bewusstsein Labs

use core::time;
use std::{
    fmt::Debug,
    ops::{ Deref, DerefMut, Sub, Mul, Div, AddAssign }
};

use const_expr_bounds::{ Assert, IsTrue };
use linear_algebra::vector::Vector;
use kinematics::body::Body;

#[derive( Clone, Default, Debug, PartialEq )]
pub struct Torque<T, const DIM: usize>( Vector<T, DIM> )
where
    T: 'static + Default + Copy + Debug;

impl<T, const DIM: usize> Torque<T, DIM>
where
    T: 'static + Default + Copy + Debug
{
    pub fn new( torque: [ T; DIM ] ) -> Self {
        Self( Vector::from( torque ) )
    }

    pub fn u( &self ) -> &T where Assert<{ DIM >= 1 }>: IsTrue { &self.0[0] }
    pub fn v( &self ) -> &T where Assert<{ DIM >= 2 }>: IsTrue { &self.0[1] }
    pub fn w( &self ) -> &T where Assert<{ DIM >= 3 }>: IsTrue { &self.0[2] }

    pub fn apply_1st_ord( &self, body: &mut Body<T, DIM, 1>, time_step: T )
    where
        T: 'static + Default + Copy + Debug + Sub<Output = T> + Mul<Output = T> + Div<Output = T> + AddAssign,
    {
        let acceleration = self.0 / *body.mass(); // a = F / m
        *body.angular_velocity_mut() += acceleration * time_step; // v = v + a * time_step
    }

    pub fn apply_2nd_ord( &self, body: &mut Body<T, DIM, 2> )
    where
        T: 'static + Default + Copy + Debug + Sub<Output = T> + Mul<Output = T> + Div<Output = T> + AddAssign,
    {
        let acceleration = self.0 / *body.mass(); // a = F / m
        *body.angular_acceleration_mut() = acceleration;
    }

    pub fn apply<const ORD: usize>( &self, body: &mut Body<T, DIM, ORD>, time_step: T )
    where
        Assert<{ ORD >= 2 }>: IsTrue,
        Assert<{ ORD >= 3 }>: IsTrue,
        T: 'static + Default + Copy + Debug + Sub<Output = T> + Mul<Output = T> + Div<Output = T> + AddAssign,
        [(); ORD + 1]:
    {
        let acceleration = self.0 / *body.mass(); // a = F / m
        body.update_angular_acceleration( acceleration, time_step );
    }
}

impl<T, const DIM: usize> Deref for Torque<T, DIM>
where
    T: 'static + Default + Copy + Debug
{
    type Target = Vector<T, DIM>;

    fn deref( &self ) -> &Self::Target {
        &self.0
    }
}

impl <T, const DIM: usize> DerefMut for Torque<T, DIM>
where
    T: 'static + Default + Copy + Debug
{
    fn deref_mut( &mut self ) -> &mut Self::Target {
        &mut self.0
    }
}

impl<T, const DIM: usize> From<Vector<T, DIM>> for Torque<T, DIM>
where
    T: 'static + Default + Copy + Debug
{
    fn from( force: Vector<T, DIM> ) -> Self {
        Self( force )
    }
}

pub type Torque1D<T> = Torque<T, 1>;
pub type Torque2D<T> = Torque<T, 2>;
pub type Torque3D<T> = Torque<T, 3>;
pub type Torque4D<T> = Torque<T, 4>;

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_torque() {
        let force = Torque1D::<f32>::default();
        assert_eq!( *force.u(), 0.0 );
    }
}
