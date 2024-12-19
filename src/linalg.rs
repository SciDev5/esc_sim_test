use std::{
    fmt::Debug,
    ops::{Add, AddAssign, Div, DivAssign, Index, IndexMut, Mul, MulAssign, Sub, SubAssign},
};

pub trait Field:
    Sized
    + Clone
    + Copy
    + Debug
    + Add<Self, Output = Self>
    + AddAssign<Self>
    + Sub<Self, Output = Self>
    + SubAssign<Self>
    + Mul<Self, Output = Self>
    + MulAssign<Self>
    + Div<Self, Output = Self>
    + DivAssign<Self>
    + From<i16>
    + From<f32>
{
    // fn from_i32(n: i32) -> Self;
}
macro_rules! impl_Field {
    ($($T: ident),*) => {$(
            impl Field for $T {
            // fn from_i32(n: i32) -> Self {
            //     n as Self
            // }
        }
    )*};
}
impl_Field!(f32, f64);

#[derive(Debug, Clone)]
pub struct Mat<T: Field> {
    n_rows: usize,
    n_cols: usize,
    data: Vec<T>,
}
macro_rules! _assert_square {
    ($mat:expr) => {
        assert_eq!($mat.n_rows, $mat.n_cols, "Matrix must be square.");
    };
}
impl<T: Field> Mat<T> {
    pub fn new<const ROWS: usize, const COLS: usize>(data: [[T; COLS]; ROWS]) -> Self {
        Self {
            n_cols: COLS,
            n_rows: ROWS,
            data: Vec::from_iter((0..COLS).flat_map(|j| (0..ROWS).map(move |i| data[i][j]))),
        }
    }
    pub fn from_raw(n_cols: usize, data: Vec<T>) {
        assert!(
            data.len() % n_cols == 0,
            "Raw data does not have correct number of elements for the number of rows"
        );
    }
    /// `i` is row number, `j` is column number
    fn raw_index(&self, i: usize, j: usize) -> usize {
        debug_assert!(i < self.n_rows && j < self.n_cols, "Index out of bounds.");
        i + j * self.n_rows
    }
    pub fn tr(&self) -> T {
        let mut accum: T = 0.into();
        for i in 0..usize::min(self.n_rows, self.n_cols) {
            accum += self[[i, i]];
        }
        accum
    }
    fn swap(&mut self, [i, j]: [usize; 2], [k, l]: [usize; 2]) {
        let i0 = self.raw_index(i, j);
        let i1 = self.raw_index(k, l);
        self.data.swap(i0, i1);
    }
    pub fn transpose(&mut self) {
        if self.n_rows == self.n_cols {
            let n = self.n_rows;
            for i in 0..n {
                for j in i + 1..n {
                    self.swap([i, j], [j, i]);
                }
            }
        }
        todo!("transpose rectangular matrices");
    }
    pub fn t(mut self) -> Self {
        self.transpose();
        self
    }
    pub fn inverse(&mut self) {
        _assert_square!(self);
        todo!("inverse");
    }
    pub fn i(mut self) -> Self {
        self.inverse();
        self
    }

    pub fn matmul(&self, rhs: &Self) -> Self {
        assert_eq!(
            self.n_cols, rhs.n_rows,
            "Matrix dimensions are not compatible for matmul."
        );
        let mut out = Mat {
            n_cols: rhs.n_cols,
            n_rows: self.n_rows,
            data: Vec::with_capacity(self.n_rows * rhs.n_cols),
        };

        for i in 0..self.n_rows {
            for j in 0..rhs.n_cols {
                let mut accum = 0.into();
                for k in 0..self.n_cols {
                    accum += self[[i, k]] * rhs[[k, j]];
                }
                out.data.push(accum);
            }
        }

        out
    }
    pub fn to_scalar(self) -> T {
        assert_eq!(
            self.n_cols, 1,
            "Matrix is not 1x1 and so may not be converted to scalar."
        );
        assert_eq!(
            self.n_rows, 1,
            "Matrix is not 1x1 and so may not be converted to scalar."
        );
        self.data[0]
    }
}
impl<T: Field> Index<[usize; 2]> for Mat<T> {
    type Output = T;
    fn index(&self, [i, j]: [usize; 2]) -> &Self::Output {
        &self.data[self.raw_index(i, j)]
    }
}
impl<T: Field> IndexMut<[usize; 2]> for Mat<T> {
    fn index_mut(&mut self, [i, j]: [usize; 2]) -> &mut Self::Output {
        let k = self.raw_index(i, j);
        &mut self.data[k]
    }
}
impl<T: Field> Add<Self> for Mat<T> {
    type Output = Self;
    fn add(mut self, rhs: Self) -> Self::Output {
        assert_eq!(self.n_rows, rhs.n_rows);
        assert_eq!(self.n_cols, rhs.n_cols);
        for i in 0..self.data.len() {
            self.data[i] += rhs.data[i];
        }
        self
    }
}
impl<T: Field> Sub<Self> for Mat<T> {
    type Output = Self;
    fn sub(mut self, rhs: Self) -> Self::Output {
        assert_eq!(self.n_rows, rhs.n_rows);
        assert_eq!(self.n_cols, rhs.n_cols);
        for i in 0..self.data.len() {
            self.data[i] -= rhs.data[i];
        }
        self
    }
}
impl<T: Field> AddAssign<Self> for Mat<T> {
    fn add_assign(&mut self, rhs: Self) {
        assert_eq!(self.n_rows, rhs.n_rows);
        assert_eq!(self.n_cols, rhs.n_cols);
        for i in 0..self.data.len() {
            self.data[i] += rhs.data[i];
        }
    }
}
impl<T: Field> SubAssign<Self> for Mat<T> {
    fn sub_assign(&mut self, rhs: Self) {
        assert_eq!(self.n_rows, rhs.n_rows);
        assert_eq!(self.n_cols, rhs.n_cols);
        for i in 0..self.data.len() {
            self.data[i] -= rhs.data[i];
        }
    }
}
impl<T: Field> MulAssign<T> for Mat<T> {
    fn mul_assign(&mut self, rhs: T) {
        for i in 0..self.data.len() {
            self.data[i] *= rhs;
        }
    }
}
impl<T: Field> DivAssign<T> for Mat<T> {
    fn div_assign(&mut self, rhs: T) {
        for i in 0..self.data.len() {
            self.data[i] /= rhs;
        }
    }
}

// matmul
impl<T: Field> Mul<Self> for Mat<T> {
    type Output = Self;
    fn mul(self, rhs: Self) -> Self::Output {
        self.matmul(&rhs)
    }
}
impl<T: Field> Mul<Self> for &Mat<T> {
    type Output = Mat<T>;
    fn mul(self, rhs: Self) -> Self::Output {
        self.matmul(rhs)
    }
}
