// klipper_host_rust/src/mathutil.rs
// Corresponds to klippy/mathutil.py - Math utility functions.

pub type Vector3d = [f64; 3];
pub type Matrix3x3d = [[f64; 3]; 3];

// Corresponds to Python's matrix_cross(m1, m2)
pub fn vector_cross(v1: Vector3d, v2: Vector3d) -> Vector3d {
    [
        v1[1] * v2[2] - v1[2] * v2[1],
        v1[2] * v2[0] - v1[0] * v2[2],
        v1[0] * v2[1] - v1[1] * v2[0],
    ]
}

// Corresponds to Python's matrix_dot(m1, m2)
pub fn vector_dot(v1: Vector3d, v2: Vector3d) -> f64 {
    v1[0] * v2[0] + v1[1] * v2[1] + v1[2] * v2[2]
}

// Corresponds to Python's matrix_magsq(m1)
pub fn vector_mag_sq(v: Vector3d) -> f64 {
    v[0].powi(2) + v[1].powi(2) + v[2].powi(2)
}

// Corresponds to Python's matrix_add(m1, m2)
// Note: The commented out stub was named vector_add already, but with underscores.
// Standard Rust naming convention is snake_case for functions.
pub fn vector_add(v1: Vector3d, v2: Vector3d) -> Vector3d {
    [v1[0] + v2[0], v1[1] + v2[1], v1[2] + v2[2]]
}

// Corresponds to Python's matrix_sub(m1, m2)
pub fn vector_sub(v1: Vector3d, v2: Vector3d) -> Vector3d {
    [v1[0] - v2[0], v1[1] - v2[1], v1[2] - v2[2]]
}

// Corresponds to Python's matrix_mul(m1, s)
pub fn vector_mul_scalar(v: Vector3d, s: f64) -> Vector3d {
    [v[0] * s, v[1] * s, v[2] * s]
}

// Corresponds to Python's matrix_det(a)
pub fn matrix_determinant(a: Matrix3x3d) -> f64 {
    // x0, x1, x2 = a
    // return matrix_dot(x0, matrix_cross(x1, x2))
    let x0 = a[0];
    let x1 = a[1];
    let x2 = a[2];
    vector_dot(x0, vector_cross(x1, x2))
}

// Corresponds to Python's matrix_inv(a)
pub fn matrix_inverse(a: Matrix3x3d) -> Option<Matrix3x3d> {
    // x0, x1, x2 = a
    // inv_det = 1. / matrix_det(a)
    // return [matrix_mul(matrix_cross(x1, x2), inv_det),
    //         matrix_mul(matrix_cross(x2, x0), inv_det),
    //         matrix_mul(matrix_cross(x0, x1), inv_det)]
    let det = matrix_determinant(a);
    if det.abs() < 1e-9 { // Consider determinant as zero if it's very small
        return None; // Matrix is singular, inverse does not exist
    }

    let inv_det = 1.0 / det;

    let x0 = a[0];
    let x1 = a[1];
    let x2 = a[2];

    let res_row0 = vector_mul_scalar(vector_cross(x1, x2), inv_det);
    let res_row1 = vector_mul_scalar(vector_cross(x2, x0), inv_det);
    let res_row2 = vector_mul_scalar(vector_cross(x0, x1), inv_det);

    // The result from python code gives rows of the inverse matrix.
    // To construct the inverse matrix, these rows need to be columns.
    // So, we transpose the result.
    Some([
        [res_row0[0], res_row1[0], res_row2[0]],
        [res_row0[1], res_row1[1], res_row2[1]],
        [res_row0[2], res_row1[2], res_row2[2]],
    ])
}

// pub fn solve_quadratic(a: f64, b: f64, c: f64) -> Vec<f64> {
//     // ...
//     Vec::new()
// }

// ... other math utilities (rotation_matrix, homing_coord, etc.)

#[cfg(test)]
mod tests {
    use super::*; // Make functions and types from outer module available

    const EPSILON: f64 = 1e-9; // A small epsilon for float comparisons

    fn assert_vec_equals(v1: Vector3d, v2: Vector3d, msg: &str) {
        assert!((v1[0] - v2[0]).abs() < EPSILON &&
                (v1[1] - v2[1]).abs() < EPSILON &&
                (v1[2] - v2[2]).abs() < EPSILON,
                "{} expected {:?}, got {:?}", msg, v2, v1);
    }

    #[test]
    fn test_vector_cross() {
        let v1 = [1.0, 0.0, 0.0];
        let v2 = [0.0, 1.0, 0.0];
        let expected = [0.0, 0.0, 1.0];
        assert_vec_equals(vector_cross(v1, v2), expected, "vector_cross unit vectors");

        let v3 = [1.0, 2.0, 3.0];
        let v4 = [4.0, 5.0, 6.0];
        let expected_v3v4 = [-3.0, 6.0, -3.0]; // (2*6 - 3*5), (3*4 - 1*6), (1*5 - 2*4)
        assert_vec_equals(vector_cross(v3, v4), expected_v3v4, "vector_cross general vectors");
    }

    #[test]
    fn test_vector_dot() {
        let v1 = [1.0, 2.0, 3.0];
        let v2 = [4.0, 5.0, 6.0];
        let expected = 1.0*4.0 + 2.0*5.0 + 3.0*6.0; // 4 + 10 + 18 = 32
        assert!((vector_dot(v1, v2) - expected).abs() < EPSILON, "vector_dot product");

        let v3 = [1.0, 0.0, 0.0];
        let v4 = [0.0, 1.0, 0.0];
        assert!((vector_dot(v3, v4) - 0.0).abs() < EPSILON, "vector_dot orthogonal");
    }

    #[test]
    fn test_vector_mag_sq() {
        let v1 = [1.0, 2.0, 3.0];
        let expected = 1.0*1.0 + 2.0*2.0 + 3.0*3.0; // 1 + 4 + 9 = 14
        assert!((vector_mag_sq(v1) - expected).abs() < EPSILON, "vector_mag_sq");

        let v2 = [0.0, 0.0, 0.0];
        assert!((vector_mag_sq(v2) - 0.0).abs() < EPSILON, "vector_mag_sq zero vector");
    }

    #[test]
    fn test_vector_add() {
        let v1 = [1.0, 2.0, 3.0];
        let v2 = [4.0, 5.0, 6.0];
        let expected = [5.0, 7.0, 9.0];
        assert_vec_equals(vector_add(v1, v2), expected, "vector_add");
    }

    #[test]
    fn test_vector_sub() {
        let v1 = [4.0, 5.0, 6.0];
        let v2 = [1.0, 2.0, 3.0];
        let expected = [3.0, 3.0, 3.0];
        assert_vec_equals(vector_sub(v1, v2), expected, "vector_sub");
    }

    #[test]
    fn test_vector_mul_scalar() {
        let v1 = [1.0, 2.0, 3.0];
        let s = 3.0;
        let expected = [3.0, 6.0, 9.0];
        assert_vec_equals(vector_mul_scalar(v1, s), expected, "vector_mul_scalar");

        let v2 = [1.0, 2.0, 3.0];
        let s_zero = 0.0;
        let expected_zero = [0.0, 0.0, 0.0];
        assert_vec_equals(vector_mul_scalar(v2, s_zero), expected_zero, "vector_mul_scalar by zero");
    }

    fn assert_matrix_equals(m1: Matrix3x3d, m2: Matrix3x3d, msg: &str) {
        for i in 0..3 {
            assert_vec_equals(m1[i], m2[i], &format!("{} (row {})", msg, i));
        }
    }

    #[test]
    fn test_matrix_determinant() {
        let m1 = [[1.0, 2.0, 3.0], [0.0, 1.0, 4.0], [5.0, 6.0, 0.0]];
        // Det = 1*(1*0 - 4*6) - 2*(0*0 - 4*5) + 3*(0*6 - 1*5)
        //     = 1*(-24) - 2*(-20) + 3*(-5)
        //     = -24 + 40 - 15 = 1
        let expected_det1 = 1.0;
        assert!((matrix_determinant(m1) - expected_det1).abs() < EPSILON, "matrix_determinant m1");

        let m2 = [[2.0, 0.0, 0.0], [0.0, 2.0, 0.0], [0.0, 0.0, 2.0]]; // Diagonal matrix
        let expected_det2 = 8.0;
        assert!((matrix_determinant(m2) - expected_det2).abs() < EPSILON, "matrix_determinant diagonal");

        let m3 = [[1.0, 2.0, 3.0], [1.0, 2.0, 3.0], [4.0, 5.0, 6.0]]; // Singular matrix (row 1 == row 2)
        let expected_det3 = 0.0;
        assert!((matrix_determinant(m3) - expected_det3).abs() < EPSILON, "matrix_determinant singular");
    }

    #[test]
    fn test_matrix_inverse() {
        let m1 = [[1.0, 2.0, 3.0], [0.0, 1.0, 4.0], [5.0, 6.0, 0.0]];
        // From online calculator, inverse is:
        // [[-24.0, 18.0,  5.0],
        //  [ 20.0,-15.0, -4.0],
        //  [ -5.0,  4.0,  1.0]]
        let expected_inv_m1 = [
            [-24.0, 18.0, 5.0],
            [20.0, -15.0, -4.0],
            [-5.0, 4.0, 1.0],
        ];
        match matrix_inverse(m1) {
            Some(inv_m1) => assert_matrix_equals(inv_m1, expected_inv_m1, "matrix_inverse m1"),
            None => panic!("matrix_inverse m1 returned None, expected a matrix"),
        }

        let m_identity = [[1.0,0.0,0.0],[0.0,1.0,0.0],[0.0,0.0,1.0]];
        match matrix_inverse(m_identity) {
            Some(inv_identity) => assert_matrix_equals(inv_identity, m_identity, "matrix_inverse identity"),
            None => panic!("matrix_inverse identity returned None"),
        }

        let m_singular = [[1.0, 2.0, 3.0], [1.0, 2.0, 3.0], [4.0, 5.0, 6.0]];
        assert!(matrix_inverse(m_singular).is_none(), "matrix_inverse singular should return None");

        // Test with a matrix whose inverse is known
        let m_test = [[2.0, 0.0, 0.0], [0.0, 0.5, 0.0], [0.0, 0.0, 4.0]];
        let expected_inv_test = [[0.5, 0.0, 0.0], [0.0, 2.0, 0.0], [0.0, 0.0, 0.25]];
         match matrix_inverse(m_test) {
            Some(inv_test) => assert_matrix_equals(inv_test, expected_inv_test, "matrix_inverse m_test (diagonal)"),
            None => panic!("matrix_inverse m_test returned None, expected a matrix"),
        }
    }
}
